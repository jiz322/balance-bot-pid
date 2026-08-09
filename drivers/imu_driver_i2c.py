import smbus2 as smbus
import struct
import time
import threading
import math

# ================= Default Configuration =================
DEFAULT_I2C_ADDR   = 0x23
DEFAULT_I2C_BUS_ID = 1
DEFAULT_LOOP_HZ    = 100

# ================= Register Addresses =================
REG_ACCEL = 0x04      # Accel Raw (int16 x3)
REG_GYRO  = 0x0A      # Gyro Raw (int16 x3)
REG_EULER = 0x26      # Euler Angles (float x3) - Roll, Pitch, Yaw
REG_ALGO  = 0x61      # Algorithm setting
REG_CALIB = 0x70      # Calibration setting


class IMUDriverI2c:
    def __init__(self, bus_id=DEFAULT_I2C_BUS_ID, addr=DEFAULT_I2C_ADDR, loop_hz=DEFAULT_LOOP_HZ):
        self.bus_id = bus_id
        self.addr = addr
        self.loop_dt = 1.0 / loop_hz
        self.loop_hz = loop_hz
        self.bus = None
        self.running = False
        self.thread = None

        # Data container (same interface as serial IMU driver)
        self.data = {
            'acc': [0.0, 0.0, 0.0],    # acceleration (g)
            'gyro': [0.0, 0.0, 0.0],   # angular velocity (rad/s)
            'euler': [0.0, 0.0, 0.0],  # euler angles (rad)
            'quat': [1.0, 0.0, 0.0, 0.0]
        }

        # Int16 raw → physical units
        self.acc_scale = 16.0 / 32768.0
        self.gyro_scale = (2000.0 / 32768.0) * (math.pi / 180.0)

        # Diagnostics
        self._loop_count = 0
        self._last_diag_time = 0.0
        self._actual_hz = 0.0
        # Counted separately from _loop_count: a failed read leaves the old
        # sample in self.data, so a healthy loop rate does not imply the data
        # is fresh. Compare these to tell a stale reading from a live one.
        self._read_ok = 0
        self._read_err = 0
        self._last_err = None
        self._last_ok_t = 0.0

    def open(self):
        try:
            self.bus = smbus.SMBus(self.bus_id)
            print(f"[IMU] I2C bus {self.bus_id} opened, addr=0x{self.addr:02X}, target={self.loop_hz}Hz")

            # Test connection
            try:
                self.bus.read_byte_data(self.addr, 0x00)
            except OSError:
                print(f"[IMU] Error: cannot reach device at 0x{self.addr:02X}, check wiring!")
                return False

            self.running = True
            self._last_diag_time = time.monotonic()
            self.thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.thread.start()
            return True
        except Exception as e:
            print(f"[IMU] Open failed: {e}")
            return False

    def close(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.bus:
            self.bus.close()
            print("[IMU] I2C closed")

    def _reader_loop(self):
        """Fixed-rate reader loop using monotonic clock."""
        t_next = time.monotonic()

        while self.running:
            t_next += self.loop_dt

            try:
                # 1. Euler angles: 12 bytes (3x float32)
                #    REG_EULER already returns radians — no conversion needed
                block_euler = self.bus.read_i2c_block_data(self.addr, REG_EULER, 12)
                r_rad, p_rad, y_rad = struct.unpack('<fff', bytes(block_euler))
                self.data['euler'] = [r_rad, p_rad, y_rad]

                # 2. Accelerometer: 6 bytes (3x int16)
                block_acc = self.bus.read_i2c_block_data(self.addr, REG_ACCEL, 6)
                ax_raw, ay_raw, az_raw = struct.unpack('<hhh', bytes(block_acc))
                self.data['acc'] = [
                    ax_raw * self.acc_scale,
                    ay_raw * self.acc_scale,
                    az_raw * self.acc_scale
                ]

                # 3. Gyroscope: 6 bytes (3x int16)
                block_gyro = self.bus.read_i2c_block_data(self.addr, REG_GYRO, 6)
                gx_raw, gy_raw, gz_raw = struct.unpack('<hhh', bytes(block_gyro))
                self.data['gyro'] = [
                    gx_raw * self.gyro_scale,
                    gy_raw * self.gyro_scale,
                    gz_raw * self.gyro_scale
                ]

                self._read_ok += 1
                self._last_ok_t = time.monotonic()

            except OSError as e:
                self._read_err += 1
                self._last_err = str(e)
            except Exception as e:
                self._read_err += 1
                self._last_err = str(e)
                print(f"[IMU] Read error: {e}")

            # Diagnostics: measure actual Hz every 5 seconds
            self._loop_count += 1
            now = time.monotonic()
            if now - self._last_diag_time >= 5.0:
                self._actual_hz = self._loop_count / (now - self._last_diag_time)
                self._loop_count = 0
                self._last_diag_time = now

            # Sleep until next scheduled tick
            sleep_time = t_next - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                t_next = time.monotonic()

    def get_actual_hz(self):
        """Return measured loop frequency (updated every 5s).

        This counts loop iterations, not successful reads. Use get_read_stats()
        to check whether the data behind it is actually fresh.
        """
        return self._actual_hz

    def get_read_stats(self):
        """(successful reads, failed reads, seconds since last good read, last error)."""
        age = (time.monotonic() - self._last_ok_t) if self._last_ok_t else float('inf')
        return self._read_ok, self._read_err, age, self._last_err

    def _write_register(self, reg, value):
        try:
            self.bus.write_byte_data(self.addr, reg, value)
            time.sleep(0.05)
            print(f"[IMU] Write reg 0x{reg:02X} = 0x{value:02X}")
        except Exception as e:
            print(f"[IMU] Write failed: {e}")

    def set_frequency(self, freq):
        """I2C mode: read rate is controlled by loop timing, not a register."""
        print(f"[IMU] I2C mode: read rate controlled by code, target {self.loop_hz}Hz")

    def set_algorithm_6axis(self):
        """Switch to 6-axis algorithm (avoids magnetometer interference)."""
        self._write_register(REG_ALGO, 0x06)

    def calibrate_acc_gyro(self):
        """Calibrate accelerometer and gyroscope. Keep device still for 3 seconds."""
        print("[IMU] Calibrating... keep still for 3 seconds")
        self._write_register(REG_CALIB, 0x01)
        time.sleep(3)
        print("[IMU] Calibration command sent")


if __name__ == "__main__":
    imu = IMUDriverI2c()

    if imu.open():
        imu.set_algorithm_6axis()
        time.sleep(1)

        print("IMU I2C ready. Move sensor to see values...")
        print("-" * 70)
        print(f"{'Time':<8} | {'Roll (deg)':<10} {'Pitch (deg)':<10} | "
              f"{'Acc Z (g)':<10} | {'Gyro Y (rad/s)':<14} | {'Hz':<6}")
        print("-" * 70)

        try:
            start_t = time.time()
            while True:
                euler = imu.data['euler']
                acc = imu.data['acc']
                gyro = imu.data['gyro']

                r_deg = math.degrees(euler[0])
                p_deg = math.degrees(euler[1])
                t = time.time() - start_t

                print(f"{t:6.2f} s | {r_deg:10.2f} {p_deg:10.2f} | "
                      f"{acc[2]:10.2f} | {gyro[1]:10.4f}     | "
                      f"{imu.get_actual_hz():5.1f}", end='\r')

                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            imu.close()