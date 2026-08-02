"""
IMU Driver – Raspberry Pi 5 Version
====================================
Reads acceleration, gyroscope, and euler angles over USB serial.
Protocol-compatible with the Windows version; changes:
  - Default port: /dev/ttyUSB1 (instead of COM5)
  - Uses monotonic clock for timing
  - Adds optional latency tuning for FTDI/CH340 chips
"""

import serial
import struct
import time
import threading
import math
import os

# ── Defaults ──────────────────────────────────────────────────
IMU_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200


def _tune_latency(port_path, latency_ms=1):
    """
    Lower the USB-serial latency timer (default 16 ms on FTDI).
    Requires the port device name, e.g. 'ttyUSB1'.
    Needs root or udev rule to work.
    """
    devname = os.path.basename(port_path)
    lat_path = f'/sys/bus/usb-serial/devices/{devname}/latency_timer'
    try:
        if os.path.exists(lat_path):
            with open(lat_path, 'w') as f:
                f.write(str(latency_ms))
            print(f"[IMU] Set USB latency to {latency_ms} ms for {devname}")
    except PermissionError:
        print(f"[IMU] Could not set latency (need root or udev rule for {lat_path})")
    except Exception as e:
        print(f"[IMU] Latency tuning skipped: {e}")


class IMUDriver:
    def __init__(self, port=IMU_PORT, baudrate=BAUD_RATE):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        self.thread = None

        # Data container
        self.data = {
            'acc':   [0.0, 0.0, 0.0],        # acceleration (g)
            'gyro':  [0.0, 0.0, 0.0],        # angular velocity (rad/s)
            'euler': [0.0, 0.0, 0.0],        # euler angles (rad) Roll/Pitch/Yaw
            'quat':  [1.0, 0.0, 0.0, 0.0],   # quaternion
        }

        # Pre-computed scale factors (per protocol doc)
        self.acc_scale  = 16.0 / 32767.0
        self.gyro_scale = (2000.0 / 32767.0) * (math.pi / 180.0)

    def open(self):
        try:
            self.ser = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.1,
                exclusive=True,          # Prevent concurrent access on Linux
            )
            print(f"[IMU] Opened {self.port} @ {self.baudrate}")

            # Try to lower USB latency for better real-time performance
            _tune_latency(self.port)

            # Start reader thread
            self.running = True
            self.thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.thread.start()
            return True

        except serial.SerialException as e:
            print(f"[IMU] Open failed: {e}")
            if 'Permission' in str(e):
                print(f"[IMU] Hint: run  sudo chmod 666 {self.port}")
                print(f"[IMU]   or add a udev rule (see setup_udev.sh)")
            return False

    def close(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[IMU] Port closed")

    # ── Reader thread ─────────────────────────────────────────

    def _reader_loop(self):
        """Background thread: continuously read and parse IMU frames."""
        buffer = b''
        while self.running:
            try:
                waiting = self.ser.in_waiting
                if waiting:
                    buffer += self.ser.read(waiting)

                # Protocol: header 0x7E 0x23
                while len(buffer) >= 4:
                    # Find header
                    if buffer[0] != 0x7E or buffer[1] != 0x23:
                        buffer = buffer[1:]
                        continue

                    pkt_len = buffer[2]

                    # Wait for full frame
                    if len(buffer) < pkt_len:
                        break

                    frame  = buffer[:pkt_len]
                    buffer = buffer[pkt_len:]

                    # Checksum: sum of all bytes except last, masked to 8 bits
                    calc_sum = sum(frame[:-1]) & 0xFF
                    if calc_sum == frame[-1]:
                        self._parse_frame(frame)

                time.sleep(0.0005)  # 0.5 ms – lighter than Windows 1 ms

            except Exception as e:
                if self.running:
                    print(f"[IMU] Reader error: {e}")
                    time.sleep(0.5)

    # ── Frame parser ──────────────────────────────────────────

    def _parse_frame(self, frame):
        func = frame[3]
        payload = frame[4:-1]

        try:
            # 0x04: Raw sensor data (Acc + Gyro + Mag)
            if func == 0x04 and len(payload) >= 18:
                vals = struct.unpack('<hhhhhhhhh', payload[:18])
                self.data['acc'] = [
                    vals[0] * self.acc_scale,
                    vals[1] * self.acc_scale,
                    vals[2] * self.acc_scale,
                ]
                self.data['gyro'] = [
                    vals[3] * self.gyro_scale,
                    vals[4] * self.gyro_scale,
                    vals[5] * self.gyro_scale,
                ]

            # 0x26: Euler angles (Roll, Pitch, Yaw) as floats
            elif func == 0x26 and len(payload) >= 12:
                r, p, y = struct.unpack('<fff', payload[:12])
                self.data['euler'] = [r, p, y]

        except Exception as e:
            print(f"[IMU] Parse error: {e}")

    # ── Commands ──────────────────────────────────────────────

    def _send_cmd(self, func_code, param1, param2=0x5F):
        header = [0x7E, 0x23]
        length = 0x07
        data_to_sum = [0x7E, 0x23, length, func_code, param1, param2]
        checksum = sum(data_to_sum) & 0xFF
        cmd = struct.pack('BBBBBBB', 0x7E, 0x23, length, func_code,
                          param1, param2, checksum)
        if self.ser and self.ser.is_open:
            self.ser.write(cmd)
            time.sleep(0.1)
            print(f"[IMU] Sent cmd 0x{func_code:02X} param 0x{param1:02X}")

    def set_frequency(self, freq):
        """Set output frequency (10–100 Hz)."""
        if 10 <= freq <= 100:
            self._send_cmd(0x60, freq)
        else:
            print("[IMU] Frequency must be 10–100")

    def set_algorithm_6axis(self):
        """Switch to 6-axis algorithm (no magnetometer)."""
        self._send_cmd(0x61, 0x06)

    def calibrate_acc_gyro(self):
        """Calibrate accelerometer + gyro (keep still & level)."""
        print("[IMU] Calibrating... keep still for 3 s")
        self._send_cmd(0x70, 0x01)
        time.sleep(3)
        print("[IMU] Calibration command sent")


# ── Standalone test ───────────────────────────────────────────
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default=IMU_PORT)
    parser.add_argument('--baud', type=int, default=BAUD_RATE)
    args = parser.parse_args()

    imu = IMUDriver(args.port, args.baud)

    if imu.open():
        imu.set_frequency(100)
        imu.set_algorithm_6axis()
        time.sleep(1)

        print("IMU ready. Move the sensor to see values...")
        print("-" * 65)
        print(f"{'Time':<8} | {'Roll°':<10} {'Pitch°':<10} | {'AccZ (g)':<10} | {'GyroY (r/s)':<12}")
        print("-" * 65)

        try:
            t0 = time.monotonic()
            while True:
                euler = imu.data['euler']
                acc   = imu.data['acc']
                gyro  = imu.data['gyro']
                t = time.monotonic() - t0
                print(
                    f"{t:6.2f} s | "
                    f"{math.degrees(euler[0]):10.2f} {math.degrees(euler[1]):10.2f} | "
                    f"{acc[2]:10.2f} | {gyro[1]:10.4f}",
                    end='\r',
                )
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            imu.close()
