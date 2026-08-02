"""
RobStride Motor Driver – Raspberry Pi 5 Version (v2)
======================================================
CAN2USB adapter over USB serial.  MIT Protocol.

Fix: Dedicated background reader thread continuously parses ALL
incoming feedback frames, so no motor response is ever lost.
The main thread only writes commands — no more reset_input_buffer().
"""

import serial
import struct
import time
import math
import os
from threading import Lock, Thread, Event

# ── Defaults ──────────────────────────────────────────────────
MOTOR_IDS  = [1, 2]
BAUD_RATE  = 921600

# Safety limits
MAX_POS    = 12.57
MAX_VEL    = 30.0
MAX_TORQUE = 6.0
MAX_KP     = 200.0
MAX_KD     = 5.0


def _tune_latency(port_path, latency_ms=1):
    """Lower USB-serial latency timer (FTDI default is 16 ms)."""
    devname = os.path.basename(port_path)
    lat_path = f'/sys/bus/usb-serial/devices/{devname}/latency_timer'
    try:
        if os.path.exists(lat_path):
            with open(lat_path, 'w') as f:
                f.write(str(latency_ms))
            print(f"[Driver] Set USB latency to {latency_ms} ms for {devname}")
    except PermissionError:
        print(f"[Driver] Could not set latency (need root or udev rule)")
    except Exception as e:
        print(f"[Driver] Latency tuning skipped: {e}")


class RobStrideDriver:
    def __init__(self, port='/dev/ttyUSB0', motor_ids=None):
        if motor_ids is None:
            motor_ids = MOTOR_IDS
        self.port = port
        self.motor_ids = motor_ids
        self.ser = None
        self._write_lock = Lock()       # Protects serial writes only
        self._state_lock = Lock()       # Protects motor_states reads

        # Motor state storage
        self.motor_states = {
            mid: {'pos': 0.0, 'vel': 0.0, 'torque': 0.0, 'updated': False}
            for mid in motor_ids
        }

        try:
            self.ser = serial.Serial(
                port,
                BAUD_RATE,
                timeout=0.001,       # Short timeout for reader loop
                exclusive=True,
            )
            print(f"[Driver] Opened {port} @ {BAUD_RATE}")
            _tune_latency(port)
        except Exception as e:
            print(f"[Driver] Open failed: {e}")
            if 'Permission' in str(e):
                print(f"[Driver] Hint: run  sudo chmod 666 {port}")
            raise

        # Start background reader thread
        self._stop_event = Event()
        self._reader_thread = Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()
        print("[Driver] Background reader thread started")

    # ── Background reader thread ──────────────────────────────

    def _reader_loop(self):
        """
        Continuously read from serial and parse ALL feedback frames.
        Runs in a dedicated thread — no data is ever discarded.
        """
        buf = bytearray()
        FRAME_MIN_LEN = 16  # AT(2) + ID(4) + DLC(1) + DATA(8) + CR LF(2) = 17, but we search from header

        while not self._stop_event.is_set():
            try:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    buf.extend(self.ser.read(waiting))
                else:
                    # Small sleep to avoid busy-spin when no data
                    time.sleep(0.0002)
                    continue

                # Parse all complete frames in buffer
                while True:
                    header_idx = buf.find(b'\x41\x54')
                    if header_idx == -1:
                        # No header found — discard stale bytes but keep last byte
                        # (in case half of 0x4154 is at the end)
                        if len(buf) > 1:
                            buf = buf[-1:]
                        break

                    # Discard any garbage before header
                    if header_idx > 0:
                        buf = buf[header_idx:]

                    # Need at least: header(2) + canid(4) + dlc(1) + data(8) + footer(2) = 17
                    if len(buf) < 17:
                        break  # Wait for more data

                    # Extract data payload (bytes 7..14)
                    data = buf[7:15]
                    current_motor_id = data[0]

                    p_int = (data[1] << 8) | data[2]
                    v_int = (data[3] << 4) | (data[4] >> 4)
                    t_int = ((data[4] & 0xF) << 8) | data[5]

                    p_val = self._uint_to_float(p_int, -12.57, 12.57, 16)
                    v_val = self._uint_to_float(v_int, -30.0, 30.0, 12)
                    t_val = self._uint_to_float(t_int, -6.0, 6.0, 12)

                    if current_motor_id in self.motor_states:
                        with self._state_lock:
                            self.motor_states[current_motor_id]['pos'] = p_val
                            self.motor_states[current_motor_id]['vel'] = v_val
                            self.motor_states[current_motor_id]['torque'] = t_val
                            self.motor_states[current_motor_id]['updated'] = True

                    # Consume this frame
                    buf = buf[17:]

            except serial.SerialException:
                if not self._stop_event.is_set():
                    print("[Driver] Serial read error in reader thread")
                break
            except Exception as e:
                if not self._stop_event.is_set():
                    print(f"[Driver] Reader error: {e}")
                time.sleep(0.001)

    # ── CAN frame packing ─────────────────────────────────────

    def _pack_std_can_frame(self, can_id, data_bytes):
        header = b'\x41\x54'
        packed_id_val = can_id << 21
        id_bytes = struct.pack('>I', packed_id_val)
        dlc = b'\x08'
        footer = b'\x0d\x0a'
        return header + id_bytes + dlc + data_bytes + footer

    def _float_to_uint(self, x, x_min, x_max, bits):
        if x > x_max:
            x = x_max
        elif x < x_min:
            x = x_min
        span = x_max - x_min
        return int((x - x_min) * ((1 << bits) - 1) / span)

    def _uint_to_float(self, x_int, x_min, x_max, bits):
        span = x_max - x_min
        return (x_int * span / ((1 << bits) - 1)) + x_min

    # ── Motor lifecycle ───────────────────────────────────────

    def enable_all(self):
        payload = b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFC'
        for mid in self.motor_ids:
            frame = self._pack_std_can_frame(mid, payload)
            with self._write_lock:
                self.ser.write(frame)
                time.sleep(0.005)
        print(f"[Driver] Motors {self.motor_ids} enabled")

    def disable_all(self):
        payload = b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFD'
        for mid in self.motor_ids:
            frame = self._pack_std_can_frame(mid, payload)
            with self._write_lock:
                self.ser.write(frame)
        print(f"[Driver] Motors {self.motor_ids} disabled")

    def set_zero_all(self):
            payload = b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFE'
            for mid in self.motor_ids:
                for attempt in range(3):
                    frame = self._pack_std_can_frame(mid, payload)
                    with self._write_lock:
                        self.ser.write(frame)
                    time.sleep(0.15)

                    # Send a no-op command to trigger a feedback response
                    self.send_command(mid, p_des=0, v_des=0, kp=0, kd=0, t_ff=0)
                    time.sleep(0.05)

                    pos, _, _ = self.get_state(mid)
                    if abs(pos) < 0.1:
                        print(f"[Driver] Motor {mid} zeroed (pos={pos:.4f})")
                        break
                    print(f"[Driver] Motor {mid} zero attempt {attempt+1} failed (pos={pos:.4f}), retrying...")
                else:
                    print(f"[Driver] WARNING: Motor {mid} failed to zero after 3 attempts (pos={pos:.4f})")

    # ── Core command (write-only, no read) ────────────────────

    def send_command(self, motor_id, p_des, v_des, kp, kd, t_ff):
        """
        Send MIT-protocol command.
        Feedback is handled by the background reader thread.
        """
        # Safety clamp
        p_des = max(min(p_des, MAX_POS), -MAX_POS)
        v_des = max(min(v_des, MAX_VEL), -MAX_VEL)
        kp    = max(min(kp, MAX_KP), 0)
        kd    = max(min(kd, MAX_KD), 0)
        t_ff  = max(min(t_ff, MAX_TORQUE), -MAX_TORQUE)

        # Pack (MIT protocol)
        p_int  = self._float_to_uint(p_des, -12.57, 12.57, 16)
        v_int  = self._float_to_uint(v_des, -50.0, 50.0, 12)
        kp_int = self._float_to_uint(kp, 0.0, 500.0, 12)
        kd_int = self._float_to_uint(kd, 0.0, 5.0, 12)
        t_int  = self._float_to_uint(t_ff, -6.0, 6.0, 12)

        buf = bytearray(8)
        buf[0] = p_int >> 8
        buf[1] = p_int & 0xFF
        buf[2] = v_int >> 4
        buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8)
        buf[4] = kp_int & 0xFF
        buf[5] = kd_int >> 4
        buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8)
        buf[7] = t_int & 0xFF

        frame = self._pack_std_can_frame(motor_id, buf)

        with self._write_lock:
            self.ser.write(frame)

    # ── State access ──────────────────────────────────────────

    def get_state(self, motor_id):
        with self._state_lock:
            s = self.motor_states[motor_id]
            return s['pos'], s['vel'], s['torque']

    def close(self):
        self._stop_event.set()
        self._reader_thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.disable_all()
            self.ser.close()
            print("[Driver] Port closed")


# ── Standalone test ───────────────────────────────────────────
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyUSB0')
    args = parser.parse_args()

    driver = RobStrideDriver(args.port, [1, 2])

    try:
        driver.set_zero_all()
        driver.enable_all()
        time.sleep(1)

        print("Dual motor sync test (10 s)...")
        t0 = time.monotonic()

        while True:
            t = time.monotonic() - t0
            if t > 10:
                break

            p1 = 0.5 * math.sin(2 * t)
            p2 = 0.5 * math.cos(2 * t)

            driver.send_command(1, p_des=p1, v_des=0, kp=10, kd=0.5, t_ff=0)
            driver.send_command(2, p_des=p2, v_des=0, kp=10, kd=0.5, t_ff=0)

            q1, dq1, _ = driver.get_state(1)
            q2, dq2, _ = driver.get_state(2)
            print(f"t={t:.2f} | M1 ref:{p1:.2f} act:{q1:.2f} | M2 ref:{p2:.2f} act:{q2:.2f}", end='\r')

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        driver.close()