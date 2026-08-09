# Balanced Wheel Robot – Control Software

Classical PID control for a two-wheel self-balancing robot on Raspberry Pi 5.
CAN2USB motor adapter and IMU connect via USB/UART (no GPIO wiring required).

An earlier RL (reinforcement-learning) policy was tried and removed from this
repo — it only stayed balanced for a few seconds and was not reliable enough
to keep as a supported path. The classical PID controllers below are the
maintained, working implementations.

## Layout

```
control/    Control loop entrypoints (the scripts you actually run)
drivers/    Hardware drivers: motor (RobStride/CAN2USB), IMU, RC receiver
tools/      Standalone diagnostics / one-off scripts
setup.sh    One-time Pi 5 setup: deps, udev rules, permissions
```

### `control/`

| File | Purpose |
|------|---------|
| `classical_mit.py` | PID balancer, 100 Hz loop, I2C IMU |
| `classical_mit_400hz.py` | PID balancer, 400 Hz loop, UART IMU (recommended — best tuning results, see below) |
| `classical_mit_rc.py` | 400 Hz PID balancer with selectable RC input (`--rc elrs\|web`) and selectable IMU (`--imu i2c\|usb\|uart6\|uart9`) |
| `test_elrs.py` | Standalone ELRS/CRSF receiver test |

### `drivers/`

| File | Purpose |
|------|---------|
| `robstride_driver.py` | RobStride motor driver over CAN2USB (MIT protocol) |
| `imu_driver.py` | IMU driver, USB serial, 100 Hz |
| `imu_driver_6_400hz.py` | Yesense YIS IMU driver, 6-axis (acc+gyro+euler), 400 Hz |
| `imu_driver_9_400hz.py` | Yesense YIS IMU driver, 9-axis (+mag), 400 Hz |
| `imu_driver_i2c.py` | IMU driver over I2C |
| `imu_select.py` | Selects an IMU driver by name and normalizes their differing units / euler axis order |
| `crsf_reader.py` | ELRS/CRSF RC receiver frame parser |
| `web_rc.py` | LAN web RC receiver — serves a touch page, reports CRSF channels (drop-in for `crsf_reader.py`) |
| `web_rc_page.html` | The touch control page served by `web_rc.py` |
| `rc_mapper.py` | Maps CRSF RC channels to balance-robot commands |

### `tools/`

| File | Purpose |
|------|---------|
| `pi5_usb_helper.py` | Scan & auto-detect USB serial ports by VID/PID |
| `try_i2c.py` | Quick I2C IMU register read test |

## Quick Start

```bash
# 1. One-time setup
chmod +x setup.sh
sudo ./setup.sh
# Reboot or re-login

# 2. Check which USB port is which
python3 tools/pi5_usb_helper.py

# 3. Run the balancer (400 Hz, recommended)
python3 control/classical_mit_400hz.py \
    --kd 0.60 --pitch-kp 75 --pitch-kd 3.5 \
    --vel-kp 0.08 --vel-ki 0.01 --pos-kp 0.0 \
    --yaw-kp 0.3 --print-every 50 \
    --cmd-yaw 1.2 --cmd-vel 0.0
```

More tuning examples (100 Hz vs 400 Hz, standing-still vs moving) are in
`note.txt`.

## IMU Selection

`control/classical_mit_rc.py` picks its IMU with `--imu`, default `i2c`.

| `--imu` | Driver | Bus | Rate | Notes |
|---------|--------|-----|------|-------|
| `i2c` (default) | `IMUDriverI2c` | I2C | polled, `--imu-hz` | `--imu-bus`, `--imu-addr` |
| `usb` | `IMUDriver` | USB serial | 100 Hz | `--imu-port`, `--imu-baud` |
| `uart6` | `IMUDriver6Axis` | UART | 400 Hz | Yesense YIS, 6-axis |
| `uart9` | `IMUDriver9Axis` | UART | 400 Hz | Yesense YIS, 9-axis (+mag) |

```bash
python3 control/classical_mit_rc.py                              # i2c, default
python3 control/classical_mit_rc.py --imu i2c --imu-addr 0x23 --imu-hz 400
python3 control/classical_mit_rc.py --imu uart6 --imu-port /dev/ttyAMA0
```

The drivers do not agree on units or axis order: the I2C and USB drivers
report euler in **radians** ordered roll/pitch/yaw, while the two UART
drivers report **degrees** ordered pitch/roll/yaw. `drivers/imu_select.py`
normalizes both, so the control loop always receives rad and rad/s.

Two consequences worth knowing:

- Swapping drivers without that adapter reads roll as pitch and scales
  radians by 57.3. Both conventions are live in the tree — `classical_mit.py`
  reads `euler[1]` raw, `classical_mit_rc.py` used to read `euler[0] * DEG2RAD`.
- `--imu-hz` is a *target*. I2C does three block reads per cycle, so 400 Hz
  may not be reachable; the detailed print shows the measured rate. If it
  sits well under the 400 Hz control loop, the pitch derivative is being fed
  repeated samples — prefer `uart6` for the fastest loop.

## Remote Control

`control/classical_mit_rc.py` takes RC input from either source. The channel
mapping, deadband, and failsafe behaviour are identical, so tuning carries
over between them.

```bash
# ELRS transmitter (default)
python3 control/classical_mit_rc.py --rc elrs --elrs-port /dev/ttyAMA2

# Phone / tablet over the private LAN — no RC hardware
python3 control/classical_mit_rc.py --rc web --web-token mysecret
```

Web mode prints the URL to open, e.g. `http://192.168.1.42:8080/?token=mysecret`.
The page is a landscape touch UI: throttle bar on the left (drag, holds
position, double-tap to zero), a self-centering pitch/yaw pad on the right,
and an ARM button. Arming is refused while the throttle is up.

Only stdlib is used — no extra packages to install. Test the RC path on its
own before wiring it to motors:

```bash
python3 drivers/web_rc.py --port 8080
```

**Failsafe.** Both sources disarm and zero the sticks after 0.5 s without
frames, so a closed tab, a backgrounded phone, or WiFi dropping stops the
robot. The browser also disarms on losing focus and after 3 failed posts.

**Security.** The web server is plain HTTP with no authentication unless you
pass `--web-token`. Anyone who can reach the Pi on the LAN can drive the
robot. Keep it on a trusted private network, use a token, and do not
port-forward it.

## Notes

- Both CAN2USB adapter and IMU connect via USB/UART — no GPIO wiring.
- Timer uses `time.monotonic()`.
- Run with `sudo` to enable `SCHED_FIFO` real-time scheduling for lower
  loop jitter, or pass `--no-rt` to skip it (where supported).

## Troubleshooting

**Permission denied on `/dev/ttyUSB*`:**
```bash
sudo chmod 666 /dev/ttyUSB0 /dev/ttyUSB1   # quick fix
# or rerun setup.sh for permanent udev rules
```

**Ports swap on reboot:**
USB port numbering can change. Use `python3 tools/pi5_usb_helper.py` to
identify, or edit VID/PID in that script for stable auto-detection.
