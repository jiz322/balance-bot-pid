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
| `classical_mit_rc.py` | 400 Hz PID balancer with ELRS/CRSF RC control input |
| `test_elrs.py` | Standalone ELRS/CRSF receiver test |

### `drivers/`

| File | Purpose |
|------|---------|
| `robstride_driver.py` | RobStride motor driver over CAN2USB (MIT protocol) |
| `imu_driver.py` | IMU driver, USB serial, 100 Hz |
| `imu_driver_6_400hz.py` | Yesense YIS IMU driver, 6-axis (acc+gyro+euler), 400 Hz |
| `imu_driver_9_400hz.py` | Yesense YIS IMU driver, 9-axis (+mag), 400 Hz |
| `imu_driver_i2c.py` | IMU driver over I2C |
| `crsf_reader.py` | ELRS/CRSF RC receiver frame parser |
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
