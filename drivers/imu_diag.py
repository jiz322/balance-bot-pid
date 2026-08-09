#!/usr/bin/env python3
"""
IMU diagnostic — raw euler, normalized pitch, and an accelerometer cross-check
=============================================================================
Reads ONE IMU in ONE process and prints, side by side:

  * raw euler roll/pitch/yaw straight from the driver, in degrees
  * what IMUSource.read() hands the balance loop (the production path)
  * tilt magnitude derived from the accelerometer, which needs no fusion and
    so acts as ground truth for "how far from upright is this thing"

Use it when the standalone driver demo and a control script disagree about
pitch. Hold the robot at one fixed attitude and compare the columns; then
re-run at a different --hz to see whether the poll rate is what changes the
answer.

    python3 drivers/imu_diag.py --hz 100
    python3 drivers/imu_diag.py --hz 400
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from imu_select import IMU_CHOICES, make_imu


def acc_tilt_deg(acc):
    """Angle between the sensor's Z axis and gravity, in degrees.

    Fusion-independent: at rest the accelerometer measures gravity alone, so
    this is the true tilt magnitude regardless of euler convention. Sign-free
    on purpose — it says how far from upright, not which way.
    """
    ax, ay, az = acc
    horiz = math.hypot(ax, ay)
    return math.degrees(math.atan2(horiz, az))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--imu", choices=IMU_CHOICES, default="i2c")
    p.add_argument("--hz", type=int, default=100,
                   help="driver poll rate (i2c only); try 100 vs 400")
    p.add_argument("--bus", type=int, default=1)
    p.add_argument("--addr", type=lambda s: int(s, 0), default=0x23)
    p.add_argument("--port", default=None)
    p.add_argument("--baud", type=int, default=None)
    p.add_argument("--seconds", type=float, default=0.0,
                   help="stop after N seconds (0 = until Ctrl-C)")
    args = p.parse_args()

    src = make_imu(args.imu, port=args.port, baud=args.baud,
                   bus=args.bus, addr=args.addr, hz=args.hz)
    if not src.open():
        print("[Diag] IMU open failed.")
        return 1

    print(f"[Diag] {src.label}")
    time.sleep(1.0)  # let the fusion settle before the first sample

    print()
    print("  raw euler (deg, driver order)   | production path | acc ground truth")
    print(f"  {'e[0]':>8} {'e[1]':>8} {'e[2]':>8} | {'pitch':>8} "
          f"| {'|tilt|':>7} {'accZ':>6} | {'Hz':>5}")
    print("  " + "-" * 74)

    t0 = time.monotonic()
    try:
        while True:
            euler = src.driver.data['euler']
            acc = src.driver.data['acc']
            pitch, _, _ = src.read()

            # euler is radians for i2c/usb, degrees for the uart drivers
            unit = 1.0 if args.imu in ('uart6', 'uart9') else 180.0 / math.pi
            e = [v * unit for v in euler]
            tilt = acc_tilt_deg(acc)

            agree = abs(abs(math.degrees(pitch)) - tilt)
            flag = "" if agree < 8.0 else f"   <-- disagrees by {agree:.1f} deg"

            stats = getattr(src.driver, 'get_read_stats', None)
            if stats:
                ok, err, age, last = stats()
                if err:
                    flag += f"   [{err} failed reads, last: {last}]"
                if age > 0.2:
                    flag += f"   [STALE {age:.1f}s]"

            print(f"  {e[0]:8.2f} {e[1]:8.2f} {e[2]:8.2f} "
                  f"| {math.degrees(pitch):8.2f} "
                  f"| {tilt:7.2f} {acc[2]:6.2f} "
                  f"| {src.rate_hz:5.1f}{flag}")

            if args.seconds and time.monotonic() - t0 >= args.seconds:
                break
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        src.close()

    print("\n  If |tilt| tracks the real attitude but the production pitch does")
    print("  not, the euler fusion is at fault, not the axis mapping. If a")
    print("  different euler index matches |tilt|, the pitch index is wrong.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
