"""
Classical PID Balancer — MIT Mode with RC Control (ELRS or Web)
===============================================================
Motor equation (runs at motor-controller rate, ~10-40 kHz):
    τ = Kd × (v_target − v_current) + t_ff

Control law (400 Hz):
    v_target = Kp_pitch * pitch_error + Kd_pitch * pitch_rate
             + Kp_vel * (cmd_vel - avg_vel) + Ki_vel * ∫vel_err
             + pos_correction

    v_target_L = v_target + Δv_yaw
    v_target_R = v_target - Δv_yaw

RC source (--rc elrs | web) — identical channel mapping either way:
    CH3 (Throttle) → cmd_vel    0.0 to 2.0
    CH2 (Pitch/E)  → cmd_pitch  -0.3 to 0.3
    CH4 (Yaw/R)    → cmd_yaw    -2.0 to 2.0
    CH8 (Aux4)     → arm/disarm (low=still, high=active)

    elrs  ELRS/CRSF receiver on a serial port (default /dev/ttyAMA2)
    web   no RC hardware — serves a touch control page on the private LAN,
          open it on a phone or tablet. Both lose-link failsafes disarm.

Usage:
    python classical_mit_rc.py
    python classical_mit_rc.py --pitch-kp 75 --pitch-kd 3.5 --kd 0.6
    python classical_mit_rc.py --elrs-port /dev/ttyAMA2
    python classical_mit_rc.py --rc web --web-port 8080
    python classical_mit_rc.py --rc web --web-token mysecret
"""

import argparse
import time
import math
import signal
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "drivers"))

from robstride_driver import RobStrideDriver
from imu_driver_6_400hz import IMUDriver6Axis
from crsf_reader import CRSFReader
from web_rc import WebRCReader
from rc_mapper import RCMapper

# ── Hardware constants ────────────────────────────────────────
MOTOR_PORT   = '/dev/ttyUSB0'
IMU_PORT     = '/dev/ttyAMA0'
ELRS_PORT    = '/dev/ttyAMA2'
WEB_HOST     = '0.0.0.0'
WEB_PORT     = 8080
MOTOR_IDS    = [1, 2]
CONTROL_HZ   = 400
DT           = 1.0 / CONTROL_HZ

RC_DEADBAND  = 30    # CRSF counts, shared by RCMapper and the web stick encoder

MAX_VEL_CMD   = 15.0
MAX_TORQUE_FF = 4.0
MAX_PITCH_DEG = 30.0

LEFT_DIR  =  1
RIGHT_DIR = -1

DEG2RAD = math.pi / 180.0

# ── MIT Mode ─────────────────────────────────────────────────
HW_KP = 0.0
HW_KD = 0.6

# ── Pitch → Velocity ─────────────────────────────────────────
PITCH_KP = 75.0
PITCH_KD = 3.5

# ── Velocity tracking ────────────────────────────────────────
VEL_KP = 0.08
VEL_KI = 0.01
VEL_INTEGRATOR_LIMIT = 0.10

# ── Position hold ─────────────────────────────────────────────
POS_KP = 0.0

# ── Yaw control ──────────────────────────────────────────────
YAW_VEL_KP  = 0.3
YAW_VEL_KD  = 0.02
YAW_FF_KP   = 0.1

PRINT_EVERY = 200


def _try_set_realtime():
    try:
        os.sched_setaffinity(0, {2})
        param = os.sched_param(50)
        os.sched_setscheduler(0, os.SCHED_FIFO, param)
        print("[Main] Real-time scheduling enabled (SCHED_FIFO, core 2)")
    except PermissionError:
        print("[Main] No RT permissions — running with normal priority")
    except Exception as e:
        print(f"[Main] RT scheduling skipped: {e}")


class LowPassFilter:
    def __init__(self, alpha=0.15):
        self.alpha = alpha
        self.value = 0.0
        self.initialized = False

    def update(self, raw):
        if not self.initialized:
            self.value = raw
            self.initialized = True
        else:
            self.value = self.alpha * raw + (1.0 - self.alpha) * self.value
        return self.value


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def main():
    parser = argparse.ArgumentParser(description="Classical Balancer — MIT Mode + RC (ELRS or Web)")
    parser.add_argument("--motor-port",  type=str, default=MOTOR_PORT)
    parser.add_argument("--imu-port",    type=str, default=IMU_PORT)
    parser.add_argument("--elrs-port",   type=str, default=ELRS_PORT)
    # RC source
    parser.add_argument("--rc",          type=str, default="elrs",
                        choices=["elrs", "web"],
                        help="RC input: ELRS receiver, or LAN web page (no RC hardware)")
    parser.add_argument("--web-host",    type=str, default=WEB_HOST)
    parser.add_argument("--web-port",    type=int, default=WEB_PORT)
    parser.add_argument("--web-token",   type=str, default=None,
                        help="shared secret required to send web RC commands")
    parser.add_argument("--rc-wait",     type=float, default=None,
                        help="seconds to wait for RC link before starting")
    parser.add_argument("--auto-detect", action="store_true")
    parser.add_argument("--duration",    type=float, default=6000.0)
    parser.add_argument("--no-rt",       action="store_true")
    parser.add_argument("--max-pitch",   type=float, default=MAX_PITCH_DEG)
    parser.add_argument("--print-every", type=int, default=PRINT_EVERY)
    # MIT mode
    parser.add_argument("--kd",          type=float, default=HW_KD)
    # Pitch balance
    parser.add_argument("--pitch-kp",    type=float, default=PITCH_KP)
    parser.add_argument("--pitch-kd",    type=float, default=PITCH_KD)
    # Velocity
    parser.add_argument("--vel-kp",      type=float, default=VEL_KP)
    parser.add_argument("--vel-ki",      type=float, default=VEL_KI)
    # Position
    parser.add_argument("--pos-kp",      type=float, default=POS_KP)
    # Yaw
    parser.add_argument("--yaw-kp",      type=float, default=YAW_VEL_KP)
    # RC ranges
    parser.add_argument("--max-vel",     type=float, default=2.0)
    parser.add_argument("--max-yaw",     type=float, default=2.0)
    parser.add_argument("--max-cmd-pitch", type=float, default=0.3)
    args = parser.parse_args()

    pitch_kp = args.pitch_kp
    pitch_kd = args.pitch_kd
    vel_kp   = args.vel_kp
    vel_ki   = args.vel_ki
    pos_kp   = args.pos_kp
    yaw_kp   = args.yaw_kp
    hw_kd    = args.kd

    motor_port = args.motor_port
    if args.auto_detect:
        try:
            from pi5_usb_helper import find_motor_port
            motor_port = find_motor_port(fallback=motor_port)
        except ImportError:
            pass

    if not args.no_rt:
        _try_set_realtime()

    # ── Init RC source ────────────────────────────────────────
    if args.rc == "web":
        rc_link = WebRCReader(host=args.web_host, port=args.web_port,
                              deadband=RC_DEADBAND, token=args.web_token)
        rc_link.start()
        rc_source = f"web {rc_link.url}"
        rc_wait = args.rc_wait if args.rc_wait is not None else 120.0
        print(f"[RC] Web RC serving on {rc_link.url}")
        if not args.web_token:
            print("[RC] WARNING: no --web-token — anyone on this LAN can drive it")
        print("[RC] Open that URL on your phone or tablet...")
    else:
        rc_link = CRSFReader(args.elrs_port)
        rc_link.start()
        rc_source = f"elrs {args.elrs_port} @ 420000 baud"
        rc_wait = args.rc_wait if args.rc_wait is not None else 10.0
        print(f"[RC] ELRS reader started on {args.elrs_port}")
        print(f"[RC] Waiting for transmitter link...")

    rc = RCMapper(
        vel_range=(0.0, args.max_vel),
        pitch_range=(-args.max_cmd_pitch, args.max_cmd_pitch),
        yaw_range=(-args.max_yaw, args.max_yaw),
        deadband=RC_DEADBAND,
    )

    # Wait for the RC link, then start regardless (we start disarmed anyway)
    t_wait = time.monotonic()
    while time.monotonic() - t_wait < rc_wait:
        _, connected = rc_link.get_channels()
        if connected:
            print(f"[RC] Link up! ({time.monotonic() - t_wait:.1f}s)")
            break
        time.sleep(0.1)
    else:
        print(f"[RC] WARNING: No RC link after {rc_wait:.0f}s — "
              f"starting anyway (will be disarmed)")

    # ── Init hardware ─────────────────────────────────────────
    motor = RobStrideDriver(motor_port, MOTOR_IDS)
    imu   = IMUDriver6Axis(args.imu_port)

    if not imu.open():
        print("[Main] IMU open failed, aborting.")
        rc_link.stop()
        return

    time.sleep(0.5)

    motor.set_zero_all()
    motor.enable_all()
    time.sleep(0.5)

    # ── Shutdown handler ──────────────────────────────────────
    shutdown = False
    def _sig_handler(sig, frame):
        nonlocal shutdown
        shutdown = True
    signal.signal(signal.SIGINT,  _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    # ── State variables ───────────────────────────────────────
    vel_integral   = 0.0
    pitch_offset   = 0.0
    prev_yaw_rate  = 0.0
    was_armed      = False

    pitch_filter      = LowPassFilter(alpha=0.12)
    pitch_rate_filter = LowPassFilter(alpha=0.12)
    vel_filter        = LowPassFilter(alpha=0.02)
    yaw_rate_filter   = LowPassFilter(alpha=0.09)

    MAX_TORQUE_RATE = 2.0 * DT
    prev_tff_l = 0.0
    prev_tff_r = 0.0

    print(f"\n{'='*65}")
    print(f"  Classical PID Balancer — MIT Mode + {args.rc.upper()} RC")
    print(f"{'='*65}")
    print(f"  IMU:       {args.imu_port} @ 400Hz")
    print(f"  RC:        {rc_source}")
    print(f"  MIT Motor: Kp={HW_KP}  Kd={hw_kd}")
    print(f"  Pitch→Vel: Kp={pitch_kp:.1f}  Kd={pitch_kd:.2f}")
    print(f"  Vel→Pitch: Kp={vel_kp:.3f}  Ki={vel_ki:.3f}")
    print(f"  Pos→Vel:   Kp={pos_kp:.3f}")
    print(f"  Yaw:       Kp={yaw_kp:.2f}")
    print(f"  RC ranges: vel=[0, {args.max_vel}]  "
          f"pitch=[±{args.max_cmd_pitch}]  yaw=[±{args.max_yaw}]")
    print(f"  RC map:    CH3→vel  CH2→pitch  CH4→yaw  CH8→arm")
    print(f"  Failsafe:  disarm after 0.5s without RC frames")
    print(f"  Safety:    pitch ±{args.max_pitch:.0f}°  vel ±{MAX_VEL_CMD:.0f} rad/s")
    print(f"  Rate:      {CONTROL_HZ} Hz")
    print(f"  Duration:  {args.duration}s — Ctrl-C to stop")
    print(f"{'='*65}")
    arm_hint = ("Tap ARM on the web page to ARM" if args.rc == "web"
                else "Flip CH8 (Aux4) high to ARM")
    print(f"\n  >>> {arm_hint} <<<\n")

    step = 0
    t_start = time.monotonic()

    try:
        while not shutdown:
            t_loop = time.monotonic()
            elapsed = t_loop - t_start
            if elapsed > args.duration:
                print("\n[Main] Duration reached.")
                break

            # ── Read RC ───────────────────────────────────────
            channels, rc_connected = rc_link.get_channels()
            cmds = rc.map(channels)
            armed = cmds['armed'] and rc_connected

            # Arm/disarm transitions
            if armed and not was_armed:
                print(f"\n[RC] >>> ARMED — motors active <<<")
                # Reset integrators on arm
                vel_integral = 0.0
                pitch_offset = 0.0
                prev_tff_l = 0.0
                prev_tff_r = 0.0
            elif not armed and was_armed:
                print(f"\n[RC] >>> DISARMED — motors idle <<<")
            was_armed = armed

            if armed:
                cmd_vel   = cmds['cmd_vel']
                cmd_pitch = cmds['cmd_pitch']
                cmd_yaw   = cmds['cmd_yaw']
            else:
                cmd_vel   = 0.0
                cmd_pitch = 0.0
                cmd_yaw   = 0.0
                # Send zero commands and skip control loop
                motor.send_command(MOTOR_IDS[0], 0, 0, 0, 0, 0)
                motor.send_command(MOTOR_IDS[1], 0, 0, 0, 0, 0)
                # Reset state
                vel_integral = 0.0
                pitch_offset = 0.0
                prev_tff_l = 0.0
                prev_tff_r = 0.0

                if step % 400 == 0:
                    rc_status = "linked" if rc_connected else "NO LINK"
                    print(f"\r  [DISARMED] t={elapsed:.1f}s  "
                          f"RC={rc_status}  CH8={channels[7]:4d}  "
                          f"{arm_hint}     ", end='', flush=True)

                step += 1
                t_end = time.monotonic()
                sleep_time = DT - (t_end - t_loop)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                continue

            # ── Read sensors ──────────────────────────────────
            euler = imu.data['euler']
            gyro  = imu.data['gyro']

            pos_l, vel_l, _ = motor.get_state(MOTOR_IDS[0])
            pos_r, vel_r, _ = motor.get_state(MOTOR_IDS[1])

            pitch      = -euler[0] * DEG2RAD
            pitch_rate = -gyro[1]  * DEG2RAD
            yaw_rate   = -gyro[2]  * DEG2RAD

            wheel_vel_l = LEFT_DIR  * vel_l
            wheel_vel_r = RIGHT_DIR * vel_r
            wheel_pos_l = LEFT_DIR  * pos_l
            wheel_pos_r = RIGHT_DIR * pos_r

            pitch_f      = pitch_filter.update(pitch)
            pitch_rate_f = pitch_rate_filter.update(pitch_rate)
            avg_vel      = 0.5 * (wheel_vel_l + wheel_vel_r)
            avg_vel_f    = vel_filter.update(avg_vel)
            avg_pos      = 0.5 * (wheel_pos_l + wheel_pos_r)
            yaw_rate_f   = yaw_rate_filter.update(yaw_rate)

            # ── Safety check ──────────────────────────────────
            pitch_deg = math.degrees(pitch_f)
            if abs(pitch_deg) > args.max_pitch:
                print(f"\n[SAFETY] Pitch {pitch_deg:+.1f}° exceeds ±{args.max_pitch:.0f}° — shutdown!")
                break

            # ══════════════════════════════════════════════════
            # Velocity PI → pitch_offset
            # ══════════════════════════════════════════════════
            pos_correction = 0.0
            if abs(cmd_vel) < 0.01:
                pos_correction = clamp(-pos_kp * avg_pos, -1.0, 1.0)

            effective_vel_cmd = cmd_vel + pos_correction
            vel_error = effective_vel_cmd - avg_vel_f

            vel_integral += vel_error * DT
            max_int = VEL_INTEGRATOR_LIMIT / max(vel_ki, 1e-6)
            vel_integral = clamp(vel_integral, -max_int, max_int)

            pitch_offset = vel_kp * vel_error + vel_ki * vel_integral
            pitch_offset = clamp(pitch_offset, -0.10, 0.10)

            # ══════════════════════════════════════════════════
            # Pitch PD → v_target
            # ══════════════════════════════════════════════════
            desired_pitch = cmd_pitch + pitch_offset
            pitch_error   = pitch_f - desired_pitch

            v_target = pitch_kp * pitch_error + pitch_kd * pitch_rate_f

            # ══════════════════════════════════════════════════
            # Yaw — differential velocity + torque
            # ══════════════════════════════════════════════════
            yaw_error = cmd_yaw - yaw_rate_f
            yaw_deriv = (yaw_rate_f - prev_yaw_rate) / DT
            prev_yaw_rate = yaw_rate_f

            delta_v = clamp(yaw_kp * yaw_error + YAW_VEL_KD * (-yaw_deriv), -3.0, 3.0)
            delta_tff = clamp(YAW_FF_KP * yaw_error, -1.0, 1.0)

            # ══════════════════════════════════════════════════
            # Compose per-wheel commands
            # ══════════════════════════════════════════════════
            v_target_l = clamp(v_target + delta_v, -MAX_VEL_CMD, MAX_VEL_CMD)
            v_target_r = clamp(v_target - delta_v, -MAX_VEL_CMD, MAX_VEL_CMD)

            tff_l = delta_tff
            tff_r = -delta_tff

            tff_l = clamp(tff_l, prev_tff_l - MAX_TORQUE_RATE, prev_tff_l + MAX_TORQUE_RATE)
            tff_r = clamp(tff_r, prev_tff_r - MAX_TORQUE_RATE, prev_tff_r + MAX_TORQUE_RATE)
            tff_l = clamp(tff_l, -MAX_TORQUE_FF, MAX_TORQUE_FF)
            tff_r = clamp(tff_r, -MAX_TORQUE_FF, MAX_TORQUE_FF)
            prev_tff_l = tff_l
            prev_tff_r = tff_r

            # ══════════════════════════════════════════════════
            # Send to motors
            # ══════════════════════════════════════════════════
            motor.send_command(MOTOR_IDS[0], p_des=0,
                               v_des=LEFT_DIR  * v_target_l,
                               kp=HW_KP, kd=hw_kd,
                               t_ff=LEFT_DIR  * tff_l)
            motor.send_command(MOTOR_IDS[1], p_des=0,
                               v_des=RIGHT_DIR * v_target_r,
                               kp=HW_KP, kd=hw_kd,
                               t_ff=RIGHT_DIR * tff_r)

            total_l = hw_kd * (v_target_l - wheel_vel_l) + tff_l
            total_r = hw_kd * (v_target_r - wheel_vel_r) + tff_r

            step += 1

            # ── Live status (~10 Hz) ─────────────────────────
            if step % 40 == 0:
                print(
                    f"t={elapsed:6.2f} | "
                    f"RC: v={cmd_vel:.2f} p={cmd_pitch:+.3f} y={cmd_yaw:+.2f} | "
                    f"pitch={pitch_deg:+.1f}° "
                    f"v_tgt={v_target:+.2f} "
                    f"v_cur={avg_vel_f:+.2f} | "
                    f"τ≈[{total_l:+.2f},{total_r:+.2f}]",
                    end='\r',
                )

            # ── Detailed print ────────────────────────────────
            if step % args.print_every == 0:
                loop_ms = (time.monotonic() - t_loop) * 1000
                print(f"\n{'='*65}")
                print(f"  Step {step}  t={elapsed:.2f}s  loop={loop_ms:.1f}ms")
                print(f"{'='*65}")
                print(f"  RC:  armed={armed}  connected={rc_connected}")
                print(f"       cmd_vel={cmd_vel:.3f}  cmd_pitch={cmd_pitch:+.4f}  "
                      f"cmd_yaw={cmd_yaw:+.3f}")
                print(f"       CH: [{channels[0]:4d} {channels[1]:4d} {channels[2]:4d} "
                      f"{channels[3]:4d} ... {channels[7]:4d}]")
                print(f"  pitch:       {pitch_f:+.4f} rad ({pitch_deg:+.1f}°)")
                print(f"  pitch_rate:  {pitch_rate_f:+.4f} rad/s")
                print(f"  yaw_rate:    {yaw_rate_f:+.4f} rad/s")
                print(f"  wheel_vel:   L={wheel_vel_l:+.2f}  R={wheel_vel_r:+.2f} rad/s")
                print(f"  wheel_pos:   L={wheel_pos_l:+.2f}  R={wheel_pos_r:+.2f} rad")
                print(f"  avg_vel:     {avg_vel_f:+.3f} rad/s   avg_pos: {avg_pos:+.3f} rad")
                print(f"  ---")
                print(f"  vel_error:   {vel_error:+.3f}   vel_integral: {vel_integral:+.4f}")
                print(f"  pitch_offset:{pitch_offset:+.4f} rad")
                print(f"  desired_pitch:{desired_pitch:+.4f} rad")
                print(f"  pitch_error: {pitch_error:+.4f} rad")
                print(f"  v_target:    {v_target:+.3f} rad/s")
                print(f"  ---")
                print(f"  yaw_error:   {yaw_error:+.4f}   delta_v: {delta_v:+.3f}")
                print(f"  ---")
                print(f"  v_des:       L={v_target_l:+.2f}  R={v_target_r:+.2f} rad/s")
                print(f"  t_ff:        L={tff_l:+.3f}  R={tff_r:+.3f} Nm")
                print(f"  τ_estimated: L={total_l:+.3f}  R={total_r:+.3f} Nm")
                print(f"  IMU rate:    {imu.fps:.1f} Hz")
                print(f"{'='*65}")

            # ── Sleep to maintain 400 Hz ──────────────────────
            t_end = time.monotonic()
            sleep_time = DT - (t_end - t_loop)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except Exception as e:
        print(f"\n[Main] Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n[Main] Shutting down...")
        motor.send_command(MOTOR_IDS[0], 0, 0, 0, 0, 0)
        motor.send_command(MOTOR_IDS[1], 0, 0, 0, 0, 0)
        time.sleep(0.05)
        motor.disable_all()
        motor.close()
        imu.close()
        rc_link.stop()
        print("[Main] Done.")


if __name__ == "__main__":
    main()
