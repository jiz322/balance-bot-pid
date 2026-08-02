"""
Classical PID Balancer – MIT Mode (Direct Velocity, No Integration)
====================================================================
Motor equation (runs at motor-controller rate, ~10-40 kHz):
    τ = Kd × (v_target − v_current) + t_ff

Control law (400 Hz):
    v_target = Kp_pitch * pitch_error + Kd_pitch * pitch_rate
             + Kp_vel * (cmd_vel - avg_vel) + Ki_vel * ∫vel_err
             + pos_correction

    v_target_L = v_target + Δv_yaw
    v_target_R = v_target - Δv_yaw

This is full-state feedback (like LQR) mapped through MIT mode.
No inner-loop integration = no lag = fast response.

Usage:
    python classical_mit.py \
        --kd 0.6 \
        --pitch-kp 75 --pitch-kd 3.5 \
        --vel-kp 0.08 --vel-ki 0.01 \
        --yaw-kp 0.3 \
        --cmd-vel 1.2 \
        --cmd-yaw 1.2 \
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

# ── Hardware constants ────────────────────────────────────────
MOTOR_PORT   = '/dev/ttyUSB0'
IMU_PORT     = '/dev/ttyAMA0'
MOTOR_IDS    = [1, 2]
CONTROL_HZ   = 400
DT           = 1.0 / CONTROL_HZ

MAX_VEL_CMD   = 15.0         # rad/s
MAX_TORQUE_FF = 4.0           # Nm safety clamp on feedforward
MAX_PITCH_DEG = 30.0

LEFT_DIR  =  1
RIGHT_DIR = -1

DEG2RAD = math.pi / 180.0

# ── MIT Mode ─────────────────────────────────────────────────
HW_KP = 0.0
HW_KD = 0.8                  # Nm/(rad/s) — motor velocity gain

# ── Pitch → Velocity (DIRECT, no integration) ────────────────
PITCH_KP = 80.0              # (rad/s) / rad — primary balance gain
PITCH_KD = 4.0               # (rad/s) / (rad/s) — pitch rate damping

# ── Velocity tracking (PI → pitch offset) ────────────────────
VEL_KP = 0.04                # rad / (rad/s)
VEL_KI = 0.01                # rad / (rad)
VEL_INTEGRATOR_LIMIT = 0.10  # rad — anti-windup

# ── Position hold (P → velocity nudge) ───────────────────────
POS_KP = 0.2                 # (rad/s) / rad

# ── Yaw control ──────────────────────────────────────────────
YAW_VEL_KP  = 0.5            # (rad/s differential) / (rad/s yaw error)
YAW_VEL_KD  = 0.02           # damp yaw oscillations
YAW_FF_KP   = 0.1            # Nm / (rad/s yaw error)

PRINT_EVERY = 200             # ~2x per second at 400Hz


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
    """First-order IIR low-pass filter."""
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
    parser = argparse.ArgumentParser(description="Classical Balancer — MIT Mode (Pi 5)")
    parser.add_argument("--motor-port",  type=str, default=MOTOR_PORT)
    parser.add_argument("--imu-port",    type=str, default=IMU_PORT)
    parser.add_argument("--auto-detect", action="store_true")
    parser.add_argument("--cmd-pitch",   type=float, default=0.0,
                        help="Target pitch angle in rad (0 = upright)")
    parser.add_argument("--cmd-yaw",     type=float, default=0.0,
                        help="Target yaw rate in rad/s")
    parser.add_argument("--cmd-vel",     type=float, default=0.0,
                        help="Target avg wheel velocity in rad/s")
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
    args = parser.parse_args()

    pitch_kp   = args.pitch_kp
    pitch_kd   = args.pitch_kd
    vel_kp     = args.vel_kp
    vel_ki     = args.vel_ki
    pos_kp     = args.pos_kp
    yaw_kp     = args.yaw_kp
    hw_kd      = args.kd
    cmd_pitch  = args.cmd_pitch
    cmd_yaw    = args.cmd_yaw
    cmd_vel    = args.cmd_vel

    motor_port = args.motor_port
    if args.auto_detect:
        try:
            from pi5_usb_helper import find_motor_port
            motor_port = find_motor_port(fallback=motor_port)
        except ImportError:
            pass

    if not args.no_rt:
        _try_set_realtime()

    # ── Init hardware ─────────────────────────────────────────
    motor = RobStrideDriver(motor_port, MOTOR_IDS)
    imu   = IMUDriver6Axis(args.imu_port)

    # IMUDriver6Axis.open() auto-configures VRU 6-axis mode + 400Hz
    if not imu.open():
        print("[Main] IMU open failed, aborting.")
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

    # Low-pass filters — lower alpha for 400Hz (was tuned for 100Hz)
    # alpha_400 ≈ 1 - (1 - alpha_100)^(1/4) to match time constant
    pitch_filter      = LowPassFilter(alpha=0.12)
    pitch_rate_filter = LowPassFilter(alpha=0.12)
    vel_filter        = LowPassFilter(alpha=0.02)
    yaw_rate_filter   = LowPassFilter(alpha=0.09)

    # Torque rate limiter (Nm/step)
    MAX_TORQUE_RATE = 2.0 * DT   # 2.0 Nm/s → 0.005 Nm per step at 400Hz
    prev_tff_l = 0.0
    prev_tff_r = 0.0

    print(f"\n{'='*65}")
    print(f"  Classical PID Balancer — MIT Mode (Direct Velocity)")
    print(f"{'='*65}")
    print(f"  IMU:       {args.imu_port} @ 400Hz (YIS 6-axis, deg output)")
    print(f"  MIT Motor: Kp={HW_KP}  Kd={hw_kd}")
    print(f"  Pitch→Vel: Kp={pitch_kp:.1f}  Kd={pitch_kd:.2f}")
    print(f"  Vel→Pitch: Kp={vel_kp:.3f}  Ki={vel_ki:.3f}")
    print(f"  Pos→Vel:   Kp={pos_kp:.3f}")
    print(f"  Yaw:       Kp={yaw_kp:.2f}")
    print(f"  Commands:  pitch={cmd_pitch:.3f}  yaw={cmd_yaw:.3f}  vel={cmd_vel:.2f}")
    print(f"  Safety:    pitch ±{args.max_pitch:.0f}°  vel ±{MAX_VEL_CMD:.0f} rad/s")
    print(f"  Rate:      {CONTROL_HZ} Hz")
    print(f"  Duration:  {args.duration}s — Ctrl-C to stop")
    print(f"{'='*65}\n")

    step = 0
    t_start = time.monotonic()

    try:
        while not shutdown:
            t_loop = time.monotonic()
            elapsed = t_loop - t_start
            if elapsed > args.duration:
                print("\n[Main] Duration reached.")
                break

            # ── Read sensors ──────────────────────────────────
            # IMUDriver6Axis: euler = [pitch, roll, yaw] in DEGREES
            #                 gyro  = [gx, gy, gz]       in DEG/S
            euler = imu.data['euler']
            gyro  = imu.data['gyro']

            pos_l, vel_l, _ = motor.get_state(MOTOR_IDS[0])
            pos_r, vel_r, _ = motor.get_state(MOTOR_IDS[1])

            # Convert degrees → radians and apply sign convention
            # 400Hz driver: euler[0]=pitch, euler[1]=roll, euler[2]=yaw
            pitch      = -euler[0] * DEG2RAD
            pitch_rate = -gyro[1]  * DEG2RAD
            yaw_rate   = -gyro[2]  * DEG2RAD

            wheel_vel_l = LEFT_DIR  * vel_l
            wheel_vel_r = RIGHT_DIR * vel_r
            wheel_pos_l = LEFT_DIR  * pos_l
            wheel_pos_r = RIGHT_DIR * pos_r

            # Filtered signals
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
            # Pitch PD → v_target (DIRECT, no integration)
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

            # Rate-limit feedforward torque
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

            # Estimated total torque (for monitoring)
            total_l = hw_kd * (v_target_l - wheel_vel_l) + tff_l
            total_r = hw_kd * (v_target_r - wheel_vel_r) + tff_r

            step += 1

            # ── Live status (~10 Hz) ─────────────────────────
            if step % 40 == 0:
                print(
                    f"t={elapsed:6.2f} | "
                    f"pitch={pitch_f:+.3f}({pitch_deg:+.1f}°) "
                    f"p_off={pitch_offset:+.3f} | "
                    f"v_tgt={v_target:+.2f} "
                    f"v_cur={avg_vel_f:+.2f} | "
                    f"v_cmd=[{v_target_l:+.1f},{v_target_r:+.1f}] "
                    f"tff=[{tff_l:+.2f},{tff_r:+.2f}] "
                    f"τ≈[{total_l:+.2f},{total_r:+.2f}]",
                    end='\r',
                )

            # ── Detailed print ────────────────────────────────
            if step % args.print_every == 0:
                loop_ms = (time.monotonic() - t_loop) * 1000
                print(f"\n{'='*65}")
                print(f"  Step {step}  t={elapsed:.2f}s  loop={loop_ms:.1f}ms")
                print(f"{'='*65}")
                print(f"  pitch:       {pitch_f:+.4f} rad ({pitch_deg:+.1f}°)")
                print(f"  pitch_rate:  {pitch_rate_f:+.4f} rad/s")
                print(f"  yaw_rate:    {yaw_rate_f:+.4f} rad/s")
                print(f"  wheel_vel:   L={wheel_vel_l:+.2f}  R={wheel_vel_r:+.2f} rad/s")
                print(f"  wheel_pos:   L={wheel_pos_l:+.2f}  R={wheel_pos_r:+.2f} rad")
                print(f"  avg_vel:     {avg_vel_f:+.3f} rad/s   avg_pos: {avg_pos:+.3f} rad")
                print(f"  ---")
                print(f"  vel_error:   {vel_error:+.3f}   vel_integral: {vel_integral:+.4f}")
                print(f"  pitch_offset:{pitch_offset:+.4f} rad ({math.degrees(pitch_offset):+.2f}°)")
                print(f"  desired_pitch:{desired_pitch:+.4f} rad")
                print(f"  pitch_error: {pitch_error:+.4f} rad")
                print(f"  v_target:    {v_target:+.3f} rad/s  (= {pitch_kp}×{pitch_error:+.4f} + {pitch_kd}×{pitch_rate_f:+.4f})")
                print(f"  ---")
                print(f"  yaw_error:   {yaw_error:+.4f}   delta_v: {delta_v:+.3f}   delta_tff: {delta_tff:+.3f}")
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
        print("[Main] Done.")


if __name__ == "__main__":
    main()