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

IMU source (--imu i2c | usb | uart6 | uart9), default i2c:
    i2c    IMUDriverI2c    on the I2C bus (--imu-bus / --imu-addr / --imu-hz)
    usb    IMUDriver       USB serial, 100 Hz
    uart6  IMUDriver6Axis  Yesense YIS UART, 6-axis, 400 Hz
    uart9  IMUDriver9Axis  Yesense YIS UART, 9-axis, 400 Hz

    drivers/imu_select.py normalizes their differing units and euler axis
    order, so pitch and rates reach the loop in rad and rad/s either way.

Usage:
    python classical_mit_rc.py
    python classical_mit_rc.py --pitch-kp 75 --pitch-kd 3.5 --kd 0.6
    python classical_mit_rc.py --elrs-port /dev/ttyAMA2
    python classical_mit_rc.py --rc web --web-port 8080
    python classical_mit_rc.py --rc web --web-token mysecret
    python classical_mit_rc.py --imu uart6 --imu-port /dev/ttyAMA0
    python classical_mit_rc.py --imu i2c --imu-addr 0x23 --imu-hz 400

Bench testing with no hardware:
    Motors or an IMU that will not open are replaced by stubs (drivers/
    hw_stubs.py) with a warning, so the RC page, arming and mapping stay
    testable on a bare Pi. Stub mode has no balance physics — it says
    nothing about whether a gain is stable. Pass --require-hw to abort on
    a missing device instead, which is what a real run should use.
"""

import argparse
import time
import math
import signal
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "drivers"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from debug_log import DebugLogger

from robstride_driver import RobStrideDriver
from imu_select import IMU_CHOICES, make_imu
from hw_stubs import StubIMU, StubMotor
from crsf_reader import CRSFReader
from web_rc import WebRCReader
from rc_mapper import RCMapper

# ── Hardware constants ────────────────────────────────────────
MOTOR_PORT   = '/dev/ttyUSB0'
IMU_I2C_BUS  = 1
IMU_I2C_ADDR = 0x23
ELRS_PORT    = '/dev/ttyAMA2'
WEB_HOST     = '0.0.0.0'
WEB_PORT     = 8080
MOTOR_IDS    = [0, 1]
CONTROL_HZ   = 400
DT           = 1.0 / CONTROL_HZ

RC_DEADBAND  = 30    # CRSF counts, shared by RCMapper and the web stick encoder

# Column order for --log. Must match the DebugLogger.log() call in the loop.
LOG_FIELDS = [
    't', 'armed', 'tripped', 'rc_ok',
    'cmd_vel', 'cmd_pitch', 'cmd_yaw',
    # pitch_deg is trim-corrected; pitch_raw_deg is what the IMU reported, so
    # a wrong --pitch-trim can be diagnosed and re-derived from an old log.
    'pitch_deg', 'pitch_raw_deg', 'pitch_f', 'pitch_rate_f', 'yaw_rate_f',
    'avg_vel', 'avg_vel_f', 'avg_pos',
    'vel_err', 'vel_integral', 'pitch_offset',
    'desired_pitch', 'pitch_error', 'v_target',
    'delta_v', 'delta_tff',
    'v_target_l', 'v_target_r', 'tff_l', 'tff_r',
    'wheel_vel_l', 'wheel_vel_r', 'total_l', 'total_r',
    'loop_ms',
]

MAX_VEL_CMD   = 15.0
MAX_TORQUE_FF = 4.0
MAX_PITCH_DEG = 30.0

# Reading the IMU reports when the robot is actually balanced, in degrees.
# Combines IMU mounting tilt with any centre-of-mass offset. Subtracted from
# every pitch sample, so downstream "pitch = 0" means upright, and the ±
# MAX_PITCH_DEG trip is measured from true vertical.
#
# To measure: hold the bot at balance, read the printed pitch, put that number
# here. If it reads +0.8° when balanced, this is +0.8. Getting the sign wrong
# doubles the lean instead of removing it, so check that the printed pitch
# lands near 0.0° at balance after changing it.
PITCH_TRIM_DEG = 0.8

LEFT_DIR  = -1
RIGHT_DIR = 1

# ── MIT Mode ─────────────────────────────────────────────────
HW_KP = 0.0
HW_KD = 0.6

# ── Pitch → Velocity ─────────────────────────────────────────
# Tuned down from 75/3.5, which were set against a 400 Hz IMU. A 100 Hz IMU
# holds each sample for 4 control iterations, adding ~4 ms of pure delay that
# ate the remaining phase margin: logs showed a 4.8 Hz oscillation growing
# ~9%/cycle. Neutral stability needs only ~x0.92; this is x0.73 for margin.
PITCH_KP = 55.0
PITCH_KD = 2.6

# ── Velocity tracking ────────────────────────────────────────
VEL_KP = 0.08
VEL_KI = 0.01
VEL_INTEGRATOR_LIMIT = 0.10

# Ceiling on the lean angle the velocity loop may ask for, in radians.
# The velocity PI must not out-authority the balance loop it sits on top of:
# at the old ±0.10 (5.7°) a saturated integrator commanded maximum lean into
# a bot that was already falling. 0.035 rad is ~2°.
PITCH_OFFSET_LIMIT = 0.035

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
    parser.add_argument("--elrs-port",   type=str, default=ELRS_PORT)
    # IMU source
    parser.add_argument("--imu",         type=str, default="i2c",
                        choices=list(IMU_CHOICES),
                        help="IMU driver: i2c (default), usb, uart6, uart9")
    parser.add_argument("--imu-port",    type=str, default=None,
                        help="serial device for usb/uart6/uart9 "
                             "(default: per-driver, ignored for i2c)")
    parser.add_argument("--imu-baud",    type=int, default=None,
                        help="serial baud override for usb/uart6/uart9")
    parser.add_argument("--imu-bus",     type=int, default=IMU_I2C_BUS,
                        help="I2C bus number (i2c only)")
    parser.add_argument("--imu-addr",    type=lambda v: int(v, 0),
                        default=IMU_I2C_ADDR,
                        help="I2C device address, e.g. 0x23 (i2c only)")
    parser.add_argument("--imu-hz",      type=int, default=CONTROL_HZ,
                        help="I2C polling rate target (i2c only)")
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
    parser.add_argument("--require-hw", action="store_true",
                        help="abort if motors or IMU fail to open, instead of "
                             "falling back to stubs")
    parser.add_argument("--auto-detect", action="store_true")
    parser.add_argument("--duration",    type=float, default=6000.0)
    parser.add_argument("--no-rt",       action="store_true")
    parser.add_argument("--max-pitch",   type=float, default=MAX_PITCH_DEG)
    parser.add_argument("--pitch-trim",  type=float, default=PITCH_TRIM_DEG,
                        metavar='DEG',
                        help="pitch the IMU reads when actually balanced, in "
                             "degrees; subtracted from every sample "
                             f"(default: {PITCH_TRIM_DEG:+.2f})")
    parser.add_argument("--print-every", type=int, default=PRINT_EVERY)
    # Debug logging
    parser.add_argument("--log", nargs='?', const='auto', default=None,
                        metavar='PATH',
                        help="log every loop iteration to CSV for debugging. "
                             "Bare --log writes logs/balance_<timestamp>.csv")
    parser.add_argument("--log-hz", type=float, default=float(CONTROL_HZ),
                        help="rows per second to log (default: every "
                             "iteration). Lower it for long runs.")
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
    pitch_trim = math.radians(args.pitch_trim)

    motor_port = args.motor_port
    if args.auto_detect:
        try:
            from pi5_usb_helper import find_motor_port
            motor_port = find_motor_port(fallback=motor_port)
        except ImportError:
            pass

    if args.imu == "i2c" and args.imu_port:
        print("[IMU] Note: --imu-port is ignored for the i2c driver "
              "(use --imu-bus / --imu-addr)")

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
    # A device that will not open becomes a stub so the RC path stays
    # testable on the bench, unless --require-hw asks to fail fast.
    try:
        motor = RobStrideDriver(motor_port, MOTOR_IDS)
    except Exception as e:
        if args.require_hw:
            print(f"[Main] Motor open failed: {e}")
            rc_link.stop()
            return
        print(f"[Main] Motors not connected ({e.__class__.__name__}) — "
              f"continuing with a STUB motor. Nothing will move.")
        motor = StubMotor(MOTOR_IDS, reason=f"{motor_port} unavailable")

    # Construction can fail too, not just open() — a missing driver
    # dependency (smbus2, pyserial) raises on import.
    imu = None
    try:
        imu = make_imu(args.imu,
                       port=args.imu_port, baud=args.imu_baud,
                       bus=args.imu_bus, addr=args.imu_addr, hz=args.imu_hz)
        imu_ok = imu.open()
    except Exception as e:
        print(f"[IMU] {'Open' if imu else 'Init'} raised "
              f"{e.__class__.__name__}: {e}")
        imu_ok = False

    if not imu_ok:
        if args.require_hw:
            print("[Main] IMU open failed, aborting.")
            motor.close()
            rc_link.stop()
            return
        print(f"[Main] IMU ({args.imu}) not connected — continuing with a "
              f"STUB IMU reporting level and still.")
        imu = StubIMU(kind=args.imu, reason=f"{args.imu} unavailable")
        imu.open()

    stub_motor = isinstance(motor, StubMotor)
    stub_imu   = isinstance(imu, StubIMU)
    sim_mode   = stub_motor or stub_imu

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
    # Set when the previous iteration's v_target hit MAX_VEL_CMD. Freezes the
    # velocity integrator while the wheels are already maxed out, since more
    # integral cannot buy speed the motors do not have.
    vel_saturated  = False
    was_armed      = False
    # Latched by the pitch safety trip. Blocks arming until the operator
    # cycles the arm switch/button low again, so the bot cannot re-arm on
    # its own just because it swung back through vertical while on the floor.
    safety_tripped = False
    trip_count     = 0

    pitch_filter      = LowPassFilter(alpha=0.12)
    # Raised from 0.12: pitch_rate is the raw gyro, not a differentiated
    # signal, so it needs far less smoothing than attitude does — and it
    # carries the damping term, where lag costs the most. 0.25 cuts this
    # path's time constant from 18.3 ms to 7.5 ms.
    pitch_rate_filter = LowPassFilter(alpha=0.25)
    vel_filter        = LowPassFilter(alpha=0.02)
    yaw_rate_filter   = LowPassFilter(alpha=0.09)

    MAX_TORQUE_RATE = 2.0 * DT
    prev_tff_l = 0.0
    prev_tff_r = 0.0

    print(f"\n{'='*65}")
    print(f"  Classical PID Balancer — MIT Mode + {args.rc.upper()} RC")
    if sim_mode:
        print(f"  *** STUB MODE — {'motors' if stub_motor else ''}"
              f"{' and ' if stub_motor and stub_imu else ''}"
              f"{'IMU' if stub_imu else ''} not connected ***")
    print(f"{'='*65}")
    print(f"  Motors:    {'STUB (no hardware) — nothing will move' if stub_motor else motor_port}")
    print(f"  IMU:       {imu.label}")
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
    print(f"  Pitch trim:{args.pitch_trim:+.2f}° "
          f"(balanced reads 0.0° after correction)")
    print(f"  Rate:      {CONTROL_HZ} Hz")
    print(f"  Duration:  {args.duration}s — Ctrl-C to stop")
    print(f"{'='*65}")
    if sim_mode:
        print("  Stub mode is for exercising the RC path only. It has no "
              "balance\n  physics, so nothing here indicates whether a gain "
              "is stable.")
    arm_hint = ("Tap ARM on the web page to ARM" if args.rc == "web"
                else "Flip CH8 (Aux4) high to ARM")
    print(f"\n  >>> {arm_hint} <<<\n")

    # ── Debug logging ─────────────────────────────────────────
    # Rows are only written while armed (plus the row that trips the pitch
    # limit) — the disarmed branch never reads the IMU, so there would be
    # nothing to record.
    logger = None
    log_every = max(1, int(round(CONTROL_HZ / max(args.log_hz, 0.01))))
    if args.log:
        log_path = args.log
        if log_path == 'auto':
            log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   "..", "logs")
            os.makedirs(log_dir, exist_ok=True)
            log_path = os.path.join(
                log_dir, time.strftime("balance_%Y%m%d_%H%M%S.csv"))
        logger = DebugLogger(log_path, LOG_FIELDS)
        print(f"[Log] Debug logging to {log_path} "
              f"at {CONTROL_HZ / log_every:.0f} Hz "
              f"({len(LOG_FIELDS)} columns, armed periods only)\n")

    step = 0
    prev_t_loop = time.monotonic()
    t_start = time.monotonic()

    try:
        while not shutdown:
            t_loop = time.monotonic()
            loop_ms = (t_loop - prev_t_loop) * 1000.0
            prev_t_loop = t_loop
            elapsed = t_loop - t_start
            if elapsed > args.duration:
                print("\n[Main] Duration reached.")
                break

            # ── Read RC ───────────────────────────────────────
            channels, rc_connected = rc_link.get_channels()
            cmds = rc.map(channels)
            # Releasing the arm switch clears a latched safety trip, so
            # recovery is an explicit disarm → re-arm by the operator.
            if not cmds['armed'] and safety_tripped:
                safety_tripped = False
                print(f"\n[SAFETY] Trip cleared — ready to arm again")

            armed = cmds['armed'] and rc_connected and not safety_tripped

            # Arm/disarm transitions
            if armed and not was_armed:
                print(f"\n[RC] >>> ARMED — motors active <<<")
                # Reset integrators on arm
                vel_integral = 0.0
                pitch_offset = 0.0
                vel_saturated = False
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
                vel_saturated = False
                prev_tff_l = 0.0
                prev_tff_r = 0.0

                if step % 400 == 0:
                    rc_status = "linked" if rc_connected else "NO LINK"
                    hint = ("SAFETY TRIP — release arm to clear"
                            if safety_tripped else arm_hint)
                    print(f"\r  [DISARMED]{' [STUB]' if sim_mode else ''} "
                          f"t={elapsed:.1f}s  "
                          f"RC={rc_status}  CH8={channels[7]:4d}  "
                          f"{hint}     ", end='', flush=True)

                step += 1
                t_end = time.monotonic()
                sleep_time = DT - (t_end - t_loop)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                continue

            # ── Read sensors ──────────────────────────────────
            pitch_raw, pitch_rate, yaw_rate = imu.read()
            # Remove the mounting/CoM bias here, before the filter, so every
            # consumer below (safety trip, pitch PD, prints, log) works in
            # "0 = upright" terms. pitch_rate needs no trim: a constant
            # offset differentiates to zero.
            pitch = pitch_raw - pitch_trim

            pos_l, vel_l, _ = motor.get_state(MOTOR_IDS[0])
            pos_r, vel_r, _ = motor.get_state(MOTOR_IDS[1])

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
                # Disarm and latch rather than exit — the process keeps
                # running so the bot can be picked up and re-armed.
                trip_count += 1
                safety_tripped = True
                motor.send_command(MOTOR_IDS[0], 0, 0, 0, 0, 0)
                motor.send_command(MOTOR_IDS[1], 0, 0, 0, 0, 0)
                vel_integral = 0.0
                pitch_offset = 0.0
                vel_saturated = False
                prev_tff_l = 0.0
                prev_tff_r = 0.0
                print(f"\n[SAFETY] Pitch {pitch_deg:+.1f}° exceeds "
                      f"±{args.max_pitch:.0f}° — DISARMED (trip #{trip_count}). "
                      f"Disarm, stand it up, then arm again.")
                if logger is not None:
                    # Outputs are zero here: the trip pre-empts the control
                    # math, so only the sensor columns are meaningful.
                    logger.log(elapsed, 0, 1, rc_connected,
                               cmd_vel, cmd_pitch, cmd_yaw,
                               pitch_deg, math.degrees(pitch_raw),
                               pitch_f, pitch_rate_f, yaw_rate_f,
                               avg_vel, avg_vel_f, avg_pos,
                               0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0,
                               0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0,
                               wheel_vel_l, wheel_vel_r, 0.0, 0.0,
                               loop_ms)
                step += 1
                t_end = time.monotonic()
                sleep_time = DT - (t_end - t_loop)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                continue

            # ══════════════════════════════════════════════════
            # Velocity PI → pitch_offset
            # ══════════════════════════════════════════════════
            pos_correction = 0.0
            if abs(cmd_vel) < 0.01:
                pos_correction = clamp(-pos_kp * avg_pos, -1.0, 1.0)

            effective_vel_cmd = cmd_vel + pos_correction
            vel_error = effective_vel_cmd - avg_vel_f

            # Conditional integration: only accumulate while there is still
            # wheel speed left to spend. Integrating through saturation is
            # what turned a bounded oscillation into a fall — the integrator
            # ramped to -7.0 and pinned pitch_offset at full lean while the
            # wheels were already stuck at their limit.
            if not vel_saturated or vel_error * vel_integral < 0.0:
                vel_integral += vel_error * DT
                max_int = VEL_INTEGRATOR_LIMIT / max(vel_ki, 1e-6)
                vel_integral = clamp(vel_integral, -max_int, max_int)

            pitch_offset = vel_kp * vel_error + vel_ki * vel_integral
            pitch_offset = clamp(pitch_offset,
                                 -PITCH_OFFSET_LIMIT, PITCH_OFFSET_LIMIT)

            # ══════════════════════════════════════════════════
            # Pitch PD → v_target
            # ══════════════════════════════════════════════════
            desired_pitch = cmd_pitch + pitch_offset
            pitch_error   = pitch_f - desired_pitch

            v_target = pitch_kp * pitch_error + pitch_kd * pitch_rate_f
            vel_saturated = abs(v_target) > MAX_VEL_CMD

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

            if logger is not None and step % log_every == 0:
                logger.log(elapsed, 1, 0, rc_connected,
                           cmd_vel, cmd_pitch, cmd_yaw,
                           pitch_deg, math.degrees(pitch_raw),
                           pitch_f, pitch_rate_f, yaw_rate_f,
                           avg_vel, avg_vel_f, avg_pos,
                           vel_error, vel_integral, pitch_offset,
                           desired_pitch, pitch_error, v_target,
                           delta_v, delta_tff,
                           v_target_l, v_target_r, tff_l, tff_r,
                           wheel_vel_l, wheel_vel_r, total_l, total_r,
                           loop_ms)

            step += 1

            # ── Live status (~10 Hz) ─────────────────────────
            if step % 40 == 0:
                print(
                    f"{'[STUB] ' if sim_mode else ''}"
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
                compute_ms = (time.monotonic() - t_loop) * 1000
                print(f"\n{'='*65}")
                print(f"  Step {step}  t={elapsed:.2f}s  "
                      f"compute={compute_ms:.1f}ms  period={loop_ms:.1f}ms")
                print(f"{'='*65}")
                print(f"  RC:  armed={armed}  connected={rc_connected}")
                print(f"       cmd_vel={cmd_vel:.3f}  cmd_pitch={cmd_pitch:+.4f}  "
                      f"cmd_yaw={cmd_yaw:+.3f}")
                print(f"       CH: [{channels[0]:4d} {channels[1]:4d} {channels[2]:4d} "
                      f"{channels[3]:4d} ... {channels[7]:4d}]")
                print(f"  pitch:       {pitch_f:+.4f} rad ({pitch_deg:+.1f}°)"
                      f"   raw={math.degrees(pitch_raw):+.2f}° "
                      f"trim={args.pitch_trim:+.2f}°")
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
                print(f"  IMU rate:    "
                      f"{'n/a (stub)' if stub_imu else f'{imu.rate_hz:.1f} Hz'}")
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
        if logger is not None:
            rows, dropped = logger.close()
            print(f"[Log] Wrote {rows} rows to {logger.path}"
                  + (f" ({dropped} dropped — writer fell behind)"
                     if dropped else ""))
        if trip_count:
            print(f"[Main] Pitch safety tripped {trip_count}x this session.")
        print("[Main] Done.")


if __name__ == "__main__":
    main()
