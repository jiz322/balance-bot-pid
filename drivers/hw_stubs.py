"""
Hardware stubs — run the control stack with no motors or IMU attached
=====================================================================
Drop-in stand-ins used when a device fails to open, so the RC path, arming,
mapping, safety limits and prints can be exercised on a bench with nothing
but a Pi. They accept every call the real drivers do and discard the writes.

These are NOT a simulator. StubMotor gives wheels a first-order velocity
response only so the velocity PI term sees a plausible signal instead of a
hard zero (which would wind its integrator up and make the printed numbers
meaningless). StubIMU always reports perfectly level and motionless: there
is no balance physics here, so nothing observed in stub mode says anything
about whether a gain is stable on the real robot.
"""

import time


class StubMotor:
    """Stands in for RobStrideDriver. Same call surface, no serial port."""

    #: wheel velocity time constant, seconds — fast enough to feel responsive
    TAU = 0.08

    def __init__(self, motor_ids, reason=""):
        self.motor_ids = motor_ids
        self.reason = reason
        self.port = None
        self.command_count = 0
        self.enabled = False
        self._t_last = time.monotonic()
        self._state = {mid: {'pos': 0.0, 'vel': 0.0, 'cmd_vel': 0.0}
                       for mid in motor_ids}

    # ── Lifecycle (all no-ops, kept for interface parity) ─────
    def set_zero_all(self):
        for s in self._state.values():
            s['pos'] = 0.0
        print("[Driver:STUB] zero (no hardware)")

    def enable_all(self):
        self.enabled = True
        print(f"[Driver:STUB] motors {self.motor_ids} 'enabled' (no hardware)")

    def disable_all(self):
        self.enabled = False
        print(f"[Driver:STUB] motors {self.motor_ids} 'disabled' (no hardware)")

    def close(self):
        print(f"[Driver:STUB] closed after {self.command_count} discarded "
              f"commands")

    # ── Command / state ───────────────────────────────────────
    def send_command(self, motor_id, p_des=0.0, v_des=0.0,
                     kp=0.0, kd=0.0, t_ff=0.0):
        """Discard the frame; relax this wheel's velocity toward v_des."""
        self.command_count += 1
        s = self._state.get(motor_id)
        if s is None:
            return
        s['cmd_vel'] = v_des

        now = time.monotonic()
        dt = now - self._t_last
        self._t_last = now
        if dt <= 0.0 or dt > 0.25:      # first call, or a stall — don't jump
            return

        alpha = min(1.0, dt / self.TAU)
        for st in self._state.values():
            st['vel'] += (st['cmd_vel'] - st['vel']) * alpha
            st['pos'] += st['vel'] * dt

    def get_state(self, motor_id):
        s = self._state[motor_id]
        return s['pos'], s['vel'], 0.0


class StubIMU:
    """Stands in for IMUSource. Always level and still."""

    def __init__(self, kind="", reason=""):
        self.kind = kind
        self.reason = reason
        self.port = None
        self.label = f"STUB (no hardware{': ' + reason if reason else ''})"

    @property
    def driver(self):
        return None

    def open(self):
        return True

    def close(self):
        print("[IMU:STUB] closed (no hardware)")

    def read(self):
        """Always (0, 0, 0) — level, motionless, no balance dynamics."""
        return 0.0, 0.0, 0.0

    @property
    def rate_hz(self):
        return 0.0
