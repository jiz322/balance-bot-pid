"""
RC Mapper — ELRS channels to balance robot commands
=====================================================
Channel mapping:
    CH3 (Throttle, idx 2) → cmd_vel    0.0 to +2.0  (one-directional)
    CH2 (Pitch/E, idx 1)  → cmd_pitch  -0.3 to +0.3 (centered)
    CH4 (Yaw/R,   idx 3)  → cmd_yaw    -2.0 to +2.0 (centered)
    CH8 (Aux4,    idx 7)  → arm switch  low=disarmed, high=armed

CRSF range: 172 (min) — 992 (center) — 1811 (max)
"""

CRSF_MIN = 172
CRSF_MID = 992
CRSF_MAX = 1811


class RCMapper:
    def __init__(self,
                 vel_range=(0.0, 2.0),
                 pitch_range=(-0.3, 0.3),
                 yaw_range=(-2.0, 2.0),
                 deadband=30,
                 arm_threshold=600):
        """
        Args:
            vel_range:      (min, max) for cmd_vel mapped from throttle
            pitch_range:    (min, max) for cmd_pitch mapped from centered stick
            yaw_range:      (min, max) for cmd_yaw mapped from centered stick
            deadband:       counts around center to treat as zero (for centered sticks)
            arm_threshold:  CH8 value above this = armed
        """
        self.vel_range = vel_range
        self.pitch_range = pitch_range
        self.yaw_range = yaw_range
        self.deadband = deadband
        self.arm_threshold = arm_threshold

    def _map_throttle(self, val):
        """Map throttle (172-1811) → vel_range (one-directional, 172=min)."""
        t = (val - CRSF_MIN) / (CRSF_MAX - CRSF_MIN)
        t = max(0.0, min(1.0, t))
        return self.vel_range[0] + t * (self.vel_range[1] - self.vel_range[0])

    def _map_centered(self, val, out_range):
        """Map centered stick (992=center) → symmetric range with deadband."""
        centered = val - CRSF_MID
        if abs(centered) < self.deadband:
            return 0.0
        # Remove deadband from the signal
        if centered > 0:
            normalized = (centered - self.deadband) / (CRSF_MAX - CRSF_MID - self.deadband)
        else:
            normalized = (centered + self.deadband) / (CRSF_MID - CRSF_MIN - self.deadband)
        normalized = max(-1.0, min(1.0, normalized))
        # Map to output range (symmetric around 0)
        half = max(abs(out_range[0]), abs(out_range[1]))
        return normalized * half

    def is_armed(self, channels):
        """CH8 (idx 7): low (~191) = disarmed, high = armed."""
        return channels[7] > self.arm_threshold

    def map(self, channels):
        """Map CRSF channels to robot commands."""
        return {
            'cmd_vel':   self._map_throttle(channels[2]),    # CH3 Throttle
            'cmd_pitch': self._map_centered(channels[1], self.pitch_range),  # CH2 Pitch/E
            'cmd_yaw':   self._map_centered(channels[3], self.yaw_range),    # CH4 Yaw/R
            'armed':     self.is_armed(channels),            # CH8 Aux4
        }