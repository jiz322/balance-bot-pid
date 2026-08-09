"""
IMU Selector — one uniform interface over the four IMU drivers
==============================================================
The drivers do NOT agree on units or axis order, so control loops cannot
simply swap one for another:

    driver              class            euler units  euler order       gyro
    imu_driver_i2c      IMUDriverI2c     rad          roll,pitch,yaw    rad/s
    imu_driver          IMUDriver        rad          roll,pitch,yaw    rad/s
    imu_driver_6_400hz  IMUDriver6Axis   deg          pitch,roll,yaw    deg/s
    imu_driver_9_400hz  IMUDriver9Axis   deg          pitch,roll,yaw    deg/s

(Both conventions are load-bearing in the existing scripts: classical_mit.py
reads euler[1] raw for I2C, classical_mit_rc.py reads euler[0]*DEG2RAD for
the 6-axis UART driver. Mixing them up reads roll as pitch and scales
radians by 57.3.)

IMUSource normalizes all of that. read() always returns radians, sign
convention matching the balance loops (nose-down positive):

    pitch_rad, pitch_rate_rad_s, yaw_rate_rad_s = src.read()

Choices: i2c (default) | usb | uart6 | uart9
"""

import math

IMU_CHOICES = ('i2c', 'usb', 'uart6', 'uart9')

DEG2RAD = math.pi / 180.0

# name → (module, class, euler pitch index, euler/gyro scale, default port)
_SPECS = {
    'i2c':   ('imu_driver_i2c',     'IMUDriverI2c',   1, 1.0,     None),
    'usb':   ('imu_driver',         'IMUDriver',      1, 1.0,     '/dev/ttyUSB0'),
    'uart6': ('imu_driver_6_400hz', 'IMUDriver6Axis', 0, DEG2RAD, '/dev/ttyAMA0'),
    'uart9': ('imu_driver_9_400hz', 'IMUDriver9Axis', 0, DEG2RAD, '/dev/ttyAMA0'),
}


def default_port(kind):
    """Default device for a driver, or None for I2C (bus/addr instead)."""
    return _SPECS[kind][4]


class IMUSource:
    """Uniform, unit-normalized view over one IMU driver."""

    def __init__(self, kind, port=None, baud=None,
                 bus=1, addr=0x23, hz=400):
        if kind not in _SPECS:
            raise ValueError(f"unknown IMU {kind!r}, expected one of "
                             f"{', '.join(IMU_CHOICES)}")
        module, cls_name, self._pitch_idx, self._scale, dflt_port = _SPECS[kind]

        self.kind = kind
        self.port = port or dflt_port
        self.hz = hz

        mod = __import__(module)
        cls = getattr(mod, cls_name)

        if kind == 'i2c':
            self._imu = cls(bus_id=bus, addr=addr, loop_hz=hz)
            self.label = f"i2c bus {bus} addr 0x{addr:02X} @ {hz}Hz target"
        elif kind == 'usb':
            self._imu = cls(self.port, baud or 115200)
            self.label = f"usb {self.port} @ {baud or 115200} baud, 100Hz"
        else:
            self._imu = cls(self.port, baud or 460800)
            axes = '6-axis' if kind == 'uart6' else '9-axis'
            self.label = f"uart {self.port} @ {baud or 460800} baud, {axes} 400Hz"

    @property
    def driver(self):
        """The underlying driver, for calls this wrapper does not cover."""
        return self._imu

    def open(self):
        """Open and apply the per-driver post-open setup. False on failure."""
        if not self._imu.open():
            return False
        # The UART drivers configure themselves inside open(); the other two
        # need the 6-axis algorithm selected so the magnetometer cannot pull
        # pitch around near motors.
        if self.kind == 'i2c':
            self._imu.set_algorithm_6axis()
        elif self.kind == 'usb':
            self._imu.set_frequency(100)
            self._imu.set_algorithm_6axis()
        return True

    def close(self):
        self._imu.close()

    def read(self):
        """(pitch, pitch_rate, yaw_rate) in rad and rad/s, balance-loop sign."""
        euler = self._imu.data['euler']
        gyro = self._imu.data['gyro']
        s = self._scale
        return (-euler[self._pitch_idx] * s,
                -gyro[1] * s,
                -gyro[2] * s)

    @property
    def rate_hz(self):
        """Measured update rate, or 0.0 if the driver does not report one."""
        if hasattr(self._imu, 'get_actual_hz'):
            return self._imu.get_actual_hz()
        return getattr(self._imu, 'fps', 0.0)


def make_imu(kind='i2c', **kwargs):
    """Build an IMUSource. See IMUSource for the accepted keywords."""
    return IMUSource(kind, **kwargs)
