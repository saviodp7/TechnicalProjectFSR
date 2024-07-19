from machine import I2C, Pin
from math import sqrt, atan2, pi, copysign, sin, cos
from mpu6500 import MPU6500
import time


class IMU:
    """ Class for RPY evaluation """

    to_rad = pi/180.0

    def __init__(self, id=0, sda=Pin(0), scl=Pin(1), i2c=None, imu=None):
        if i2c is None:
            i2c = I2C(id=id, scl=scl, sda=sda)
        
        if imu is None:
            self._imu = MPU6500(i2c)
        else:
            self._imu = MPU6500
        time.sleep(0.1)

        self._pitch_bias = 0
        self._roll_bias = 0
        self._gyr_x_bias = 0
        self._gyr_y_bias = 0
        self._gyr_z_bias = 0
        self._acc_x_bias = 0
        self._acc_y_bias = 0
        self._acc_z_bias = 0

    @property
    def pitch_bias(self):
        return self._pitch_bias

    @pitch_bias.setter
    def pitch_bias(self, pitch_bias_value: float):
        if -180 < pitch_bias_value < 180:
            self._pitch_bias = pitch_bias_value
        else:
            raise ValueError("Pitch bias must be between -180.0 and 180.0")

    @property
    def roll_bias(self):
        return self._roll_bias

    @roll_bias.setter
    def roll_bias(self, roll_bias_value: float):
        if -180 < roll_bias_value < 180:
            self._roll_bias = roll_bias_value
        else:
            raise ValueError("Roll bias must be between -180.0 and 180.0")

    def calibrate(self) -> tuple[float, float]:
        """ Set the current position as neutral position returning the biases """
        _roll_bias = _pitch_bias = _gyr_x_bias = _gyr_y_bias = _gyr_z_bias = _acc_x_bias = _acc_y_bias = _acc_z_bias= 0
        for i in range(100):
            roll, pitch = self.rp_deg
            _roll_bias += roll
            _pitch_bias += pitch
            _gyr_x_bias += self.gyro[0]
            _gyr_y_bias += self.gyro[1]
            _gyr_z_bias += self.gyro[2]
            _acc_x_bias += self.acceleration[0]
            _acc_y_bias += self.acceleration[1]
            _acc_z_bias += self.acceleration[2]
        self._roll_bias = _roll_bias/100
        self._pitch_bias = _pitch_bias/100
        self._gyr_x_bias = _gyr_x_bias/100
        self._gyr_y_bias = _gyr_y_bias/100
        self._gyr_z_bias = _gyr_z_bias/100
        self._acc_x_bias = _acc_x_bias/100
        self._acc_y_bias = _acc_y_bias/100
        self._acc_z_bias = _acc_z_bias/100
        return roll, pitch

    @property
    def acceleration(self) -> tuple[float, float, float]:
        """
        Acceleration measured by the sensor. By default, will return a
        3-tuple of X, Y, Z axis values in m/s^2 as floats. To get values in g
        pass `accel_fs=SF_G` parameter to the MPU6500 constructor.
        """
        if self._acc_x_bias or self._acc_y_bias or self._acc_z_bias :
            return [self._imu.accelerations[0]-self._acc_x_bias, self._imu.accelerations[1]-self._acc_y_bias, self._imu.accelerations[2]-self._acc_z_bias]
        else:
            return self._imu.accelerations

    @property
    def gyro(self) -> tuple[float, float, float]:
        """
        Gyro measured by the sensor. By default, will return a 3-tuple of
        X, Y, Z axis values in rad/s as floats. To get values in deg/s pass
        `gyro_sf=SF_DEG_S` parameter to the MPU6500 constructor.
        """
        if self._gyr_x_bias or self._gyr_y_bias or self._gyr_z_bias :
            return [self._imu.gyro[0]-self._gyr_x_bias, self._imu.gyro[1]-self._gyr_y_bias, self._imu.gyro[2]-self._gyr_z_bias]
        else:
            return self._imu.gyro

    @property
    def temperature(self) -> float:
        """ Return temperature in celcius as a float. """
        return self._imu.temperature

    @property
    def whoami(self):
        """ Value of the whoami register. """
        return self._imu.whoami

    @property
    def rp_deg(self) -> tuple[float, float]:
        """ Returns the pitch and roll in degrees """
        x, y, z = self.acceleration

        # Pitch and Roll in Degrees
        pitch_deg = atan2(-x, sqrt((z * z) + (y * y))) * 180 / pi
        roll_deg = atan2(y, z) * 180 / pi

        return roll_deg + self._roll_bias, pitch_deg + self._pitch_bias

    @property
    def rp_rad(self) -> tuple[float, float]:
        """ Returns the pitch and roll in radians """
        x, y, z = self.acceleration

        # Pitch and Roll in Radians
        roll_rad = atan2(-x, sqrt((z * z) + (y * y)))
        # pitch_rad = atan2(z, copysign(y,y)*sqrt((0.01*x*x)+(y*y)))
        pitch_rad = atan2(y, z)

        return roll_rad + self._roll_bias*to_rad, pitch_rad + self._pitch_bias*to_rad

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        pass

    # TODO: Calibration e Bias
    # Calibration and bias offset
    # m.calibrate(count=100,delay=1)


if __name__ == "__main__":
   imu = IMU(id=1, sda=Pin(26), scl=Pin(27))
   imu.calibrate()
   while True:
        roll, pitch = imu.rp_deg
        print(f'acc_x: {imu.acceleration[0]} \t acc_y: {imu.acceleration[1]} \t acc_z: {imu.acceleration[2]}')
        print(f'gyr_x: {imu.gyro[0]} \t gyr_y: {imu.gyro[1]} \t gyr_z: {imu.gyro[2]}')
        print(f'roll: {roll} \t pitch: {pitch}')
        print(f'temperature: {imu.temperature}\n')
        time.sleep(0.1)
