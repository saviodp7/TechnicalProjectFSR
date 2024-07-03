from math import cos, sin
from machine import Timer, Pin
import time
if __name__ == "__main__":
    from imu import IMU
    from stepper import Stepper
    from wheeled import DifferentialDrive

EULER_APPROX = 0
RUNGE_KUTTA_APPROX = 1


class UnicycleLocalization:
    def __init__(self, robot, imu, mode=EULER_APPROX, f_s: float = 100, x_0: float = 0, y_0: float = 0, theta_0: float = 0):
        self._mode = mode
        self._imu = imu
        self._robot = robot
        self._x = x_0
        self._y = y_0
        self._theta = theta_0
        self._T_s = 1/f_s
        self._timer = Timer()
        self._timer.init(freq=f_s, mode=Timer.PERIODIC, callback=self.update_callback)

    def update_callback(self, timer):
        """
        Update the estimated pose with the selected method (self._mode) using the sensor reading and
        odometric information
        """
        if self._mode == EULER_APPROX:
            self.euler_approximation()
        elif self._mode == RUNGE_KUTTA_APPROX:
            self.runge_kutta_approximation()
        else:
            raise Exception("Localization method not found.")
        
    def get_imu(self, value: float = 0.1):
        return tuple([gyro if abs(gyro) >= 0.035 else 0.0 for gyro in self._imu.gyro])

    def euler_approximation(self):
        """Update the position with the euler approximation method"""
        self._x = self._x + self._robot.speed*self._T_s*cos(self._theta)
        self._y = self._y + self._robot.speed*self._T_s*sin(self._theta)
        self._theta = self._theta + (-self.get_imu()[2])*self._T_s

    def runge_kutta_approximation(self):
        """Update the position with the second-order Runge-Kutta approximation method"""
        self._x = self._x + self._robot.speed*self._T_s*cos(self._theta + (-self.get_imu()[2])*self._T_s/2)
        self._y = self._y + self._robot.speed*self._T_s*sin(self._theta + (-self.get_imu()[2])*self._T_s/2)
        self._theta = self._theta + (-self.get_imu()[2])*self._T_s

    @property
    def position(self) -> tuple[float, float, float]:
        return tuple([self._x, self._y, self._theta])

    @position.setter
    def position(self, position: tuple[float, float, float]):
        self._x = position[0]
        self._y = position[1]
        self._theta = position[2]
        
if __name__ == "__main__":
    imu = IMU(id=1, sda=Pin(26), scl=Pin(27))
    motor_sx = Stepper(15 ,14, 11, 12, 13, resolution=4)
    motor_dx = Stepper(3, 2, 6, 7, 8,resolution=4)
    robot = DifferentialDrive(motor_sx, motor_dx, wheels_diameter_m=12.65e-2, track_width_m=13.95e-2, max_vel=0.5)
    estimator = UnicycleLocalization(robot, imu)
    prev_time = time.time_ns()
    while True:
        now = time.time_ns()
        if now - prev_time > 0.5e+9:
            prev_time = now
            print(*estimator.position)

