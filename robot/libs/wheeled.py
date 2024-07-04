from math import pi
from stepper import Stepper
import time

# TODO: TESTING!!!


class DifferentialDrive:

    def __init__(self, motor_sx, motor_dx, wheels_diameter_m: float | int, track_width_m: float | int, max_vel: float | int = 0) -> None:
        self.motor_sx = motor_sx
        self.motor_dx = motor_dx
        self.wheel_radius = wheels_diameter_m/2  # m
        self.track_width = track_width_m  # m
        self._speed = 0.0
        self._omega = 0.0
        self.max_vel = max_vel  # m/s

        # Stop the motors
        self.stop()
 
    def stop(self):
        self._speed = 0.0
        self._omega = 0.0
        self.motor_sx.speed = 0
        self.motor_dx.speed = 0

    def differential_drive_ik(self, speed: float | int, angular_velocity_rad: float | int) -> tuple[float, float]:
        """DDR inverse kinematics: calculate wheels speeds from desired velocity."""
        vel_sx = (speed - (self.track_width/2)*angular_velocity_rad)/self.wheel_radius
        vel_dx = (speed + (self.track_width/2)*angular_velocity_rad)/self.wheel_radius
        return vel_sx, vel_dx

    @property
    def speed(self):
        return self._speed
    # TODO: Setter

    @property
    def omega(self):
        return self._omega
    # TODO: Setter
 
    def go(self, speed: float | int, angular_velocity_rad: float | int = 0, angular_velocity_deg: float | int = 0, turning_radius: float | int = 0) -> None:
        
        # Check if only one steering parameter is passed
        if [angular_velocity_rad, angular_velocity_deg, turning_radius].count(0) < 2:
            raise TypeError("Only one angular velocity/steering angle must be given")
           
        # saturate speed
        if self.max_vel:
            speed = min(speed, self.max_vel)
            speed = max(speed, -self.max_vel)
        
        if angular_velocity_deg:
            omega = angular_velocity_deg*pi/180
        elif turning_radius:
            omega = speed/turning_radius
        else:
            omega = angular_velocity_rad

        self._speed = speed
        self._omega = omega

        vel_sx, vel_dx = self.differential_drive_ik(speed, omega)
        self.motor_sx.speed = vel_sx*60/(2*pi)
        self.motor_dx.speed = vel_dx*60/(2*pi)


if __name__ == "__main__":
    motor_sx = Stepper(15, 14, 7, 8, 9, resolution=4)
    motor_dx = Stepper(3, 2, 11, 12, 13, resolution=4)
    robot = DifferentialDrive(motor_sx, motor_dx, wheels_diameter_m=12.65e-2, track_width_m=13.95e-2, max_vel=0.5)
    robot.go(1, 0)
    time.sleep(2)
    robot.stop()
