from math import pi
from machine import Pin, Timer
import utime

class Stepper:
    """ Control class for a Nema stepper motor """

    toRPM = 60/(2*pi)
    torad_s = 2*pi/60
    
    def __init__(self, STEP_PIN : int, DIR_PIN : int, M0_PIN : int, M1_PIN : int, M2_PIN : int, resolution : int=1):
        # PINS
        self.STEP_PIN = Pin(STEP_PIN, Pin.OUT)
        self.DIR_PIN = Pin(DIR_PIN, Pin.OUT)
        self.M0 = Pin(M0_PIN, Pin.OUT)
        self.M1 = Pin(M1_PIN, Pin.OUT)
        self.M2 = Pin(M2_PIN, Pin.OUT)
        self.RES_PINS = (self.M0, self.M1, self.M2)

        self.RESOLUTION = {1: (0, 0, 0),
                          2: (1, 0, 0),
                          4: (0, 1, 0),
                          8: (1, 1, 0),
                          16: (0, 0, 1),
                          32: (1, 0, 1)}
        # Motor parameters
        self.step_per_revolution_NEMA = 200
        self.step_angle_increment, self.step_per_revolution = self.set_resolution(resolution)
        # Kynematics parameters
        self.position = 0
        self.target_position = 0
        self._speed = 0
        
        self.timer = Timer()

    def set_resolution(self, resolution: int) -> Tuple[int | float, int] | None:
        """Pull the right PINS to set resolution"""
        if resolution in self.RESOLUTION.keys():
            for i in range(3):
                self.RES_PINS[i].value(self.RESOLUTION[resolution][i])
            step_per_revolution = self.step_per_revolution_NEMA*resolution
            step_angle_increment = 360.0/step_per_revolution
            return step_angle_increment, step_per_revolution
        else:
            raise ValueError(f"{resolution} is not a valid resolution!")

    def set_direction(self, speed_value: float | int) -> None:
        """Take a value of velocity/angle and set the DIR_PIN"""
        if speed_value >= 0:
            self.DIR_PIN.value(1)
        else:
            self.DIR_PIN.value(0)
 
    def step(self, timer : Timer) -> None:
        """Do one step"""
        self.STEP_PIN.toggle()
        self.STEP_PIN.toggle()
        
    def increment_position(self) -> None:
        if self.DIR_PIN.value():
            self.position += self.step_angle_increment
        else:
            self.position -= self.step_angle_increment  
        
    def relative_step(self, timer : Timer) -> None:
        """Do one step if the target position is not reached"""
        if abs(self.position - self.target_position) < self.step_angle_increment:
            self.timer.deinit()
        else:
            self.step()
            self.increment_position()
        
    def get_frequency_RPM(self, speed_RPM : float | int) -> float:
        """Compute the frequency to run a motor at a given velocity (RPM)"""
        return self.step_per_revolution*abs(speed_RPM)/60
    
    @property
    def speed(self):
        return self._speed
       
    @speed.setter
    def speed(self, speed_RPM : int | float) -> None:
        """Set velocity in RPM"""
        if speed_RPM == 0:
            self._speed = speed_RPM
            self.stop()
        elif abs(speed_RPM) <= 180: # max speed
            self.set_direction(speed_RPM)
            frequency = self.get_frequency_RPM(abs(speed_RPM))
            self.timer.init(freq=frequency, mode=Timer.PERIODIC, callback=self.step)
            self._speed = speed_RPM
        else:
            raise ValueError("Too high velocity")
        
    def stop(self) -> None:
        """Stop the motors"""
        self.timer.deinit()
        self._speed = 0
         
    def relative_position(self, angle : int | float, velocity_rad_s : int | float = 1) -> float:
        """
        Move the link at a relative angle from the current position at a given velocity, 
        the approximation caused by the motor resolution will be a default approximation
        """
        self.set_direction(angle)
        n_steps = int(angle/self.step_angle_increment)
        actual_rotation = self.step_angle_increment*n_steps
        self.target_position = self.position + actual_rotation
        frequency = self.get_frequency_RPM(velocityRPM := velocity_rad_s*self.toRPM)
        self.timer.init(freq=frequency, mode=Timer.PERIODIC, callback=self.relative_step)
        return actual_rotation
    
    def set_position(self, angle : int | float, velocity_rad_s : int | float = 1) -> float:
        """
        Move the link at a absolute angle at a given velocity, the approximation caused 
        by the motor resolution will be a default approximation
        """
        self.target_position = angle
        print(f"absolute position: {self.position}")
        self.relative_position(angle_from_target_positon := self.target_position - self.position, velocity_rad_s)
        return self.position
          
               