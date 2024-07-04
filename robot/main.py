from stepper import Stepper
import time
from machine import Timer, Pin
from wheeled import DifferentialDrive
from bluetooth import Bluetooth
from imu import IMU
import localization as ul

       
if __name__ == "__main__":
    
    # Neapolitan pezza
    V3 = Pin(23, Pin.OUT)
    V3.high()
    # ---
    
    # Communications
    bluetooth = Bluetooth()
    
    # Sensors
    imu = IMU(id=1, sda=Pin(26), scl=Pin(27))
    
    # Attuation
    motor_sx = Stepper(15 ,14, 11, 12, 13, resolution=8)
    motor_dx = Stepper(3, 2, 6, 7, 8,resolution=8)
    robot = DifferentialDrive(motor_sx, motor_dx, wheels_diameter_m=12.65e-2, track_width_m=13.95e-2, max_vel=0.5)
    # Motor initialization check
    robot.go(1,0)
    time.sleep(0.5)
    robot.stop()
    
    # Localization
    estimator = ul.UnicycleLocalization(robot, imu)
    
    prev_time = time.time_ns()
    
    while True:
        now = time.time_ns()
        if now - prev_time > 0.1e+9:
            prev_time = now
            
            # BT reading and control
            data_list = bluetooth.read()
            if data_list and len(data_list)==2 and not ("" in data_list):
                command = dict()
                command["speed"] = float(data_list[0])
                command["angle"] = -float(data_list[1])
                robot.go(command["speed"], angular_velocity_rad=command["angle"])
            bluetooth.write(*estimator.position)
                
    
