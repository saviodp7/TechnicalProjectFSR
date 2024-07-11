from stepper import Stepper
import time
from machine import Timer, Pin
from wheeled import DifferentialDrive
from bluetooth import Bluetooth
from imu import IMU
from math import pi
from control import InputOutputLinearization, PostureRegulation
import localization as ul
from localization import EULER_APPROX, RUNGE_KUTTA_APPROX

def loop(timer):       
    # BT reading and control
    data_list = bluetooth.read()
    if data_list:
        #print(f'[{time.time()}] - [main.loop] data_list: {data_list}')
        if data_list[0] == 'control':
            if data_list[1] == IO_LINEARIZATION:
                controller = InputOutputLinearization(robot, estimator)
            elif data_list[1] == POSTURE_REGULATION:
                controller = PostureRegulation(robot, estimator)
            else:
                bluetooth.write('error,controller')
                raise ValueError("Controller not valid!")
        elif data_list[0] == 'start':
            pass # TODO: comando segui traiettoria
        elif data_list[0] == 'command':
            command = dict()
            command["speed"] = float(data_list[1])
            command["angle"] = float(data_list[2])
            robot.go(command["speed"], angular_velocity_deg=command["angle"] * -60)
        elif data_list[0] == 'stop': # TODO: Secondo me non funziona
            if isinstance(controller, InputOutputLinearization):
                controller = None
                time.sleep(0.1)
                controller = InputOutputLinearization(robot, estimator)
            elif isinstance(controller, PostureRegulation):
                controller = None
                time.sleep(0.1)
                controller = PostureRegulation(robot, estimator)
            robot.stop()
        elif data_list[0] == 'goto':
            command = dict()
            command["x"] = float(data_list[1])
            command["y"] = float(data_list[2])
            command["theta"] = float(data_list[3])
            controller.go((command["x"], command["y"], command["theta"]))
        elif data_list[0] == 'trajectory':
            trajectory_points = data_list[1:-1]
            print(trajectory_points)
    bluetooth.write(*estimator.position, robot.speed, robot.omega)

# Neapolitan pezza
V3 = Pin(23, Pin.OUT)
V3.high()
# ---
start = False
### SETUP ###
# Communications
bluetooth = Bluetooth()
# Sensors
imu = IMU(id=1, sda=Pin(26), scl=Pin(27))
imu.calibrate()
# Attuation
motor_sx = Stepper(3, 2, 6, 7, 8,resolution=16)
motor_dx = Stepper(15 ,14, 11, 12, 13, resolution=16)
robot = DifferentialDrive(motor_sx, motor_dx, wheels_diameter_m=12.51e-2, track_width_m=14.21e-2, max_vel=0.1)
time.sleep(0.2)
### INITIALIZATION ###
# Motor initialization check
# robot.go(1,0)
# time.sleep(0.01)
# robot.go(-1,0)
# time.sleep(0.01)
# robot.stop()
# Localization and bluetooth
estimator = ul.UnicycleLocalization(robot, imu)

# CONTROLLER
controller = InputOutputLinearization(robot, estimator)
#controller = PostureRegulation(robot, estimator)

loop_timer = Timer()
loop_timer.init(freq=10, mode=Timer.PERIODIC, callback=loop)
### TESTS ###
controller.execute_trajectory([(0.12, 0.03), (0.23, 0.17), (0.32, 0.19), (0.46, 0.14), (0.56, 0.29), (0.66, 0.4), (0.73, 0.48), (0.86, 0.58), (0.96, 0.65), (1.09, 0.82), (1.19, 0.9), (1.4, 0.96), (1.49, 0.99)], dt=2)
### ----- ###    
while True:
    pass
     