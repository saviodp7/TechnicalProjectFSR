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

IO_LINEARIZATION = 1
POSTURE_REGULATION = 2


def loop(timer):       
    # BT reading and control
    data_list = bluetooth.read()
    if data_list:
        print(f'[{time.time()}] - [main.loop] data_list: {data_list}')
        if data_list[0] == 'control':
            if data_list[1] == IO_LINEARIZATION:
                controller = InputOutputLinearization(robot, estimator)
            elif data_list[1] == POSTURE_REGULATION:
                controller = PostureRegulation(robot, estimator)
            else:
                bluetooth.write('error,controller')
                raise ValueError("Controller not valid!")
        elif data_list[0] == 'command':
            command = dict()
            command["speed"] = float(data_list[1])
            command["angle"] = float(data_list[2])
            robot.go(command["speed"], angular_velocity_deg=command["angle"] * 60)
        elif data_list[0] == 'start':
            pass # TODO: comando segui traiettoria
        elif data_list[0] == 'stop':
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
#controller = InputOutputLinearization(robot, estimator)
controller = PostureRegulation(robot, estimator)

loop_timer = Timer()
loop_timer.init(freq=10, mode=Timer.PERIODIC, callback=loop)
### TESTS ###
controller.go((1, 0.4))
#controller.execute_trajectory([(1.4,-0.4),(1.4,0.5),(0.5,0.5)], dt=10)
### ----- ###    
while True:
    pass
     