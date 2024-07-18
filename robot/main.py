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

### FLAGS ###
IO_LINEARIZATION = 1
POSTURE_REGULATION = 2
# Neapolitan pezza
V3 = Pin(23, Pin.OUT)
V3.high()

### INITIALIZATION ###
bt_queue = list()
trajectory_points = list()
f_s = 10
# Communications
bluetooth = Bluetooth()
# Sensors
imu = IMU(id=1, sda=Pin(26), scl=Pin(27))
imu.calibrate()
# Attuation
motor_sx = Stepper(3, 2, 6, 7, 8,resolution=8)
motor_dx = Stepper(15 ,14, 11, 12, 13, resolution=8)
robot = DifferentialDrive(motor_sx, motor_dx, wheels_diameter_m=12.51e-2, track_width_m=14.21e-2, max_vel=0.1)
time.sleep(0.1)
# Localization and bluetooth
estimator = ul.UnicycleLocalization(robot, imu, mode=RUNGE_KUTTA_APPROX)
# CONTROLLER
controller = InputOutputLinearization(robot, estimator)

### COMMUNICATION THREADS ###
def enqueue_bt(timer): 
    data_list = bluetooth.read()
    if data_list:
        for data in data_list:
            bt_queue.append(data)

def dequeue_bt(timer):
    global controller
    global trajectory_points
    global f_s
    if len(bt_queue):
        command = bt_queue.pop(0)
        if command == 'command':
            command = dict()
            command["speed"] = float(bt_queue.pop(0))
            command["angle"] = float(bt_queue.pop(0))
            robot.go(command["speed"], angular_velocity_deg=command["angle"] * -60)
        elif command == 'control':
            control = int(bt_queue.pop(0))
            if control == IO_LINEARIZATION:
                controller = InputOutputLinearization(robot, estimator)
            elif control == POSTURE_REGULATION:
                controller = PostureRegulation(robot, estimator)
        elif command == 'stop':
            controller.timer.deinit()
            controller.trajectory_timer.deinit()
            robot.stop()
            if isinstance(controller, InputOutputLinearization):
                controller = InputOutputLinearization(robot, estimator)
            elif isinstance(controller, PostureRegulation):
                controller = PostureRegulation(robot, estimator)
        elif command == 'start':
            print(f'[{time.time()}] - [dequeue_bt] trajectory_points : {trajectory_points}')
            controller.execute_trajectory(trajectory_points, dt=1/f_s)
        elif command == 'goto':
            command = dict()
            command["x"] = float(bt_queue.pop(0))
            command["y"] = float(bt_queue.pop(0))
            try:
                command["theta"] = float(bt_queue.pop(0))
            except IndexError:
                pass
            if isinstance(controller, InputOutputLinearization):
                controller.go((command["x"], command["y"]))
            elif isinstance(controller, PostureRegulation):
                controller.go((command["x"], command["y"], command["theta"]))
        elif 'trajectory' in command:
            trajectory_points = list()
            try:
                f_s = int(command[-2:])
            except:
                f_s = int(command[-1])
        else:
            while True:
                point = command[1:-1].split(';')
                if len(point) == 2:
                    print(f'[{time.time()}] - [dequeue_bt] point : {point}')
                    try:
                        trajectory_points.append((float(point[1]),float(point[0])))
                        print(f'{float(point[1])},{float(point[0])}')
                    except:
                        pass
                try:
                    command = bt_queue.pop(0)
                except IndexError:
                    break
    bluetooth.write(*estimator.position, robot.speed, robot.omega)
    
### ----- ###    
rx_timer = Timer()
rx_timer.init(freq=10, mode=Timer.PERIODIC, callback=enqueue_bt)
queue_timer = Timer()
queue_timer.init(freq=10, mode=Timer.PERIODIC, callback=dequeue_bt)

### ----- ###    
while True:
    pass
     