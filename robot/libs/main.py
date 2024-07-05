from stepper import Stepper
import time
from machine import Timer, Pin
from wheeled import DifferentialDrive
from bluetooth import Bluetooth
from imu import IMU
from control import InputOutputLinearization
import localization as ul

def loop(timer):       
    # BT reading and control
    data_list = bluetooth.read()
#     if data_list and len(data_list)==1 and not ("" in data_list):
#         start = not start        
    if data_list and len(data_list)==2 and not ("" in data_list):
        command = dict()
        command["speed"] = float(data_list[0])
        command["angle"] = float(data_list[1])
        robot.go(command["speed"], angular_velocity_deg=command["angle"]*60)
    if data_list and len(data_list)==3 and not ("" in data_list):
        command = dict()
        command["x"] = float(data_list[0])
        command["y"] = float(data_list[1])
        command["theta"] = float(data_list[2])
        controller.go((x, y))
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
motor_dx = Stepper(3, 2, 6, 7, 8,resolution=16)
motor_sx = Stepper(15 ,14, 11, 12, 13, resolution=16)
robot = DifferentialDrive(motor_sx, motor_dx, wheels_diameter_m=12.51e-2, track_width_m=14.21e-2, max_vel=0.1)
time.sleep(0.5)
### INITIALIZATION ###
# Motor initialization check
# robot.go(1,0)
# time.sleep(0.01)
# robot.go(-1,0)
# time.sleep(0.01)
# robot.stop()
# Localization and bluetooth
estimator = ul.UnicycleLocalization(robot, imu)
loop_timer = Timer()
loop_timer.init(freq=10, mode=Timer.PERIODIC, callback=loop)
# CONTROLLER
controller = InputOutputLinearization(robot, estimator)
### TESTS ###
controller.go((1.4,-0.4))
# ----------------
# robot.go(0.1, 0)
# stop_timer = Timer()
# stop_timer.init(mode=Timer.ONE_SHOT, period=5000, callback=lambda t: robot.stop())
### ----- ###    
while True:
    pass
     