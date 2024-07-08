from math import sin, cos, sqrt, atan2, pi
from ulab import numpy as np
from machine import Timer
import time

toRad = pi/180

class InputOutputLinearization:
    def __init__(self, robot, estimator, **kwargs):
        self.robot = robot
        self.estimator = estimator
        self.timer = Timer()
        self.trajectory_timers = list()
        self.desired_position = None
        self._desired_speed = None
        self.goal_reached = False
        self.current_point_index = 0
        self.trajectory_points = None
        self.dt = 0
        
        self.f_s = kwargs.get('f_s', 10)
        self.b = kwargs.get('b', 0.05)
        self.k1 = kwargs.get('k1', 0.01)
        self.k2 = kwargs.get('k2', 0.01)
        self.epsilon = kwargs.get('epsilon', 0.025)        
        
    def compute_control_input(self, state, des_position, speed):
        x, y, theta = state
        #print(f'[{time.time()}] - [Current state] x: {x} / y : {y} / theta : {theta}')
        y1 = x + self.b * cos(theta)
        y2 = y + self.b * sin(theta)
        y1_d, y2_d = des_position
        y1_dot_d, y2_dot_d = speed
        #print(f'[{time.time()}] - [Controller desired speed] y1_dot_d: {y1_dot_d} / y2_dot_d : {y2_dot_d}')

        u1 = y1_dot_d + self.k1 * (y1_d - y1)
        u2 = y2_dot_d + self.k2 * (y2_d - y2)

        T = np.array([[np.cos(theta), -self.b*np.sin(theta)],
                      [np.sin(theta), self.b*np.cos(theta)]])
        T_inv = np.linalg.inv(T)
        v, omega = np.dot(T_inv, np.array([u1, u2]))
        
        return v, omega/2
    
    def compute_desired_speed(self):
        dx = self.desired_position[0] - self.estimator.position[0]
        dy = self.desired_position[1] - self.estimator.position[1]
        theta = np.arctan2(dy, dx)
        speed = [self.robot.max_vel*cos(theta),self.robot.max_vel*sin(theta)]
        #print(f'[{time.time()}] - [Computed Speed] speed: {speed}')
        return speed
    
    def go(self, desired_position: Tuple[float, float]):
        if all(isinstance(x, (float, int)) for x in desired_position) and len(desired_position) == 2:
            self.desired_position = desired_position
        else:
            raise ValueError("Not valid desired position, insert (x,y)")
        self.goal_reached = False
        #print(f'[{time.time()}] - [InputOutputLinearization.go] desired_position: {desired_position}')
        self.timer.init(freq=self.f_s, mode=Timer.PERIODIC, callback=self.control_to_point)
        
    def control_to_point(self, timer):
        current_state = self.estimator.position
        if np.linalg.norm(np.ndarray(current_state[:2]) - np.ndarray(self.desired_position)) < self.epsilon:
            self.goal_reached = True
            self.robot.stop()
            self.timer.deinit()
        else:
            speed = self.compute_desired_speed()
            v, omega = self.compute_control_input(self.estimator.position, self.desired_position, speed)
            #print(f'[{time.time()}] - [Computed controls] v: {v} / omega: {omega}')
            self.robot.go(v, angular_velocity_rad=omega)
        
    def execute_trajectory(self, trajectory_points, dt=1):
        self.trajectory_timers = [Timer() for _ in range(len(trajectory_points))]
        for index, point in enumerate(trajectory_points):
            self.trajectory_timers[index].init(mode=Timer.ONE_SHOT, period=int(dt*index* 1000), callback=lambda t, p=point: self.go(p))

class PostureRegulation:
    def __init__(self, robot, estimator, **kwargs):
        self.robot = robot
        self.estimator = estimator
        self.timer = Timer()
        self.trajectory_timers = list()
        self.desired_position = None
        self.goal_reached = False
        self.current_point_index = 0
        self.trajectory_points = None
        self.dt = 0

        self.f_s = kwargs.get('f_s', 10)
        self.k1 = kwargs.get('k1', 1)
        self.k2 = kwargs.get('k2', 0.05)
        self.k3 = kwargs.get('k3', 0.05)
        self.epsilon = kwargs.get('epsilon', 0.025)
        self.epsilon_angle = kwargs.get('epsilon_angle', 5*toRad)

    def compute_polar_coordinates(self):
        x, y, theta = [a - b for a, b in zip(self.estimator.position, self.desired_position)]
        #print(f'[{time.time()}] - [PostureRegulation.compute_polar_coordinates] x: {x} / y : {y} / theta : {theta}')
        rho = sqrt(x**2+y**2)
        #print(f'[{time.time()}] - [PostureRegulation.compute_polar_coordinates] atan2(y, x): {atan2(y, x)}')
        gamma = atan2(-y, x) - theta + pi
        delta = gamma + theta
        return rho, gamma, delta

    def compute_control_input(self):
        rho, gamma, delta = self.compute_polar_coordinates()
        #print(f'[{time.time()}] - [PostureRegulation.compute_control_input] rho: {rho} / gamma : {gamma} / delta : {delta}')
        v = self.k1 * rho * cos(gamma)
        try:
            omega = self.k2 * gamma + self.k1 * sin(gamma) * cos(gamma) * (1 + self.k3 * delta / gamma)
        except ZeroDivisionError:
            omega = 0
        print(f'[{time.time()}] - [PostureRegulation.compute_control_input] v: {v} / omega: {omega}')
        return v, omega/2

    def control_to_point(self, timer):
        current_state = self.estimator.position
        if np.linalg.norm(np.array(current_state[:2]) - np.array(self.desired_position[:2])) < self.epsilon and current_state[2]-self.desired_position[2]< self.epsilon_angle:
            self.goal_reached = True
            self.robot.stop()
            self.timer.deinit()
        else:
            v, omega = self.compute_control_input()
            self.robot.go(v, angular_velocity_rad=omega)

    def go(self, desired_position):
        if all(isinstance(x, (float, int)) for x in desired_position) and len(desired_position) == 2:
            self.desired_position = [desired_position[0], -desired_position[1], 0]
            print(f'[{time.time()}] - [PostureRegulation.go] self.desired_position: {self.desired_position}')
        elif all(isinstance(x, (float, int)) for x in desired_position) and len(desired_position) == 3:
            self.desired_position = [desired_position[0], -desired_position[1], desired_position[2]]
        else:
            raise ValueError("Not valid desired position, insert (x,y) of (x,y,theta)") 
        self.goal_reached = False
        self.timer.init(freq=self.f_s, mode=Timer.PERIODIC, callback=self.control_to_point)

    def execute_trajectory(self, trajectory_points, dt=1):
        self.trajectory_timers = [Timer() for _ in range(len(trajectory_points))]
        for index, point in enumerate(trajectory_points):
            self.trajectory_timers[index].init(mode=Timer.ONE_SHOT, period=int(dt * index * 1000),
                                               callback=lambda t, p=point: self.go(p))
