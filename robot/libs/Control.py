from math import sin, cos
from ulab import numpy as np
from machine import Timer
import time

class InputOutputLinearization:
    def __init__(self, robot, estimator, **kwargs):
        self.robot = robot
        self.estimator = estimator
        self.timer = Timer()
        self.desired_position = None
        self._desired_speed = None
        self.goal_reached = False
        
        self.f_s = kwargs.get('f_s', 10)
        self.b = kwargs.get('b', 0.075)
        self.k1 = kwargs.get('k1', 0.05)
        self.k2 = kwargs.get('k2', 0.05)
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
        
        return v, omega
    
    def compute_desired_speed(self):
        dx = self.desired_position[0] - self.estimator.position[0]
        dy = self.desired_position[1] - self.estimator.position[1]
        theta = np.arctan2(dy, dx)
        speed = [self.robot.max_vel*cos(theta),self.robot.max_vel*sin(theta)]
        #print(f'[{time.time()}] - [Computed Speed] speed: {speed}')
        return speed
    
    def go(self, desired_position: Tuple[float, float]):
        self.desired_position = desired_position
        #print(f'[{time.time()}] - [Computed Speed] self.desired_speed: {self.desired_speed}')
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
        # TODO: Execute trajectory
        for point in trajectory_points:
            if type(point) is dict:
                # TODO: Traj con timestamp
            else:
                self.go(point)
                time.sleep(dt)
# 
#     def simulate_unicycle(self, initial_state, t):
#         states = [initial_state]
#         for i in range(1, len(t)):
#             current_state = states[-1]
#             v, omega = self.compute_control_inputs(current_state, t[i - 1])
#             next_state = odeint(self.unicycle_kinematics, current_state, [t[i - 1], t[i]], args=(v, omega))[1]
#             states.append(next_state)
#         return np.array(states)
#     
#     def unicycle_kinematics(self, state: Tuple[float, float, float], t, v, omega):
#         x, y, theta = state
#         dx_dt = v*cos(theta)
#         dy_dt = v*sin(theta)
#         dtheta_dt = omega
#         return [dx_dt, dy_dt, dtheta_dt]
#     
# if __name__ == "__main__":
# 
#     initial_state = [0, 0, 0]
#     controller = InputOutputLinearization()
#     states = controller.simulate_unicycle(initial_state, t)
# 
#     desired_x = [controller.desired_trajectory(time)[0] for time in t]
#     desired_y = [controller.desired_trajectory(time)[1] for time in t]

