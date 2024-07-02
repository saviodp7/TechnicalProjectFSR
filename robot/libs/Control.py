import numpy as np
from scipy.integrate import odeint
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

class InputOutputLinearization:
    def __init__(self, b=0.05, k1=8, k2=8, trajectory_points=None):
        self.b = b
        self.k1 = k1
        self.k2 = k2

        if trajectory_points is None:
            trajectory_points = np.array([[0, 0], [1, 2], [2, 4], [4, 6], [6, 8], [8, 10]])

        self.trajectory_points = trajectory_points
        self.x_interp = interp1d(trajectory_points[:, 0], trajectory_points[:, 0], kind='linear',
                                 fill_value="extrapolate")
        self.y_interp = interp1d(trajectory_points[:, 0], trajectory_points[:, 1], kind='linear',
                                 fill_value="extrapolate")

    def unicycle_kinematics(self, state, t, v, omega):
        x, y, theta = state
        dx_dt = v * np.cos(theta)
        dy_dt = v * np.sin(theta)
        dtheta_dt = omega
        return [dx_dt, dy_dt, dtheta_dt]

    def desired_trajectory(self, t):
        x_d = self.x_interp(t)
        y_d = self.y_interp(t)
        return x_d, y_d

    def compute_control_inputs(self, state, t):
        x, y, theta = state
        x_d, y_d = self.desired_trajectory(t)

        y1 = x + self.b * np.cos(theta)
        y2 = y + self.b * np.sin(theta)

        y1_d, y2_d = x_d, y_d

        y1_dot_d = 1.0
        y2_dot_d = 1.0

        u1 = y1_dot_d + self.k1 * (y1_d - y1)
        u2 = y2_dot_d + self.k2 * (y2_d - y2)

        T_inv = np.array([[np.cos(theta), np.sin(theta)],
                          [-np.sin(theta) / self.b, np.cos(theta) / self.b]])

        v, omega = T_inv @ np.array([u1, u2])
        return v, omega

    def simulate_unicycle(self, initial_state, t):
        states = [initial_state]
        for i in range(1, len(t)):
            current_state = states[-1]
            v, omega = self.compute_control_inputs(current_state, t[i - 1])
            next_state = odeint(self.unicycle_kinematics, current_state, [t[i - 1], t[i]], args=(v, omega))[1]
            states.append(next_state)
        return np.array(states)

if __name__ == "__main__":
    initial_state = [0, 0, 0]
    t = np.linspace(0, 10, 500)

    controller = InputOutputLinearization()
    states = controller.simulate_unicycle(initial_state, t)

    desired_x = [controller.desired_trajectory(time)[0] for time in t]
    desired_y = [controller.desired_trajectory(time)[1] for time in t]

    plt.figure()
    plt.plot(states[:, 0], states[:, 1], label='Unicycle Path')
    plt.plot(desired_x, desired_y, 'r--', label='Desired Path')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.title('Unicycle Trajectory Tracking')
    plt.grid()
    plt.show()

