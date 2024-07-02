import numpy as np
from scipy.integrate import odeint
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt


class InputOutputLinearization:
    def __init__(self, b=0.05, k1=50, k2=50, trajectory_points=None):
        self.b = b
        self.k1 = k1
        self.k2 = k2

        if trajectory_points is None:
            trajectory_points = np.array([[0, 0], [15, 8], [2, 9], [3, 5], [4, 4], [5, 0]])

        self.trajectory_points = trajectory_points
        self.num_segments = len(trajectory_points) - 1

    def unicycle_kinematics(self, state, t, v, omega):
        x, y, theta = state
        dx_dt = v * np.cos(theta)
        dy_dt = v * np.sin(theta)
        dtheta_dt = omega
        return [dx_dt, dy_dt, dtheta_dt]

    def desired_trajectory(self, t):
        segment_duration = 10 / self.num_segments
        segment_index = int(t // segment_duration)
        if segment_index >= self.num_segments:
            segment_index = self.num_segments - 1

        start_point = self.trajectory_points[segment_index]
        end_point = self.trajectory_points[segment_index + 1]

        segment_start_time = segment_index * segment_duration
        segment_end_time = (segment_index + 1) * segment_duration
        ratio = (t - segment_start_time) / (segment_end_time - segment_start_time)

        x_d = start_point[0] + ratio * (end_point[0] - start_point[0])
        y_d = start_point[1] + ratio * (end_point[1] - start_point[1])
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
    t = np.linspace(0, 10, 8000)

    controller = InputOutputLinearization()
    states = controller.simulate_unicycle(initial_state, t)

    desired_x = [controller.desired_trajectory(time)[0] for time in t]
    desired_y = [controller.desired_trajectory(time)[1] for time in t]

    errors = np.sqrt((states[:, 0] - desired_x) ** 2 + (states[:, 1] - desired_y) ** 2)
    mean_error = np.mean(errors)

    plt.figure()
    plt.plot(t, errors, label='Tracking Error')
    plt.xlabel('Time')
    plt.ylabel('Error')
    plt.legend()
    plt.title(f'Tracking Error Over Time (Mean Error: {mean_error:.2f})')
    plt.grid()
    plt.figure()
    plt.plot(states[:, 0], states[:, 1], label='Unicycle Path')
    plt.plot(desired_x, desired_y, 'r--', label='Desired Path')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.title('Unicycle Trajectory Tracking')
    plt.grid()

    plt.show()

