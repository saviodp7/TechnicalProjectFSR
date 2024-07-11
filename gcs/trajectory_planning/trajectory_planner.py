import numpy as np
if __name__ == "__main__":
    import reeds_shepp as rs
else:
    from . import reeds_shepp as rs
import matplotlib.pyplot as plt
import math

LINEAR_PROF = 0
TRAP_VEL_PROF = 1
CUBIC_POL_PROF = 2


def get_s(t_0=0, t_f=1, f_s=10, profile=LINEAR_PROF):
    time = np.linspace(t_0, t_f, int(f_s * (t_f - t_0)))
    duration = t_f - t_0

    if profile == LINEAR_PROF:
        t = np.linspace(t_0, t_f, int(f_s * (t_f - t_0)))
        s = t / t_f
        s_dot = np.gradient(s, t)
        s_dotdot = np.gradient(s_dot, t)

    elif profile == TRAP_VEL_PROF:
        s = np.zeros_like(time)
        s_dot = np.zeros_like(time)
        s_dotdot = np.zeros_like(time)
        
        # Assume a trapezoidal profile with acceleration and deceleration phases
        acc_time = duration / 3  # Acceleration phase duration
        dec_time = duration / 3  # Deceleration phase duration
        const_time = duration - acc_time - dec_time  # Constant speed duration

        max_speed = 2 / (duration / 3)  # TODO: Choose criteria
        acc = max_speed / acc_time
        
        for i, t in enumerate(time):
            if t < acc_time:
                s_dot[i] = acc * t
                s[i] = 0.5 * acc * t**2
                s_dotdot[i] = acc
            elif t < acc_time + const_time:
                s_dot[i] = max_speed
                s[i] = 0.5 * max_speed * acc_time + max_speed * (t - acc_time)
                s_dotdot[i] = 0
            else:
                t_dec = t - (acc_time + const_time)
                s_dot[i] = max_speed - acc * t_dec
                s[i] = (0.5 * max_speed * acc_time +
                        max_speed * const_time +
                        max_speed * t_dec - 0.5 * acc * t_dec**2)
                s_dotdot[i] = -acc

        s /= s[-1]  # Normalize s to range [0, 1]

    elif profile == CUBIC_POL_PROF:
        # Cubic polynomial profile
        s = 3 * (time / duration)**2 - 2 * (time / duration)**3
        s_dot = 6 * (time / duration) * (1 - (time / duration)) / duration
        s_dotdot = 6 * (1 - 2 * (time / duration)) / duration**2
    
    return s, s_dot, s_dotdot


class TrajectoryPlanner:
    def __init__(self, gridmap, path_list):
        self.gridmap = gridmap
        self.path_list = self.get_path_poses(path_list)

    @staticmethod
    def cartesian_poly(qi=np.array([0, 0, 0]), qf=np.array([0, 1, 0]), f_s=10, profile=LINEAR_PROF, t=1, trajectory_scaling=False, max_v=1, max_w=1):
        # Define useful parameters
        k = 0.5  # the value of k is fixed
        
        alpha_x = k * np.cos(qf[2]) - 3 * qf[0]
        alpha_y = k * np.sin(qf[2]) - 3 * qf[1]
        beta_x = k * np.cos(qi[2]) + 3 * qi[0]
        beta_y = k * np.sin(qi[2]) + 3 * qi[1]

        # Calculate s
        s, s_dot, s_dotdot = get_s(t_0=0, t_f=t, f_s=f_s, profile=profile)

        # Calculate x, y, x_first_dot, y_first_dot, x_first_ddot, y_first_ddot and theta
        x = s**3 * qf[0] - (s - 1)**3 * qi[0] + alpha_x * s**2 * (s - 1) + beta_x * s * (s - 1)**2
        y = s**3 * qf[1] - (s - 1)**3 * qi[1] + alpha_y * s**2 * (s - 1) + beta_y * s * (s - 1)**2

        x_first_dot = 3 * s**2 * qf[0] - 3 * (s - 1)**2 * qi[0] + alpha_x * s * (3 * s - 2) + beta_x * (3 * s - 1) * (s - 1)
        y_first_dot = 3 * s**2 * qf[1] - 3 * (s - 1)**2 * qi[1] + alpha_y * s * (3 * s - 2) + beta_y * (3 * s - 1) * (s - 1)

        x_first_ddot = 6 * s * qf[0] - 6 * (s - 1) * qi[0] + alpha_x * (6 * s - 2) + beta_x * (6 * s - 4)
        y_first_ddot = 6 * s * qf[1] - 6 * (s - 1) * qi[1] + alpha_y * (6 * s - 2) + beta_y * (6 * s - 4)

        theta = np.arctan2(y_first_dot, x_first_dot)

        # Get real x_dot, y_dot
        v_tilde = np.sqrt(x_first_dot**2 + y_first_dot**2)
        w_tilde = (y_first_ddot * x_first_dot - x_first_ddot * y_first_dot) / (x_first_dot**2 + y_first_dot**2)

        x_dot = x_first_dot * s_dot
        y_dot = y_first_dot * s_dot
        v = v_tilde * s_dot
        w = w_tilde * s_dot
        
        if trajectory_scaling:
            T = max(max(abs(v))/max_v, max(abs(w))/max_w)
            if T > 1.0:
                x, y, x_dot, y_dot, theta, w = TrajectoryPlanner.cartesian_poly(t=(t*math.ceil(T)), qi=qi, qf=qf, f_s=f_s, profile=profile, trajectory_scaling=trajectory_scaling, max_v=max_v, max_w=max_w)
        return x, y, x_dot, y_dot, theta, w

    def cartesian_traj(self, f_s=10, profile=LINEAR_PROF,trajectory_scaling=False, max_v=1, max_w=1):
        path, speed, theta, theta_dot = list(), list(), list(), list()
        for i in range(len(self.path_list) - 1):
            x_path, y_path, x_speed, y_speed, orientation,w = self.cartesian_poly(self.path_list[i], self.path_list[i + 1], f_s, profile=profile,trajectory_scaling=trajectory_scaling,max_v=max_v,max_w=max_w)
            for x, y, x_dot, y_dot, orientation,w in zip(list(x_path), list(y_path), list(x_speed), list(y_speed), list(orientation), list(w)):
                path.append((x, y))
                speed.append((x_dot, y_dot))
                theta.append(orientation)
                theta_dot.append(w)
        path.append(self.path_list[-1])
        x, y = zip(*path)
        x_dot, y_dot = zip(*speed)
        return x, y, x_dot, y_dot, theta, theta_dot

    def reed_sheep(self):
        reed_sheep = rs.ReedSheep(self.path_list)
        optimal_path, path_length = reed_sheep.optimal_reed_sheep()

        return optimal_path, path_length

    @staticmethod
    def get_path_poses(pts):
        path_nodes = list()
        for i in range(len(pts) - 1):
            dx = pts[i + 1][0] - pts[i][0]
            dy = pts[i + 1][1] - pts[i][1]
            phi = np.arctan2(dy, dx)
            path_nodes.append((pts[i][0], pts[i][1], phi))
        path_nodes.append((pts[-1][0], pts[-1][1], path_nodes[-1][2]))  # assign the last node
        return path_nodes


def generate_random_points(n, x_min, x_max, y_min, y_max):
    """
    Genera una lista di n coppie di numeri casuali tra x_min e x_max, y_min e y_max.

    Args:
        n (int): Numero di coppie da generare.
        x_min (float): Limite inferiore per il valore x.
        x_max (float): Limite superiore per il valore x.
        y_min (float): Limite inferiore per il valore y.
        y_max (float): Limite superiore per il valore y.

    Returns:
        list of tuples: Una lista di n coppie di numeri casuali (x, y).
    """
    x_values = np.random.uniform(x_min, x_max, n)
    y_values = np.random.uniform(y_min, y_max, n)
    points = [(x, y) for x, y in zip(x_values, y_values)]
    return points


if __name__ == '__main__':
    # pts = generate_random_points(10, x_min=0, x_max=2, y_min=0, y_max=1)
    path = [(-6, -7), (-6, 0), (-4, 6), (0, 5), (0, -2), (-2, -6), (3, -5), (3, 6), (6, 4)]
    TP = TrajectoryPlanner(np.zeros((100, 100)), path_list=path)
    max_v = 2
    max_w = 2
    x, y, x_dot, y_dot, theta, theta_dot = TP.cartesian_traj(f_s=20, profile=TRAP_VEL_PROF, trajectory_scaling=True, max_v=max_v, max_w=max_w)

    #optimal_path, path_length = Test.reed_sheep()

    # PLOTS
    plt.figure()
    plt.plot(x, y, color='blue', marker='o', linewidth=1, markersize=3)
    for (px, py, dx, dy) in zip(x, y, x_dot, y_dot):
        plt.annotate('', xy=(px + dx/50, py + dy/50), xytext=(px, py),
                     arrowprops=dict(facecolor='red', shrink=0.05, width=5, headwidth=5, headlength=7))
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title('Path and orientation')
    # x_dot Plot
    plt.figure()
    plt.plot(x_dot, color='green', linewidth=2, markersize=5)
    plt.xlabel('Time steps')
    plt.ylabel(r'$\dot{x}\ (m/s)$')
    plt.title(r'$\dot{x}$')
    plt.grid(True)
    # y_dot Plot
    plt.figure()
    plt.plot(y_dot, color='purple', linewidth=2, markersize=5)
    plt.xlabel('Time steps')
    plt.ylabel(r'$\dot{y}\ (m/s)$')
    plt.title(r'$\dot{y}$')
    plt.grid(True)
    # theta_dot Plot
    plt.figure()
    plt.plot(theta_dot, color='orange', linewidth=2, markersize=5)
    plt.xlabel('Time steps')
    plt.ylabel(r'$\dot{\theta}\ (rad/s)$')
    plt.title(r'$\dot{\theta}$ (angular velocity)')
    plt.grid(True)

    plt.show()
