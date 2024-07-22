import numpy as np
if __name__ == "__main__":
    import reeds_shepp as rs
    from ..motion_planning.gridmap import CELL_SIZE, OFFSET
else:
    from . import reeds_shepp as rs
    from motion_planning.gridmap import CELL_SIZE, OFFSET
from typing import List, Tuple
import matplotlib.pyplot as plt
import math
import pygame
import threading

LINEAR_PROF = 0
TRAP_VEL_PROF = 1
CUBIC_POL_PROF = 2


def get_s(t_0=0, t_f=1, f_s=10, profile=LINEAR_PROF, s_dot_f=0,max_vel=100):
    time = np.linspace(t_0, t_f, int(f_s * (t_f - t_0)))
    duration = t_f - t_0

    if profile == LINEAR_PROF:
        t = np.linspace(t_0, t_f, int(f_s * (t_f - t_0)))
        s = t / t_f
        s_dot = np.gradient(s, t)
        s_dotdot = np.gradient(s_dot, t)

    elif profile == TRAP_VEL_PROF:
              
        max_speed = max_vel 
        duration=1/(2*max_vel)
        acc=3*max_vel/duration
        const_time = dec_time = acc_time = duration / 3
        
        time = np.linspace(t_0, duration, int(f_s * (duration - t_0)))
        s = np.zeros_like(time)
        s_dot = np.zeros_like(time)
        s_dotdot = np.zeros_like(time)
        
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
                s_dot[i] = max_speed - acc * t_dec + max_speed * const_time + max_speed * t_dec - 0.5 * acc * t_dec**2
                s_dotdot[i] = -acc

        #s /= s[-1]  # Normalize s to range [0, 1]

    elif profile == CUBIC_POL_PROF and not s_dot_f:
        # Cubic polynomial profile
        s = 3 * (time / duration)**2 - 2 * (time / duration)**3
        s_dot = 6 * (time / duration) * (1 - (time / duration)) / duration
        s_dotdot = 6 * (1 - 2 * (time / duration)) / duration**2
    elif profile == CUBIC_POL_PROF and s_dot_f:
        a3 = (2 * (s_dot_f * t_f - 1)) / (t_f ** 3)
        a2 = (-3 * s_dot_f / t_f) + (3 / t_f ** 2)
        a1 = s_dot_f
        a0 = 0
        s = a3 * time**3 + a2 * time**2 + a1 * time + a0
        s_dot = 3 * a3 * time**2 + 2 * a2 * time + a1
        s_dotdot = 6 * a3 * time + 2 * a2
    return s, s_dot, s_dotdot



class TrajectoryPlanner:
    def __init__(self, gridmap, path, f_s=10, k=2, profile=LINEAR_PROF, scaling=False, max_v=9):
        self.gridmap = gridmap
        self.f_s = f_s
        self.profile = profile
        self.k = k
        self.scaling = scaling
        self.max_v = max_v
        self._path = self.get_path_poses(path)
        self.cartesian_path: List[Tuple[x:float, y:float, theta:float, x_dot:float, y_dot:float, theta_dot:float]] = []
        self.generate_cartesian_traj()

    def generate_cartesian_traj(self):
        path = speed = theta = theta_dot = list()
        for i in range(len(self._path) - 1):
            x_path, y_path, x_speed, y_speed, orientation, w = self.cartesian_poly(self._path[i], self._path[i + 1])
            for x, y, orientation, x_dot, y_dot, w in zip(list(x_path[1:]), list(y_path[1:]), list(x_speed[1:]), list(y_speed[1:]), list(orientation[1:]), list(w[1:])):
                path.append((x, y))
                speed.append((x_dot, y_dot))
                theta.append(orientation)
                theta_dot.append(w)
                self.cartesian_path.append((x, y, orientation, x_dot, y_dot, w))
        return self.cartesian_path

    def cartesian_poly(self, qi, qf, t=1):
        alpha_x = self.k * np.cos(qf[2]) - 3 * qf[0]
        alpha_y = self.k * np.sin(qf[2]) - 3 * qf[1]
        beta_x = self.k * np.cos(qi[2]) + 3 * qi[0]
        beta_y = self.k * np.sin(qi[2]) + 3 * qi[1]
        # Calculate s


        s, s_dot, s_dotdot = get_s(t_0=0, t_f=t, f_s=self.f_s, profile=self.profile,s_dot_f=0.05,max_vel=self.max_v)

        # Calculate x, y, x_first_dot, y_first_dot, x_first_ddot, y_first_ddot and theta
        x = s ** 3 * qf[0] - (s - 1) ** 3 * qi[0] + alpha_x * s ** 2 * (s - 1) + beta_x * s * (s - 1) ** 2
        y = s ** 3 * qf[1] - (s - 1) ** 3 * qi[1] + alpha_y * s ** 2 * (s - 1) + beta_y * s * (s - 1) ** 2
        x_first_dot = 3 * s ** 2 * qf[0] - 3 * (s - 1) ** 2 * qi[0] + alpha_x * s * (3 * s - 2) + beta_x * (
                    3 * s - 1) * (s - 1)
        y_first_dot = 3 * s ** 2 * qf[1] - 3 * (s - 1) ** 2 * qi[1] + alpha_y * s * (3 * s - 2) + beta_y * (
                    3 * s - 1) * (s - 1)
        x_first_ddot = 6 * s * qf[0] - 6 * (s - 1) * qi[0] + alpha_x * (6 * s - 2) + beta_x * (6 * s - 4)
        y_first_ddot = 6 * s * qf[1] - 6 * (s - 1) * qi[1] + alpha_y * (6 * s - 2) + beta_y * (6 * s - 4)
        theta = np.arctan2(y_first_dot, x_first_dot)
        # Get x_dot, y_dot
        v_tilde = np.sqrt(x_first_dot ** 2 + y_first_dot ** 2)
        w_tilde = (y_first_ddot * x_first_dot - x_first_ddot * y_first_dot) / (x_first_dot ** 2 + y_first_dot ** 2)
        x_dot = x_first_dot * s_dot
        y_dot = y_first_dot * s_dot
        v = v_tilde * s_dot
        w = w_tilde * s_dot
        #print(f'max_vel: {max(abs(v))}, max_ang_vel: {max(abs(w))}')
        if self.scaling and max(abs(v)) > self.max_v:
            T = max(abs(v)) / self.max_v
            x, y, theta, x_dot, y_dot, w = self.cartesian_poly(qi, qf, t * math.ceil(T))
        return x, y, theta, x_dot, y_dot, w

    @property
    def path(self):
        return self._path

    @path.setter
    def path(self, path):
        self._path = self.get_path_poses(path)
        self.cartesian_path = []
        self.generate_cartesian_traj()

    # TODO: Correggere algoritmo
    # def reed_sheep(self):
    #     optimal_path, path_length = self.rs.optimal_reed_sheep()
    #     return optimal_path, path_length

    def get_path_poses(self, pts):
        path_nodes = list()
        for i in range(len(pts) - 1):
            dx = pts[i + 1][0] - pts[i][0]
            dy = pts[i + 1][1] - pts[i][1]
            phi = np.arctan2(dy, dx)
            path_nodes.append((pts[i][0], pts[i][1], phi))
        path_nodes.append((pts[-1][0], pts[-1][1], path_nodes[-1][2]))
        return path_nodes
    
    def draw(self, screen):
        for x_pnt, y_pnt, _, _, _, _ in self.cartesian_path:
            pygame.draw.circle(screen, 'purple', (x_pnt*CELL_SIZE + OFFSET, y_pnt*CELL_SIZE + OFFSET), 4)

    def plot(self):
        x, y, theta, x_dot, y_dot, theta_dot = map(list, zip(*self.cartesian_path))
        plt.ion()
        # plt.plot(x, y, color='blue', marker='o', linewidth=1, markersize=3)
        # for (px, py, dx, dy) in zip(x, y, x_dot, y_dot):
        #     plt.annotate('', xy=(px + dx / 50, py + dy / 50), xytext=(px, py),
        #                  arrowprops=dict(facecolor='red', shrink=0.05, width=5, headwidth=5, headlength=7))
        # plt.xlabel('x (m)')
        # plt.ylabel('y (m)')
        # plt.title('Path and orientation')
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
