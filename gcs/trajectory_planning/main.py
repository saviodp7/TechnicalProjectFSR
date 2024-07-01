import random as rd
import numpy as np
import reeds_shepp as rs

LINEAR_TYPE = 0
TRAP_VEL_TYPE = 1
CUBIC_POL_TYPE = 2

class Traj_planning:
    def __init__(self, gridmap=np.zeros([100, 100]), path_list=[]):
        self.gridmap = gridmap
        self.path_list = path_list

    def s_abscise(self, profile_type=LINEAR_TYPE, t0=0, tf=1, fs=10, qi=0, qf=1):
        time = np.linspace(t0, tf, int(fs * (tf - t0)))
        duration = tf - t0
        
        if profile_type == LINEAR_TYPE:
            t = np.linspace(t0, tf, int(fs * (tf - t0)))
            s = t / tf
            s_dot = np.gradient(s, t)
            s_dotdot = np.gradient(s_dot, t)
        
        elif profile_type == TRAP_VEL_TYPE:
            s = np.zeros_like(time)
            s_dot = np.zeros_like(time)
            s_dotdot = np.zeros_like(time)
            
            # Assume a trapezoidal profile with acceleration and deceleration phases
            acc_time = duration / 3  # Acceleration phase duration
            dec_time = duration / 3  # Deceleration phase duration
            const_time = duration - acc_time - dec_time  # Constant speed duration

            max_speed = 2 / (duration / 3) # da scegliere il criterio 
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
    
        elif profile_type == CUBIC_POL_TYPE:
            # Cubic polynomial profile
            s = 3 * (time / duration)**2 - 2 * (time / duration)**3
            s_dot = 6 * (time / duration) * (1 - (time / duration)) / duration
            s_dotdot = 6 * (1 - 2 * (time / duration)) / duration**2
        
        return s, s_dot, s_dotdot
    
    def cartesian_poly(self, qi = np.array([0, 0, 0]), qf = np.array([0, 1, 0]),fs = 10,profile_type=LINEAR_TYPE):
        # Define useful parameters
        t = 2  # first guess for the time
        k = 1  # the value of k is fixed
        
        alpha_x = k * np.cos(qf[2]) - 3 * qf[0]
        alpha_y = k * np.sin(qf[2]) - 3 * qf[1]
        beta_x = k * np.cos(qi[2]) + 3 * qi[0]
        beta_y = k * np.sin(qi[2]) + 3 * qi[1]

        # Calculate s
        s, s_dot, s_dotdot = self.s_abscise(profile_type=profile_type, t0=0, tf=t, fs=fs, qi=qi, qf=qf)

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

        v = v_tilde * s_dot
        w = w_tilde * s_dot
        return x,y,v,w
        
    def cartesian_traj(self):
        path_length = 0
        optimal_path = []
        for i in range(len(self.path_list) - 1):
            x_path,y_path,v,w = self.cartesian_poly(self.path_list[i], self.path_list[i + 1])
            combined = list(zip(x_path, y_path))
            optimal_path.append(combined)

        return optimal_path, path_length
            
            
    def Reed_sheep(self):
        path_length = 0
        optimal_path = []

        for i in range(len(self.path_list) - 1):
            path = rs.get_optimal_word(self.path_list[i], self.path_list[i + 1])
            optimal_path.append(path)
            path_length += rs.word_length(path)

        return optimal_path, path_length


def gen_path_nodes():
    # assign points to be followed
    pts = [(-6, -7), (-6, 0), (-4, 6),
           (0, 5), (0, -2), (-2, -6),
           (3, -5), (3, 6), (6, 4)]

    # generate path nodes (x, y, phi)
    path_nodes = []
    for i in range(len(pts) - 1):
        # 1. let the vectors point at their successor
        dx = pts[i + 1][0] - pts[i][0]
        dy = pts[i + 1][1] - pts[i][1]
        phi = np.arctan2(dy, dx)

        # 2. or generate random directions
        # phi = rd.random() * math.pi * 2

        path_nodes.append((pts[i][0], pts[i][1], phi))
    path_nodes.append((pts[-1][0], pts[-1][1], 0))  # assign the last node

    # 3. or generate a random path
    # path_nodes = []
    # for _ in range(10):
    #     path_nodes.append( (rd.randint(-7,7), rd.randint(-7,7), rd.random() * math.pi * 2) )

    return path_nodes


def main():
    path_nodes = gen_path_nodes()
    Test = Traj_planning(path_list=gen_path_nodes())
    optimal_path, path_length = Test.Reed_sheep()
    for i, path in enumerate(optimal_path):
        print('{}:\t{}'.format(i + 1, path))
    optimal_path, path_length = Test.cartesian_traj()
    print(optimal_path) 
    #ascissa_test=Test.s_abscise(TRAP_VEL_TYPE)
    #print(ascissa_test)

    #poly_test=Test.cartesian_poly(profile_type=TRAP_VEL_TYPE)
    #print(poly_test)



if __name__ == '__main__':
    main()
