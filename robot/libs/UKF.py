from ulab import numpy as np
from math import sin, cos
from vect_utils import *

class MerweScaledSigmaPoints(object):

    def __init__(self, n, alpha, beta, kappa, subtract):
        self.n = n
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        self.sqrt = np.linalg.cholesky
        self.subtract = subtract
        self._compute_weights()

    def num_sigmas(self):
        return 2*self.n + 1

    def sigma_points(self, x, P): 
        
        if self.n != np.size(x):
            raise ValueError("expected size(x) {}, but size is {}".format(
                self.n, np.size(x)))
        n = self.n
        lambda_ = self.alpha**2 * (n + self.kappa) - n
        U=self.sqrt(multiply_scalar_matrix((lambda_ + n),P))
        sigmas = np.zeros((2*n+1, n))
        sigmas[0] = x
        for k in range(n):
            sigmas[k+1]   = self.subtract(x, -1*U[k])
            sigmas[n+k+1] = self.subtract(x, U[k])
        return sigmas

    def _compute_weights(self):
        n = self.n
        lambda_ = self.alpha**2 * (n +self.kappa) - n
        c = .5 / (n + lambda_)
        self.Wc = np.full(2*n + 1, c)
        self.Wm = np.full(2*n + 1, c)
        self.Wc[0] = lambda_ / (n + lambda_) + (1 - self.alpha**2 + self.beta)
        self.Wm[0] = lambda_ / (n + lambda_)




class UnscentedKalmanFilter(object):
   
    def __init__(self, dim_x, dim_z, dt, hx, fx, points,
                 sqrt_fn=None, x_mean_fn=None, z_mean_fn=None,
                 residual_x=None, residual_z=None, state_add=None):

        self.x = np.zeros(dim_x)
        self.P = np.eye(dim_x)
        self.x_prior = deep_copy_vect(self.x)
        self.P_prior = deep_copy(self.P)
        self.Q = multiply_scalar_matrix(0.01,np.eye(dim_x))
        self.R = np.eye(dim_z)
        self._dim_x = dim_x
        self._dim_z = dim_z
        self.points_fn = points
        self._dt = dt
        self._num_sigmas = points.num_sigmas()
        self.hx = hx
        self.fx = fx
        self.x_mean = x_mean_fn
        self.z_mean = z_mean_fn
        self.Wm, self.Wc = points.Wm, points.Wc
        self.residual_x = residual_x
        self.residual_z = residual_z
        if sqrt_fn is None:
            self.msqrt = np.linalg.cholesky
        else:
            self.msqrt = sqrt_fn
        if state_add is None:
            self.state_add = add_vectors
        else:
            self.state_add = state_add

        # sigma points transformed through f(x) and h(x)
        # variables for efficiency so we don't recreate every update
        self.sigmas_f = np.zeros((self._num_sigmas, self._dim_x))
        self.sigmas_h = np.zeros((self._num_sigmas, self._dim_z))

        self.K = np.zeros((dim_x, dim_z))    # Kalman gain
        self.y = np.zeros((dim_z))           # residual
        self.z = np.zeros((dim_z))           # measurement
        self.S = np.zeros((dim_z, dim_z))    # system uncertainty
        self.SI = np.zeros((dim_z, dim_z))   # inverse system uncertainty

        self.inv = np.linalg.inv

        # these will always be a copy of x,P after predict() is called
        self.x_prior = self.x.copy()
        self.P_prior = self.P.copy()

        # these will always be a copy of x,P after update() is called
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()

    def predict(self, dt=None, UT=None, fx=None, **fx_args):
        if dt is None:
            dt = self._dt

        if UT is None:
            UT = unscented_transform

        # calculate sigma points for given mean and covariance
        self.compute_process_sigmas(dt, fx, **fx_args)
        
        #and pass sigmas through the unscented transform to compute prior
        self.x, self.P = UT(self.sigmas_f, self.Wm, self.Wc, self.Q, self.x_mean, self.residual_x)

        # update sigma points to reflect the new variance of the points
        self.sigmas_f = self.points_fn.sigma_points(self.x, self.P)
        
        # save prior
        self.x_prior = deep_copy_vect(self.x)
        self.P_prior = deep_copy(self.P)

    def update(self, z, R=None, UT=None, hx=None, **hx_args):
        if hx is None:
            hx = self.hx
        if UT is None:
            UT = unscented_transform
        if R is None:
            R = self.R
        
        for i,s in enumerate(self.sigmas_f):
            self.sigmas_h[i]=hx(s, **hx_args)
        #self.sigmas_h = [[hx(s, **hx_args)] for s in self.sigmas_f] if not isinstance(self.sigmas_f[0], list) else [hx(s, **hx_args) for s in self.sigmas_f]
        #self.sigmas_h = np.atleast_2d(sigmas_h)

        # mean and covariance of prediction passed through unscented transform
        zp, self.S = UT(self.sigmas_h, self.Wm, self.Wc, R, self.z_mean, self.residual_z)
        self.SI = self.inv(self.S)
        
        # compute cross variance of the state and the measurements
        Pxz = self.cross_variance(self.x, zp, self.sigmas_f, self.sigmas_h)

        self.K = np.dot(Pxz, self.SI)     # Kalman gain
        self.y = self.residual_z(z, zp)   # residual
        
        # update Gaussian state estimate (x, P)
        self.x = add_vectors(self.x, np.dot(self.K, self.y))
        Matrix_reduced=threshold_matrix(np.dot(self.K, np.dot(self.S, self.K.T)),10e-3)
        #self.P = self.P - Matrix_reduced
        # save measurement and posterior state
        self.z = deep_copy_vect(z)
        self.x_post = deep_copy_vect(self.x)
        self.P_post = deep_copy(self.P)


    def cross_variance(self, x, z, sigmas_f, sigmas_h):
        Pxz = np.zeros((sigmas_f.shape[1], sigmas_h.shape[1]))
        N = sigmas_f.shape[0]
        for i in range(N):
            dx = self.residual_x(sigmas_f[i], x)
            dz = self.residual_z(sigmas_h[i], z)
            Pxz += multiply_scalar_matrix(self.Wc[i], outer_product(vector1=dx, vector2=dz))
        return Pxz

    def compute_process_sigmas(self, dt, fx=None, **fx_args):
        if fx is None:
            fx = self.fx
        # calculate sigma points for given mean and covariance
        sigmas = self.points_fn.sigma_points(self.x, self.P)
        for i, s in enumerate(sigmas):
            self.sigmas_f[i] = fx(s, dt, **fx_args)


def unscented_transform(sigmas, Wm, Wc, noise_cov=None, mean_fn=None, residual_fn=None):
    
    kmax, n = sigmas.shape
    x= mean_fn(sigmas, Wm)

    P = np.zeros((n, n))
    for k in range(kmax):
        y=residual_fn(sigmas[k], x)
        P = P + multiply_scalar_matrix(Wc[k],outer_product(y))
    if noise_cov is not None:
        P += noise_cov
        
    return (x, P)


def Hx(x):
    v_dot=x[8] 
    w_dot=x[6]
    return np.array([v_dot,w_dot])

def move(x, dt, u):
    hdg = x[4]
    x[6]=(u[1]-x[5])/dt
    x[8]=(u[0]-x[7])/dt
    x[5]=u[1]
    x[7]=u[0]
    x[1]=x[7] * cos(hdg) 
    x[3] =x[7] * sin(hdg) 
    x[0],x[2],x[4]=runge_kutta([x[0],x[2],x[4]],[x[7],x[5]],dt)
    return x


def kinematic_model(state, control, dt):
    x, y, theta = state
    v, omega = control
    dx = v * cos(theta)
    dy = v * sin(theta)
    dtheta = omega
    return np.array([dx, dy, dtheta])

def runge_kutta(state, control, dt):
    k1 = kinematic_model(state, control, dt)
    k2 = kinematic_model(state + 0.5 * dt * k1, control, dt)
    k3 = kinematic_model(state + 0.5 * dt * k2, control, dt)
    k4 = kinematic_model(state + dt * k3, control, dt)
    new_state = state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    return new_state

def residual_x(a, b):
    y = a - b
    return y

def state_mean(sigmas, Wm):
    x = np.zeros(9)
    x[0] = np.dot(sigmas[:,0],Wm)
    x[1] = np.dot(sigmas[:,1],Wm)
    x[2] = np.dot(sigmas[:,2],Wm)
    x[3] = np.dot(sigmas[:,3],Wm)
    x[4] = np.dot(sigmas[:,4],Wm)
    x[5] = np.dot(sigmas[:,5],Wm)
    x[6] = np.dot(sigmas[:,6],Wm)
    x[7] = np.dot(sigmas[:,7],Wm)
    x[8] = np.dot(sigmas[:,8],Wm)
    return x

def z_mean(sigmas, Wm):
    x = np.zeros(2)
    x[0] = np.dot(sigmas[:,0],Wm)
    x[1] = np.dot(sigmas[:,1],Wm)
    return x

def Q_discrete_white_noise(dim, dt=1., var=1., block_size=1):

    if dim not in [2, 3, 4]:
        raise ValueError("dim must be between 2 and 4")

    if dim == 2:
        Q = np.array([[.25*dt**4, .5*dt**3],[ .5*dt**3,    dt**2]])
    elif dim == 3:
        Q = np.array([[.25*dt**4, .5*dt**3, .5*dt**2],
             [ .5*dt**3,    dt**2,       dt],
             [ .5*dt**2,       dt,        1]])
    else:
        Q = p.array([[(dt**6)/36, (dt**5)/12, (dt**4)/6, (dt**3)/6],
             [(dt**5)/12, (dt**4)/4,  (dt**3)/2, (dt**2)/2],
             [(dt**4)/6,  (dt**3)/2,   dt**2,     dt],
             [(dt**3)/6,  (dt**2)/2 ,  dt,        1.]])

    return multiply_scalar_matrix(var,Q)





if __name__ == "__main__":
    dt=0.1
    proc_noise_vr=0.01
    points = MerweScaledSigmaPoints(n=9, alpha=0.1, beta=2, kappa=-6, subtract=residual_x)
    ukf=UnscentedKalmanFilter(dim_x=9, dim_z=2, fx=move, hx=Hx, dt=dt, points=points, x_mean_fn=state_mean, 
              z_mean_fn=z_mean, residual_x=residual_x, residual_z=residual_x)
    ukf.x = np.array([0.,0.,0.,0.,0.,0.,0.,0.,0.])
    ukf.P = np.diag([.01,.1,.01,.1,.005,.005,.05,.005,.05])
    ukf.R = np.diag([0.1**2,0.1**2])
    ukf.Q=block_diag(Q_discrete_white_noise(2, dt=dt, var=proc_noise_vr),
                     Q_discrete_white_noise(2, dt=dt, var=proc_noise_vr),
                     Q_discrete_white_noise(3, dt=dt, var=proc_noise_vr),
                     Q_discrete_white_noise(2, dt=dt, var=proc_noise_vr))
    sim_pos = np.array([0.,0.,0.,0.,0.,0.,0.,0.,0.])
    u=np.array([0.1,0])
    for i in range(5):

        sim_pos=move(sim_pos,dt,u)
        z=Hx(sim_pos)
        ukf.predict(u=u)
        #print("Move step",i,"x:",sim_pos[0],"y:",sim_pos[2],"theta:",sim_pos[4])
        print("Prediction step",i,"x:",ukf.x[0],"y:",ukf.x[2],"theta:",ukf.x[4])
        #print("Kalman gain",i,ukf.K)
        
        #ukf.update(z)
        #print("Mesurement",i,"v_dot:",z[0],"w_dot:",z[1])
        #print("Update step",i,"x:",ukf.x[0],"y:",ukf.x[2],"theta:",ukf.x[4])
    
    
