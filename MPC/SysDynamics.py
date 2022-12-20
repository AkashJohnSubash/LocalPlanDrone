from casadi import *
# discretization model (e.g. x2 = f(x1, v, t) = x1 + v * dt)
# rot_3d_z = vertcat( horzcat(cos(theta), -sin(theta), 0),
#                     horzcat(sin(theta),  cos(theta), 0),
#                     horzcat(         0,           0, 1) )

# Mecanum wheel transfer function which can be found here: 
# https://www.researchgate.net/publication/334319114_Model_Predictive_Control_for_a_Mecanum-wheeled_robot_in_Dynamical_Environments
# J = (wheel_radius/4) * DM([ [         1,         1,          1,         1],
#                             [        -1,         1,          1,        -1],
#                             [-1/(Lx+Ly), 1/(Lx+Ly), -1/(Lx+Ly), 1/(Lx+Ly)] ])
# RHS = states + J @ controls * step_horizon  # Euler discretization
# RHS = rot_3d_z @ J @ controls
# maps controls from [va, vb, vc, vd].T to [vx, vy, omega].T

# # state symbolic variables
# x = SX.sym('x');        y = SX.sym('y');            z = SX.sym('z')     # (in inertial frame)
# psi = SX.sym('psi');    theta = SX.sym('theta');    phi = SX.sym('phi') # Roll, pitch, yaw respectively (in inertial frame)
# u = SX.sym('u');        v = SX.sym('v');            w = SX.sym('w')     # linear velocities w.r.t x,y,z ()
# p = SX.sym('p');        q = SX.sym('q');            r = SX.sym('r')     # angular velocities w.r.t psi, theta, phi
# states = vertcat(x, y, z, psi, theta, phi, u, v, w, p, q ,r)
# 2D DEBUG
x = SX.sym('x');        y = SX.sym('y');            theta = SX.sym('theta')     # (in inertial frame)
states = vertcat(x, y, theta)

# # control symbolic variables (Motor RPM)
# w1 = SX.sym('w1');        w2 = SX.sym('w2');        w3 = SX.sym('w3');        w4 = SX.sym('w4')
# controls = vertcat( w1, w2, w3, w4) 
#2D DEBUG
v = SX.sym('v');        w = SX.sym('w')
controls = vertcat( v, w) 

class SysDyn():
    
    def __init__(self, N):


        self.stSize = states.numel()
        self.ctrlSize = controls.numel()

        # matrices containing all States, Controls, Paramters over all time steps +1
        self.St = SX.sym('St', self.stSize, N + 1)
        self.U = SX.sym('U', self.ctrlSize, N)
        self.P = SX.sym('P', self.stSize + self.stSize)

    def TransferFunction(self):
        # Transfer function : input (Ctrl)- output(State) mapping
        fnc_op = vertcat(v*cos(theta), v*sin(theta), w)
        fp = Function('f', [states, controls], [fnc_op])
        return fp 

    def TimeStep(step_horizon, t0, state_init, u, f):
        f_value = f(state_init, u[:, 0])
        next_state = DM.full(state_init + (step_horizon * f_value))

        t0 = t0 + step_horizon
        u0 = horzcat( u[:, 1:], reshape(u[:, -1], -1, 1))

        return t0, next_state, u0


class Predictor:

    def rk4_integrator(TransFunc, state, ctrl, step_horizon):
        k1 = TransFunc(state, ctrl)
        k2 = TransFunc(state + step_horizon/2*k1, ctrl)
        k3 = TransFunc(state + step_horizon/2*k2, ctrl)
        k4 = TransFunc(state + step_horizon * k3, ctrl)
        st_next_RK4 = state + (step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
        return st_next_RK4