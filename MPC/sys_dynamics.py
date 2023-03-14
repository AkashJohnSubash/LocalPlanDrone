from casadi import *
from common import *

'-------------------Symbolic variables---------------------------'
# State variables
x = SX.sym('x');    y = SX.sym('y');    z = SX.sym('z')                         # (in inertial frame)
q1 = SX.sym('q1');  q2 = SX.sym('q2');  q3 = SX.sym('q3');  q4 = SX.sym('q4')   # quarternion angle representation
u = SX.sym('u');    v = SX.sym('v');    w = SX.sym('w')                         # linear velocities (in body frame)
p = SX.sym('p');    q = SX.sym('q');    r = SX.sym('r')                         # angular velocities w.r.t phi(roll), theta(pitch), psi(yaw)

# Time derivate of state (for implicit integrator)
x_dt = SX.sym('x_dt');    y_dt = SX.sym('y_dt');    z_dt = SX.sym('z_dt')                               # (in inertial frame)
q1_dt = SX.sym('q1_dt');  q2_dt = SX.sym('q2_dt');  q3_dt = SX.sym('q3_dt');  q4_dt = SX.sym('q4_dt')   # quarternion angle representation
u_dt = SX.sym('u_dt');    v_dt = SX.sym('v_dt');    w_dt = SX.sym('w_dt')                               # linear velocities (in body frame)
p_dt = SX.sym('p_dt');    q_dt = SX.sym('q_dt');    r_dt = SX.sym('r_dt')                               # angular velocities w.r.t phi(roll), theta(pitch), psi(yaw)

state = vertcat(x, y, z, q1, q2, q3, q4, u, v, w, p, q, r)
state_dt = vertcat(x_dt, y_dt, z_dt, q1_dt, q2_dt, q3_dt, q4_dt, u_dt, v_dt, w_dt, p_dt, q_dt, r_dt)

# Control variable angles (Motor RPM)
w1 = SX.sym('w1');  w2 = SX.sym('w2');  w3 = SX.sym('w3');  w4 = SX.sym('w4')
controls = vertcat( w1, w2, w3, w4) 

class SysDyn():
    
    def init():
        
        stSize = state.numel()
        ctrlSize = controls.numel()

        # matrices containing all States, Controls, Paramters over all time steps +1
        St = SX.sym('St', stSize, hznLen + 1)
        U = SX.sym('U', ctrlSize, hznLen)
        P = SX.sym('P', stSize + stSize)

        return St, U, P        

    def model_ode():
        '''ODEs for system dynamic model'''

        # Rate of change of position
        dx = u*(2*(q1**2 + q2**2)- 1)  + v*2*(q2*q3 - q1*q4)           + w*2*(q1*q3 + q2*q4)
        dy = u*2*(q1*q4 + q2*q3)       + v*(2*(q1**2 + q3**2) - 1)     + w*2*(q3*q4 - q1*q2)
        dz = u*2*(q2*q4 - q1*q3)       + v*2*(q1*q2 + q3*q4)           + w*(2*(q1**2 + q4**2) - 1) 
        
        # Rate of change of angles (in qaurternion)
        dq1 = - (q2*p)/2 - (q3*q)/2 - (q4*r)/2
        dq2 =   (q1*p)/2 - (q4*q)/2 + (q3*r)/2
        dq3 =   (q4*p)/2 + (q1*q)/2 - (q2*r)/2
        dq4 = - (q3*p)/2 + (q2*q)/2 + (q1*r)/2        

        # Rate of change of linear velocity
        du =   -w*q  + v*r  + g0*(2*q1*q3 - 2*q2*q4) 
        dv =    w*p  - u*r  - g0*(2*q1*q2 + 2*q3*q4)
        dw =   -v*p  + u*q  - g0*(2*q1**2 + 2*q4**2 - 1) + (Ct*(w1**2 + w2**2 + w3**2 + w4**2))/mq
        
        # Rate of change of angular velocity
        dp = -(Ct*l*(w1**2 + w2**2 - w3**2 - w4**2) - Iyy*q*r + Izz*q*r)/Ixx
        dq = -(Ct*l*(w1**2 - w2**2 - w3**2 + w4**2) + Ixx*p*r - Izz*p*r)/Iyy
        dr = -(Cd*  (w1**2 - w2**2 + w3**2 - w4**2) - Ixx*p*q + Iyy*p*q)/Izz
        
        f_op = vertcat(dx, dy, dz, dq1, dq2, dq3, dq4, du, dv, dw, dp, dq, dr)
        #fp = Function('f', [state, controls], [f_op])
        
        return f_op

    def acados_model_format():
        constraint = types.SimpleNamespace()
        model = types.SimpleNamespace()

        f_expl = model_ode()
        model_name = "CrazyFlie21_model"
        # algebraic variables
        z = vertcat([])

        # parameters
        p = vertcat([])

        # Define model structure
        params = types.SimpleNamespace()
        params.g0 = g0
        params.mq = mq
        params.Ixx = Ixx
        params.Iyy = Iyy
        params.Izz = Izz
        params.Cd = Cd
        params.Ct = Ct
        params.dq = dq
        params.l = l

        model.f_impl_expr = state_dt - f_expl
        model.f_expl_expr = f_expl
        model.x = state
        model.xdot = state_dt
        model.u = controls
        model.z = z
        model.p = p
        model.name = model_name
        model.params = params

        return model, constraint
    
class Predictor:

    def rk4_explicit(Dyn_fp, state, ctrl, Dt):
        k1 = Dyn_fp(state, ctrl)
        k2 = Dyn_fp(state + Dt * k1/2, ctrl)
        k3 = Dyn_fp(state + Dt * k2/2, ctrl)
        k4 = Dyn_fp(state + Dt * k3, ctrl)
        state_next = state + (Dt /6) * (k1 + 2 * k2 + 2 * k3 + k4)
        
        return state_next