from casadi import *
from common import *

'-------------------Symbolic variables---------------------------'
# State variables
x = SX.sym('x');    y = SX.sym('y');    z = SX.sym('z')                         # (in inertial frame)
q1 = SX.sym('q1');  q2 = SX.sym('q2');  q3 = SX.sym('q3');  q4 = SX.sym('q4')   # quarternion angle representation
u = SX.sym('u');    v = SX.sym('v');    w = SX.sym('w')                         # linear velocities (in body frame)
p = SX.sym('p');    q = SX.sym('q');    r = SX.sym('r')                # angular velocities w.r.t phi(roll), theta(pitch), psi(yaw)
w1 = SX.sym('w1');  w2 = SX.sym('w2');  w3 = SX.sym('w3');  w4 = SX.sym('w4')

# Time derivate of state (for implicit integrator)
x_dt = SX.sym('x_dt');    y_dt = SX.sym('y_dt');    z_dt = SX.sym('z_dt')                               # (in inertial frame)
q1_dt = SX.sym('q1_dt');  q2_dt = SX.sym('q2_dt');  q3_dt = SX.sym('q3_dt');  q4_dt = SX.sym('q4_dt')   # quarternion angle representation
u_dt = SX.sym('u_dt');    v_dt = SX.sym('v_dt');    w_dt = SX.sym('w_dt')                               # linear velocities (in body frame)
p_dt = SX.sym('p_dt');    q_dt = SX.sym('q_dt');    r_dt = SX.sym('r_dt')                               # angular velocities w.r.t phi(roll), theta(pitch), psi(yaw)
w1_dt = SX.sym('w1_dt');  w2_dt = SX.sym('w2_dt');  w3_dt = SX.sym('w3_dt');  w4_dt = SX.sym('w4_dt')

state = vertcat(x, y, z, q1, q2, q3, q4, u, v, w, p, q, r, w1, w2, w3, w4)
state_dt = vertcat(x_dt, y_dt, z_dt, q1_dt, q2_dt, q3_dt, q4_dt, u_dt, v_dt, w_dt, p_dt, q_dt, r_dt, w1_dt, w2_dt, w3_dt, w4_dt)

# Control variable angles (rate of Motor RPMs)
w1Der = SX.sym('w1Der');  w2Der = SX.sym('w2Der');  w3Der = SX.sym('w3Der');  w4Der = SX.sym('w4Der')
controls = vertcat( w1Der, w2Der, w3Der, w4Der) 

stSize = state.numel()
ctrlSize = controls.numel()

# matrices containing all States, Controls, Paramters over all time steps +1
St = SX.sym('St', stSize, N + 1)
U  = SX.sym('U', ctrlSize, N)
#P  = SX.sym('P', stSize + stSize)

class SysDyn():      

    def model_ode():
        '''ODEs modelling system dynamics '''

        # Rate of change of position
        dx = u*(2*(q1**2 + q2**2)- 1)  + v*2*(q2*q3 - q1*q4)           + w*2*(q1*q3 + q2*q4)
        dy = u*2*(q1*q4 + q2*q3)       + v*(2*(q1**2 + q3**2) - 1)     + w*2*(q3*q4 - q1*q2)
        dz = u*2*(q2*q4 - q1*q3)       + v*2*(q1*q2 + q3*q4)           + w*(2*(q1**2 + q4**2) - 1) 
        
        # Rate of change of heading (in quarternion)
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

        # Motor RPMs extends state
        dw1 = w1Der
        dw2 = w2Der
        dw3 = w3Der
        dw4 = w4Der

        f_expl = vertcat(dx, dy, dz, dq1, dq2, dq3, dq4, du, dv, dw, dp, dq, dr, dw1, dw2, dw3, dw4)
        f_impl = state_dt - f_expl
        
        return f_expl, f_impl, state, state_dt, controls
    
class Predictor:

    def rk4_explicit(Dyn_fp, state, ctrl, Dt):
        k1 = Dyn_fp(state, ctrl)
        k2 = Dyn_fp(state + Dt * k1/2, ctrl)
        k3 = Dyn_fp(state + Dt * k2/2, ctrl)
        k4 = Dyn_fp(state + Dt * k3, ctrl)
        state_next = state + (Dt /6) * (k1 + 2 * k2 + 2 * k3 + k4)
        
        return state_next