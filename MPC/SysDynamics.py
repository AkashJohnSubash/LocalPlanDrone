from casadi import *
from MPC.common import *

#TODO evaluate the advantage of quarternion
# def eul2quart():
#     # Convert euler angles to quarternion
#     qw = cos(phi) * cos(th) * cos(psi) + sin(phi) * sin(th) * sin(psi)
#     qx = -cos(phi) * cos(th) * cos(phi) + sin(psi) * sin(th) * cos(phi)
#     qy = -cos(psi) * sin(th) * cos(phi) - sin(psi) * cos(th) * sin(phi)
#     qz = -sin(psi) * cos(th) * cos(phi) - cos(psi) * sin(th) * sin(phi)
#     quart = vertcat(qw, qx, qy, qz)
#     fp = Function('e2q', [ang], [quart])
#     return fp

'-------------------Symbolic variables---------------------------'
# State symbols
xq = SX.sym('xq');  yq = SX.sym('yq');  zq = SX.sym('zq')                       # (in inertial frame)
q1 = SX.sym('q1');  q2 = SX.sym('q2');  q3 = SX.sym('q3');  q4 = SX.sym('q4')   # quarternion angle representation
u = SX.sym('u');    v = SX.sym('v');    w = SX.sym('w')                         # linear velocities (in body frame)
p = SX.sym('p');    q = SX.sym('q');    r = SX.sym('r')                         # angular velocities w.r.t psi, theta, phi

state = vertcat(xq, yq, zq, q1, q2, q3, q4, u, v, w, p, q, r)

# Control symbols (Motor RPM)
w1 = SX.sym('w1');  w2 = SX.sym('w2');  w3 = SX.sym('w3');  w4 = SX.sym('w4')
controls = vertcat( w1, w2, w3, w4) 

class SysDyn():
    
    def __init__(self, N):
        
        self.stSize = state.numel()
        self.ctrlSize = controls.numel()

        # matrices containing all States, Controls, Paramters over all time steps +1
        self.St = SX.sym('St', self.stSize, N + 1)
        self.U = SX.sym('U', self.ctrlSize, N)
        self.P = SX.sym('P', self.stSize + self.stSize)
        
        self.hov_w = np.sqrt((mq*g0)/(4*Ct))

    def TransferFunction(self):
        # Transfer function : input (Ctrl)-> output(State) mapping
        # Rate of change of position
        dxq = u*(2*q1**2 + 2*q2**2 - 1) - v*(2*q1*q4 - 2*q2*q3) + w*(2*q1*q3 + 2*q2*q4)
        dyq = v*(2*q1**2 + 2*q3**2 - 1) + u*(2*q1*q4 + 2*q2*q3) - w*(2*q1*q2 - 2*q3*q4)
        dzq = w*(2*q1**2 + 2*q4**2 - 1) - u*(2*q1*q3 - 2*q2*q4) + v*(2*q1*q2 + 2*q3*q4)
        
        # Rate of change of angles (in qaurternion)
        dq1 = - (q2*p)/2 - (q3*q)/2 - (q4*r)/2
        dq2 =   (q1*p)/2 - (q4*q)/2 + (q3*r)/2
        dq3 =   (q4*p)/2 + (q1*q)/2 - (q2*r)/2
        dq4 =   (q2*q)/2 - (q3*p)/2 + (q1*r)/2        

        # Rate of change of linear velocity
        du = v*r - w*q + g0*(2*q1*q3 - 2*q2*q4)
        dv = w*p - u*r - g0*(2*q1*q2 + 2*q3*q4)
        dw = u*q - v*p - g0*(2*q1**2 + 2*q4**2 - 1) + (Ct*(w1**2 + w2**2 + w3**2 + w4**2))/mq
        
        # Rate of change of angular velocity
        dp = -(Ct*l*(w1**2 + w2**2 - w3**2 - w4**2) - Iyy*q*r + Izz*q*r)/Ixx
        dq = -(Ct*l*(w1**2 - w2**2 - w3**2 + w4**2) + Ixx*p*r - Izz*p*r)/Iyy
        dr = -(Cd*  (w1**2 - w2**2 + w3**2 - w4**2) - Ixx*p*q + Iyy*p*q)/Izz
        
        f_op = vertcat(dxq, dyq, dzq, dq1, dq2, dq3, dq4, du, dv, dw, dp, dq, dr)
        fp = Function('f', [state, controls], [f_op])
        return fp 

    def TimeStep(step_horizon, t0, state_init, u, f):
        f_value = f(state_init, u[:, 0])
        next_state = DM.full(state_init + (step_horizon * f_value)) # TODO Check if linearization is still valid ?
        #print(f'\nDEBUG3 {t0}, next state :\n{next_state}, \nControl :\n{u[:, 0]}' )

        t0 = t0 + step_horizon
        u0 = horzcat( u[:, 1:], reshape(u[:, -1], -1, 1))

        return t0, next_state, u0

    # # DEBUG ERROR
    # def eulRotTrans(self):
    #     Rx = vertcat([1,           0,          0],
    #                  [0,    cos(phi),   sin(phi)],
    #                  [0,   -sin(phi),   cos(phi)])

    #     Ry = vertcat([cos(th),      0,  -sin(th)],
    #                  [      0,      1,         0],
    #                  [sin(th),      0,   cos(th)])
        
    #     Rz = vertcat([cos(psi),     sin(psi),   0],
    #                  [-sin(psi),    cos(psi),   0],
    #                  [0,                   0,   1])
    #     f_op = Rx ** Ry ** Rz ** ang
    #     print(f_op)
    #     fp = Function('euRot', [ang], f_op)

class Predictor:

    def rk4_integrator(TransFunc, state, ctrl, step_horizon):
        k1 = TransFunc(state, ctrl)
        k2 = TransFunc(state + step_horizon/2*k1, ctrl)
        k3 = TransFunc(state + step_horizon/2*k2, ctrl)
        k4 = TransFunc(state + step_horizon * k3, ctrl)
        st_next_RK4 = state + (step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
        return st_next_RK4