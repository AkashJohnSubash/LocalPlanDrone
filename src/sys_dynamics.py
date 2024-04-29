import casadi as ca
from common import *

'''Global Symbolic variables'''
# State variables

# Position (inertial frame, meter)
x = ca.SX.sym('x')
y = ca.SX.sym('y')
z = ca.SX.sym('z')

# Quarternion heading (body frame )
q1 = ca.SX.sym('q1')
q2 = ca.SX.sym('q2')
q3 = ca.SX.sym('q3')
q4 = ca.SX.sym('q4')

# Transaltional velocities (inertial frame, m/s)
vx = ca.SX.sym('vx')
vy = ca.SX.sym('vy')
vz = ca.SX.sym('vz')

# Angular velocities w.r.t phi(roll), theta(pitch), psi(yaw)
# (body frame, m/s)
wr = ca.SX.sym('wr')
wp = ca.SX.sym('wp')
wy = ca.SX.sym('wy')

zeta = ca.vertcat(x, y, z, q1, q2, q3, q4, vx, vy, vz, wr, wp, wy)

# Control variable angles (Motor RPM)
ua = ca.SX.sym('ua')
ub = ca.SX.sym('ub')
uc = ca.SX.sym('uc')
ud = ca.SX.sym('ud')
u = ca.vertcat( ua, ub, uc, ud)

class SysDyn():

    def __init__(self):

        self.stSize = zeta.numel()
        self.ctrlSize = u.numel()

        # matrices containing all States, Controls, Paramters over all time steps +1
        self.St = ca.SX.sym('St', self.stSize, hznLen + 1)
        self.U = ca.SX.sym('U', self.ctrlSize, hznLen)
        self.P = ca.SX.sym('P', self.stSize + self.stSize)


    def ForwardDynamics(self):
        '''ODEs for system dynamic model'''

        # Rate of change of position
        # dx = 2*(vx * ((q1**2 + q2**2)- 0.5)  + vy *(q2*q3 - q1*q4)         + vz *(q1*q3 + q2*q4))
        # dy = 2*(vx * (q1*q4 + q2*q3)       + vy *(q1**2 + q3**2 - 0.5)     + vz *(q3*q4 - q1*q2))
        # dz = 2*(vx * (q2*q4 - q1*q3)       + vy *(q1*q2 + q3*q4)           + vz *((q1**2 + q4**2) - 0.5))

        dx = vx
        dy = vy
        dz = vz

        # Rate of change of angles (in qaurternion)
        dq1 = (-(q2 * wr) - (q3 * wp) - (q4 * wy))/2
        dq2 = ( (q1 * wr) - (q4 * wp) + (q3 * wy))/2
        dq3 = ( (q4 * wr) + (q1 * wp) - (q2 * wy))/2
        dq4 = (-(q3 * wr) + (q2 * wp) + (q1 * wy))/2

        #  Rate of change of transalational velocity
        # du =   -vz * wp  + vy * wy  + 2 * g0 * (q1 * q3 + q2 * q4)
        # dv =    vz * wr  - vx * wy  - 2 * g0 * (q1 * q2 + q3*  q4)
        # dw =   -vy * wr  + vx * wp  - 2 * g0 * (q1**2 + q4**2 - 0.5) + (Ct*( ua**2 + ub**2 + uc**2 + ud**2))/mq

        # Rate of change of transalational velocity
        du =    2 * (Ct/mq) *(q1 * q3 + q2 * q4) *(ua**2  + ub**2  + uc**2  + ud**2 ) - (Cd/mq) * vx
        dv =   -2 * (Ct/mq) *(q1 * q2 - q3 * q4) *(ua**2  + ub**2  + uc**2  + ud**2 ) - (Cd/mq) * vy
        dw =   -g0 + 2 * Ct/mq *(q1**2 + q4**2 -0.5) * (ua**2  + ub**2  + uc**2  + ud**2 ) - (Cd/mq) * vz

        # Rate of change of angular velocity
        dp = -(Ct*l*(ua**2 + ub**2 - uc**2 - ud**2) - Iy * wp * wy + Iz * wp * wy) / Ix
        dq = -(Ct*l*(ua**2 - ub**2 - uc**2 + ud**2) + Ix * wr * wy - Iz * wr * wy) / Iy
        dr = -(Cd*  (ua**2 - ub**2 + uc**2 - ud**2) - Ix * wr * wp + Iy * wr * wp) / Iz

        f_op = ca.vertcat(dx, dy, dz, dq1, dq2, dq3, dq4, du, dv, dw, dp, dq, dr)
        fp = ca.Function('f', [zeta, u], [f_op])

        return fp

class Predictor:

    def rk4_explicit(Dyn_fp, state, ctrl, Dt):
        k1 = Dyn_fp(state, ctrl)
        k2 = Dyn_fp(state + Dt * k1/2, ctrl)
        k3 = Dyn_fp(state + Dt * k2/2, ctrl)
        k4 = Dyn_fp(state + Dt * k3, ctrl)
        st_next_RK4 = state + (Dt /6) * (k1 + 2 * k2 + 2 * k3 + k4)

        return st_next_RK4