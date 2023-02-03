import numpy as np
from casadi import *

'''----------------OCP parammeters-----------------'''

hznStep = 0.1                       # time between steps in seconds
hznLen = 10                         # number of look ahead steps
sim_time = 10                       # simulation time
milestones = 5

v_max = 0.2    ;   v_min = -0.2     #  [m/s]
w_max = pi/10  ;   w_min = -pi/10     #  [rad/s]
del_rpm_max = 0.5                   #  [Krpm]

# State
n_states = 13
                   #x,  y,  z, qw, qx, qy, qz,  u,  v,  w,  p,  q,  r
init_st = np.array([0,      0,  0.5,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0])            
targ_st = np.array([1,   1,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0])
rob_rad = 0.05                               # radius of the robot sphere

obst_st = np.array([2.3,  2.3,  2.3,  0,   0,  0, 0, 0, 0, 0, 0, 0])
obst_rad = .1

# Control
n_controls = 4

'------------------------CF parameters--------------------------------'

g0  = 9.8066     # [m.s^2] accerelation of gravity
mq  = 33e-3      # [kg] total mass (with one marker)
Ixx = 1.395e-5   # [kg.m^2] Inertia moment around x-axis
Iyy = 1.395e-5   # [kg.m^2] Inertia moment around y-axis
Izz = 2.173e-5   # [kg.m^2] Inertia moment around z-axis
Cd  = 7.9379e-06 # [N/krpm^2] Drag coef
Ct  = 3.25e-4    # [N/krpm^2] Thrust coef
dq  = 65e-3      # [m] distance between motors' center
l   = dq/2       # [m] distance between motors' center and the axis of rotation
max_krpm = 22    # [krpm]
hover_krpm = int(sqrt(.25 * 1e6* mq * g0 /Ct)) /1000 #[krpm]

ROLL_TRIM  = 0
PITCH_TRIM = 0
'''-----------------------------------------------------------'''

def DM2Arr(dm):
    return np.array(dm.full())

def quat2eul(qoid):
    ''' qoid -> [qw, qx, qy, qz]
        reference NMPC'''

    R11 = 2*(qoid[0]*qoid[0] + qoid[1]*qoid[1]) - 1
    R21 = 2*(qoid[1]*qoid[2] - qoid[0]*qoid[3])
    R31 = 2*(qoid[1]*qoid[3] + qoid[0]*qoid[2])
    R32 = 2*(qoid[2]*qoid[3] - qoid[0]*qoid[1])
    R33 = 2*(qoid[0]*qoid[0] + qoid[3]*qoid[3]) - 1

    # Euler angles in degrees
    phi	  =  atan2(R32, R33) * 180 / pi         # roll
    theta = -asin(R31)       * 180 / pi         # pitch
    psi	  =  atan2(R21, R11) * 180 / pi         # yaw

    return [phi, theta, psi]

def eul2quat(eul):
    ''' eul ->  [phi, theta, psi] in degrees
        a.k.a roll, pitch, yaw
        reference NMPC'''
    
    phi = 0.5* eul[0] * pi / 180
    th  = 0.5* eul[1] * pi / 180
    psi = 0.5* eul[2] * pi / 180

    qw =  cos(phi) * cos(th) * cos(psi) + sin(phi) * sin(th) * sin(psi)
    qx = -cos(psi) * cos(th) * cos(phi) + sin(psi) * sin(th) * cos(phi)
    qy = -cos(psi) * sin(th) * cos(phi) - sin(psi) * cos(th) * sin(phi)
    qz = -sin(psi) * cos(th) * cos(phi) + cos(psi) * sin(th) * sin(phi)

    if(qw < 0):
      qw = -qw
      qx = -qx
      qy = -qy
      qz = -qz  
    
    return [qw, qx, qy, qz]

def krpm2pwm( Krpm):
    '''Convert CF propellor angular speed into PWM values'''
    pwm = ((Krpm*1000)-4070.3)/0.2685

    return pwm

# def quatdecompress(comp, q_4):
# 	mask = int((1 << 9) - 1)
# 	i_largest = comp >> 30
# 	sum_squares = float(0)
# 	for i in range(3, -1):
# 		if (i != i_largest):
# 			mag = comp & mask
# 			negbit = (comp >> 9) & 0x1
# 			comp = comp >> 10
# 			q_4[i] = float(sqrt(0.5)) * float(mag) / mask
# 			if negbit == 1:
# 				q_4[i] = -q_4[i]
# 			sum_squares += q_4[i] * q_4[i]
# 	q_4[i_largest] = float(sqrt(1 - sum_squares))


def calc_thrust_setpoint(St_0, U_0):
    #TODO find documentation of this mapping, add to report 
    # euler in deg from q1,      q2,       q3,       q4
    eul_deg = quat2eul([St_0[3], St_0[4], St_0[5], St_0[6]])
    roll_y  = -eul_deg[0]                                           # Roll 
    pitch_x  = eul_deg[1]                                           # Pitch
    thrust_z  = krpm2pwm((U_0[0] + U_0[1]+ U_0[2]+ U_0[3])/4)       # convert average prop RPM to PWM                              
    roll_c   = roll_y + ROLL_TRIM                                   # TODO calibrate !
    pitch_c  = pitch_x + PITCH_TRIM                                 # corrected values
    thrust_c = int(min(max(thrust_z, 0.0), 60000))
    yawrate = St_0[12] * 180 /pi                                    # r in deg
    # print(f"\n DEBUG roll {roll}, pitch {pitch}, yawrate {yawrate}, thrust {thrust}")
    return roll_c, pitch_c, yawrate, thrust_c

