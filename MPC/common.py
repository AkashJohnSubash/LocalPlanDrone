import numpy as np
from casadi import *

'''----------------Defining Setup parammeters-----------------'''

hznStep = 0.1                   # time between steps in seconds
hznLen = 5                      # number of look ahead steps
sim_time = 20                   # simulation time

v_max = 0.6   ;   v_min = -0.6
w_max = pi/4  ;   w_min = -pi/4
max_thrust = 22

# State
n_states = 13
                   #x,  y,  z, qw, qx, qy, qz,  u,  v,  w,  p,  q,  r
#init_st = np.array([0,  0,  0,  0,  0,  0])                            # 2D DEBUG
init_st = np.array([0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0])            
#targ_st = np.array([4,  4,  4,  0,  0,  0])                            # 2D DEBUG
targ_st = np.array([4,  4,  4, 0, 0, 0,  0,  0,  0,  0,  0,  0,  0])
rob_rad = 0.3                               # radius of the robot sphere

#obst_st = np.array([1.7,  3,   3,  0,   0,  0])
obst_st = np.array([8, 8, 8,  0,   0,  0])  # 2D DEBUG
obst_rad = 0.3                              # radius of the obstacle sphere

# Control
n_controls = 4

'''-----------------------------------------------------------'''

def DM2Arr(dm):
    return np.array(dm.full())