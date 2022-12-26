from time import time
import numpy as np
from casadi import *
from simulation_code import simulate, plot_dataset
from SysDynamics import SysDyn as Sys, Predictor as Pred
import common as tool

'''----------------Defining Setup parammeters-----------------'''

hznStep = 0.1                   # time between steps in seconds
hznLen = 5                     # number of look ahead steps
sim_time = 20                   # simulation time

v_max = 1   ;   v_min = -1
w_max = pi/4  ;   w_min = -pi/4
max_thrust = 22
                   #x,  y,  z, qw, qx, qy, qz,  u,  v,  w,  p,  q,  r
#init_st = np.array([0,  0,  0,  0,  0,  0])
init_st = np.array([0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0])            # 2D DEBUG
#targ_st = np.array([4,  4,  4,  0,  0,  0])
targ_st = np.array([4,  4,  4, 0, 0, 0,  0,  0,  0,  0,  0,  0,  0])            # 2D DEBUG
rob_rad = 0.3                  # radius of the robot sphere

#obst_st = np.array([1.7,  3,   3,  0,   0,  0])
obst_st = np.array([8, 8, 8,  0,   0,  0]) # 2D DEBUG
obst_rad = 0.3                 # radius of the obstacle sphere

'''-----------------------------------------------------------'''

SysObj = Sys(hznLen)
ST = SysObj.St;   U = SysObj.U;   P = SysObj.P
n_states = SysObj.stSize;  n_controls = SysObj.ctrlSize

TF_fp = SysObj.TransferFunction()

# State, Control weight matrices
# TODO investigate effect of weights
Q = diagcat(1, 1.5, 1, 1e-5, 1e-5, 1e-5, 1e-5, 0.5, 0.5, 0.5, 1e-5, 1e-5, 1e-5) 
R = diagcat(0.05, 0.05, 0.05, 0.05)

cost_fn = 0                  # cost function
g = ST[:, 0] - P[:n_states]  # constraints in the equation

# obtain optimized and estimated state from MS, RK
for k in range(hznLen):
    st = ST[:, k]
    U_k = U[:, k]
    cost_fn = cost_fn + (st - P[n_states:]).T @ Q @ (st - P[n_states:]) + U_k.T @ R @ U_k
    st_opt = ST[:, k+1]
    st_est = Pred.rk4_integrator(TF_fp, st, U_k, hznStep)           # generates discrete system dynamics model from TF
    g = vertcat(g, st_opt - st_est)                                 # add equality constraints on predicted state (MS)
    # print(f'DEBUG1 : iter {k}, optim state{st_opt}')

for k in range(hznLen +1): 
    # inequality path constraints (obstacle avoidance)
    g = vertcat(g , (-sqrt( power(ST[0,k] - obst_st[0], 2)  +
                            power(ST[1,k] - obst_st[1], 2)  + 
                            power(ST[2,k] - obst_st[2], 2)) + rob_rad + obst_rad))

OPT_variables = vertcat( ST.reshape((-1, 1)),  U.reshape((-1, 1)) )
nlp_prob = { 'f': cost_fn, 'x': OPT_variables, 'g': g, 'p': P }

opts = {'ipopt'     : { 'max_iter': 100, 'print_level': 0, 'acceptable_tol': 1e-8, 'acceptable_obj_change_tol': 1e-6},
        'print_time': 0 }

solver = nlpsol('solver', 'ipopt', nlp_prob, opts)

st_size = n_states * (hznLen+1)
U_size = n_controls * hznLen

'''------------------Defining constraints---------------------'''


# Bounds on State, Controls 
lbx = DM.zeros((st_size + U_size, 1))
ubx = DM.zeros((st_size + U_size, 1))
# Bounds on obstace avoidance
lbg = DM.zeros((st_size + (hznLen+1) , 1))
ubg = DM.zeros((st_size + (hznLen+1), 1))

lbx[0:  st_size: n_states] = -10;           ubx[0: st_size: n_states] = 10                  # x lower, upper bounds
lbx[1:  st_size: n_states] = -10;           ubx[1: st_size: n_states] = 10                  # y bounds
lbx[2:  st_size: n_states] = -10;           ubx[2: st_size: n_states] = 10                  # z bounds
lbx[3:  st_size: n_states] = -inf;          ubx[3:  st_size: n_states] = inf                # qw bounds TODO find appropriate val
lbx[4:  st_size: n_states] = -inf;          ubx[4:  st_size: n_states] = inf                # qx bounds
lbx[5:  st_size: n_states] = -inf;          ubx[5:  st_size: n_states] = inf                # qy bounds
lbx[6:  st_size: n_states] = -inf;          ubx[6:  st_size: n_states] = inf                # qz bounds
lbx[7:  st_size: n_states] = v_min;         ubx[7:  st_size: n_states] = v_max              # u bounds
lbx[8:  st_size: n_states] = v_min;         ubx[8:  st_size: n_states] = v_max              # v bounds
lbx[9:  st_size: n_states] = v_min;         ubx[9:  st_size: n_states] = v_max              # w bounds
lbx[10: st_size: n_states] = w_min;         ubx[10: st_size: n_states] = w_max              # p bounds
lbx[11: st_size: n_states] = w_min;         ubx[11: st_size: n_states] = w_max              # q bounds
lbx[12: st_size: n_states] = w_min;         ubx[12: st_size: n_states] = w_max              # r bounds

lbx[st_size    : : n_controls] = 0;         ubx[st_size : : n_controls] = max_thrust        # w1 bounds
lbx[st_size +1 : : n_controls] = 0;         ubx[st_size : : n_controls] = max_thrust        # w2 bounds
lbx[st_size +2 : : n_controls] = 0;         ubx[st_size : : n_controls] = max_thrust        # w3 bounds
lbx[st_size +3 : : n_controls] = 0;         ubx[st_size : : n_controls] = max_thrust        # w4 bounds

lbg[0: st_size] = 0;                        ubg[0: st_size] = 0                             # pred_st - optim_st = 0                  
lbg[st_size: st_size+ (hznLen+1)] = -inf;   ubg[st_size: st_size+ (hznLen+1)] = 0           # -inf < Euclidian - sum(radii) < 0
# print("\nSt, U lower bound \n", lbx)
# print("\nSt, U uppper bound \n", ubx)
# print("\nG(x) lower bound \n", lbg)
# print("\nG(x) uppper bound \n", ubg)

args = { 'lbg': lbg,                    # constraints lower bound
         'ubg': ubg,                    # constraints upper bound
         'lbx': lbx,
         'ubx': ubx}

'''-----------------------------------------------------------'''

t0 = 0
state_init = DM(init_st)                # initial state
state_target = DM(targ_st)              # target state

# xx = DM(state_init)
t_step = np.array([])

u0 = DM.zeros((n_controls, hznLen))      # initial control
X0 = repmat(state_init, 1, hznLen+1)     # initial state full


mpc_iter = 0
cat_states = tool.DM2Arr(X0)
cat_controls = tool.DM2Arr(u0[:, 0])
times = np.array([[0]])


'''--------------------Simulation-----------------------------'''

if __name__ == '__main__':
    main_loop = time()  # return time in sec
    
    #time_stamp = []
    while (norm_2(state_init - state_target) > 1e-1) and (mpc_iter * hznStep < sim_time):
        t1 = time()
        args['p'] = vertcat( state_init,  state_target)
        # optimization variable current state
        #print("DEBUG4", X0, u0)
        args['x0'] = vertcat(   reshape(X0, n_states*(hznLen+1), 1),
                                reshape(u0, n_controls*hznLen, 1))

        sol = solver(   x0=args['x0'], lbx=args['lbx'], ubx=args['ubx'], lbg=args['lbg'], ubg=args['ubg'], p=args['p'])

        X0 = reshape(sol['x'][ : n_states * (hznLen+ 1)], n_states, hznLen+1)
        u =  reshape(sol['x'][n_states * (hznLen+ 1): ], n_controls, hznLen)

        cat_states = np.dstack(( cat_states, tool.DM2Arr(X0)))
        cat_controls = np.vstack(( cat_controls, tool.DM2Arr(u[:, 0])))
        t_step = np.append(t_step, t0)
        t0, st0, u0 = Sys.TimeStep(hznStep, t0, state_init, u, TF_fp)
        # data = vertcat(data, [t0, state_init, u0])
        # print(f'Time : {t0}, State {state_init}, Control {u0}')
        X0 = horzcat(   X0[:, 1:],
                        reshape(X0[:, -1], -1, 1))

        t2 = time()
        # print(mpc_iter)
        # print(t2-t1)
        times = np.vstack(( times, t2-t1))

        #time_stamp.append(t0)
        mpc_iter = mpc_iter + 1

    main_loop_time = time()
    ss_error = norm_2(st0 - state_target)

    print('\n\n')
    print('Total time: ', main_loop_time - main_loop)
    print('avg iteration time: ', np.array(times).mean() * 1000, 'ms')
    print('final error: ', ss_error)

    # simulate
    plot_dataset( cat_controls, t_step)
    # simulate(cat_states, cat_controls, times, hznStep, hznLen, 
    #          np.append(init_st, targ_st), rob_rad,
    #          obst_st, obst_rad,             
    #          save=False)