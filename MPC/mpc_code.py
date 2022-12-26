from time import time
import numpy as np
from casadi import *
import matplotlib.pyplot as plt
from simulation_code import simulate, plot_dataset

'''----------------Defining Setup parammeters-----------------'''

step_horizon = 0.1              # time between steps in seconds
N = 10                           # number of look ahead steps
sim_time = 20                   # simulation time

v_max = 0.6   ;   v_min = -0.6
w_max = pi/4  ;   w_min = -pi/4

init_st = np.array([0, 0, 0])
targ_st = np.array([4, 4, 0])
rob_rad = 0.3                  # radius of the robot

obst_st = np.array([1.7, 2.3, 0])
obst_rad = 0.3

'''-----------------------------------------------------------'''

def shift_timestep(step_horizon, t0, state_init, u, f):
    f_value = f(state_init, u[:, 0])
    next_state = DM.full(state_init + (step_horizon * f_value))

    t0 = t0 + step_horizon
    u0 = horzcat( u[:, 1:], reshape(u[:, -1], -1, 1))

    return t0, next_state, u0

def DM2Arr(dm):
    return np.array(dm.full())

def rk4_integrator(state, ctrl):
    k1 = f(state, ctrl)
    k2 = f(state + step_horizon/2*k1, ctrl)
    k3 = f(state + step_horizon/2*k2, ctrl)
    k4 = f(state + step_horizon * k3, ctrl)
    st_next_RK4 = state + (step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
    return st_next_RK4

# state symbolic variables
x = SX.sym('x');    y = SX.sym('y');   theta = SX.sym('theta')
states = vertcat( x, y, theta)
n_state = states.numel()

# control symbolic variables
v = SX.sym('v');    w = SX.sym('w')
controls = vertcat( v, w)
n_control = controls.numel()

# matrices containing all States, Controls, Paramters over all time steps +1
X = SX.sym('X', n_state, N + 1)
U = SX.sym('U', n_control, N)
P = SX.sym('P', n_state + n_state)

# State, Control weight matrices
Q = diagcat(1, 5, 0.1); R = diagcat(0.5, 0.05)

# Transfer function : input (Ctrl)- output(State) mapping
fnc_op = vertcat(v*cos(theta), v*sin(theta), w)
f = Function('f', [states, controls], [fnc_op])

cost_fn = 0                 # cost function
g = X[:, 0] - P[:n_state]  # constraints in the equation


# obtain optimized and estimated state from MS, RK
for k in range(N):
    st = X[:, k]
    U_k = U[:, k]
    cost_fn = cost_fn + (st - P[n_state:]).T @ Q @ (st - P[n_state:]) + U_k.T @ R @ U_k
    st_opt = X[:, k+1]
    st_est = rk4_integrator(st, U_k)
    g = vertcat(g, st_opt - st_est)                 # add equality constraints on predicted state (MS)

for k in range(N +1): 
    robX = X[0,k] ; robY = X[1,k]                   # inequality constrains on distance to obstace (obstace avoidance)
    g = vertcat(g , (-sqrt( power(robX - obst_st[0], 2) + power(robY - obst_st[1], 2)) + rob_rad + obst_rad))

OPT_variables = vertcat( X.reshape((-1, 1)),  U.reshape((-1, 1)) )
nlp_prob = { 'f': cost_fn, 'x': OPT_variables, 'g': g, 'p': P }

opts = {'ipopt'     : { 'max_iter': 100, 'print_level': 0, 'acceptable_tol': 1e-8, 'acceptable_obj_change_tol': 1e-6},
        'print_time': 0 }

solver = nlpsol('solver', 'ipopt', nlp_prob, opts)

st_size = n_state * (N+1)
U_size = n_control * N

'''------------------Defining constraints---------------------'''


# Bounds on State, Controls 
lbx = DM.zeros((st_size + U_size, 1))
ubx = DM.zeros((st_size + U_size, 1))
# Bounds on obstace avoidance
lbg = DM.zeros((st_size + (N+1) , 1))
ubg = DM.zeros((st_size + (N+1), 1))

lbx[0: st_size: n_state] = -8          # X lower bound
lbx[1: st_size: n_state] = -8          # Y lower bound
lbx[2: st_size: n_state] = -inf        # theta lower bound
lbx[st_size : : n_control] = v_min     # v lower bound
lbx[st_size + 1: : n_control] = w_min  # w lower bound

ubx[0: st_size: n_state] = 8           # X upper bound
ubx[1: st_size: n_state] = 8           # Y upper bound
ubx[2: st_size: n_state] = inf         # theta upper bound
ubx[st_size : : n_control] = v_max     # v upper bound
ubx[st_size +1 : : n_control] = w_max  # w upper bound

lbg[0: st_size] = 0                     # pred_st - optim_st = 0                  
lbg[st_size: st_size+ (N+1)] = -inf     # -inf < Euclidian - sum(radii) 

ubg[0: st_size] = 0
ubg[st_size: st_size+ (N+1)] = 0        # Euclidian - sum(radii) < 0

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

u0 = DM.zeros((n_control, N))          # initial control
X0 = repmat(state_init, 1, N+1)         # initial state full


mpc_iter = 0
cat_states = DM2Arr(X0)
cat_controls = DM2Arr(u0[:, 0])
times = np.array([[0]])


'''--------------------Simulation-----------------------------'''

if __name__ == '__main__':
    main_loop = time()  # return time in sec
    
    #time_stamp = []
    while (norm_2(state_init - state_target) > 1e-1) and (mpc_iter * step_horizon < sim_time):
        t1 = time()
        args['p'] = vertcat( state_init,  state_target)
        # optimization variable current state
        args['x0'] = vertcat(   reshape(X0, n_state*(N+1), 1),
                                reshape(u0, n_control*N, 1))

        sol = solver(   x0=args['x0'], lbx=args['lbx'], ubx=args['ubx'], lbg=args['lbg'], ubg=args['ubg'], p=args['p'])

        X0 = reshape(sol['x'][ : n_state * (N+1)], n_state, N+1)
        u =  reshape(sol['x'][n_state * (N+1) : ], n_control, N)

        cat_states = np.dstack(( cat_states, DM2Arr(X0)))                  # State history
        cat_controls = np.vstack(( cat_controls, DM2Arr(u[:, 0])))         # init. control history
        t_step = np.append(t_step, t0)
        t0, state_init, u0 = shift_timestep(step_horizon, t0, state_init, u, f)
        # data = vertcat(data, [t0, state_init, u0])
        # print(f'Time : {t0}, State {state_init}, Control {u0}')
        X0 = horzcat(X0[:, 1:], reshape(X0[:, -1], -1, 1))

        t2 = time()
        # print(mpc_iter)
        # print(t2-t1)
        times = np.vstack(( times, t2-t1))

        #time_stamp.append(t0)
        mpc_iter = mpc_iter + 1

    main_loop_time = time()
    ss_error = norm_2(state_init - state_target)

    print('\n\n')
    print('Total time: ', main_loop_time - main_loop)
    print('avg iteration time: ', np.array(times).mean() * 1000, 'ms')
    print('final error: ', ss_error)

    # simulate
    plot_dataset(cat_controls, timestamp = t_step)
    simulate(cat_states, cat_controls, times, step_horizon, N, 
             np.append(init_st, targ_st), rob_rad,
             obst_st, obst_rad,             
             save=False)
