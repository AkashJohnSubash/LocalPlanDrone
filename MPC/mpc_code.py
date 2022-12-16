from time import time
import numpy as np
from casadi import *
import matplotlib.pyplot as plt
from simulation_code import simulate, plot_dataset

# setting matrix_weights' variables

step_horizon = 0.2  # time between steps in seconds
N = 10               # number of look ahead steps
rob_diam = 0.3      # diameter of the robot
sim_time = 20       # simulation time

v_max = 0.6   ;   v_min = -0.6
w_max = pi/4  ;   w_min = -pi/4

init_st= np.array([0, 0, 0])
targ_st = np.array([2, 2, pi])

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
n_states = states.numel()

# control symbolic variables
v = SX.sym('v');    w = SX.sym('w')
controls = vertcat( v, w)
n_controls = controls.numel()

# matrices containing all States, Controls, Paramters over all time steps +1
X = SX.sym('X', n_states, N + 1)
U = SX.sym('U', n_controls, N)
P = SX.sym('P', n_states + n_states)

# state weights matrix (Q_X, Q_Y, Q_THETA)
Q = diagcat(1, 5, 0.1)

# controls weights matrix
R = diagcat(0.5, 0.05)

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

# Transfer function : input (Ctrl)- output(State) mapping
fnc_op = vertcat(v*cos(theta), v*sin(theta), w)
f = Function('f', [states, controls], [fnc_op])

cost_fn = 0                 # cost function
g = X[:, 0] - P[:n_states]  # constraints in the equation


# runge kutta
for k in range(N):
    st = X[:, k]
    U_k = U[:, k]
    cost_fn = cost_fn + (st - P[n_states:]).T @ Q @ (st - P[n_states:]) + U_k.T @ R @ U_k
    st_next = X[:, k+1]
    st_next_RK4 = rk4_integrator(st, U_k)
    g = vertcat(g, st_next - st_next_RK4)         # Multiple shooting constraints on State


OPT_variables = vertcat( X.reshape((-1, 1)),  U.reshape((-1, 1)) )
nlp_prob = { 'f': cost_fn, 'x': OPT_variables, 'g': g, 'p': P }

opts = {'ipopt'     : { 'max_iter': 100, 'print_level': 0, 'acceptable_tol': 1e-8, 'acceptable_obj_change_tol': 1e-6},
        'print_time': 0 }

solver = nlpsol('solver', 'ipopt', nlp_prob, opts)

lbx = DM.zeros((n_states*(N+1) + n_controls*N, 1))
ubx = DM.zeros((n_states*(N+1) + n_controls*N, 1))
lbg = DM.zeros((n_states*(N+1), 1))
ubg = DM.zeros((n_states*(N+1), 1))
#TODO play around here
lbx[0: n_states*(N+1): n_states] = -8       # X lower bound
lbx[1: n_states*(N+1): n_states] = -8       # Y lower bound
lbx[2: n_states*(N+1): n_states] = -inf     # theta lower bound
lbx[n_states*(N+1)  :: n_controls] = v_min  # v lower bound for all V
lbx[n_states*(N+1)+1:: n_controls] = w_min  # v lower bound for all V

ubx[0: n_states*(N+1): n_states] = 8        # X upper bound
ubx[1: n_states*(N+1): n_states] = 8        # Y upper bound
ubx[2: n_states*(N+1): n_states] = inf      # theta upper bound
ubx[n_states*(N+1)  :: n_controls] = v_max  # v upper bound for all V
ubx[n_states*(N+1)+1:: n_controls] = w_max  # v upper bound for all V

args = { 'lbg': lbg,  # constraints lower bound
         'ubg': ubg,  # constraints upper bound
         'lbx': lbx,
         'ubx': ubx}

t0 = 0
state_init = DM(init_st)           # initial state
state_target = DM(targ_st)   # target state

# xx = DM(state_init)
t_step = np.array([])

u0 = DM.zeros((n_controls, N))  # initial control
X0 = repmat(state_init, 1, N+1)         # initial state full


mpc_iter = 0
cat_states = DM2Arr(X0)
cat_controls = DM2Arr(u0[:, 0])
times = np.array([[0]])

# Simulation
###############################################################################

if __name__ == '__main__':
    main_loop = time()  # return time in sec
    
    #time_stamp = []
    while (norm_2(state_init - state_target) > 1e-1) and (mpc_iter * step_horizon < sim_time):
        t1 = time()
        args['p'] = vertcat( state_init,  state_target)
        # optimization variable current state
        args['x0'] = vertcat(   reshape(X0, n_states*(N+1), 1),
                                reshape(u0, n_controls*N, 1))

        sol = solver(   x0=args['x0'], lbx=args['lbx'], ubx=args['ubx'], lbg=args['lbg'], ubg=args['ubg'], p=args['p'])

        u = reshape(sol['x'][n_states * (N + 1):], n_controls, N)
        X0 = reshape(sol['x'][: n_states * (N+1)], n_states, N+1)

        cat_states = np.dstack(( cat_states, DM2Arr(X0)))
        cat_controls = np.vstack(( cat_controls, DM2Arr(u[:, 0])))
        t_step = np.append(t_step, t0)
        t0, state_init, u0 = shift_timestep(step_horizon, t0, state_init, u, f)
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
    ss_error = norm_2(state_init - state_target)

    print('\n\n')
    print('Total time: ', main_loop_time - main_loop)
    print('avg iteration time: ', np.array(times).mean() * 1000, 'ms')
    print('final error: ', ss_error)

    # simulate
    plot_dataset( ctrl_data = DM2Arr(u), state_data = state_init, timestamp = t_step)
    simulate(cat_states, cat_controls, times, step_horizon, N, np.append(init_st, targ_st), rob_diam, save=False)
