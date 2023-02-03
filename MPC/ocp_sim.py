from time import time
import numpy as np
from casadi import *
from common import *
from SysDynamics import SysDyn as Sys, Predictor as Pred
from measurement import state_meas

def traj_commander():
    # generates optimal trajectory using an NMPC cost function
    SysObj = Sys()
    ST = SysObj.St;   U = SysObj.U;   P = SysObj.P

    Dyn_fp = SysObj.ForwardDynamics()

    # State, Control weight matrices
    # TODO investigate effect of weights
    Q = diagcat(120, 100, 100, 1e-3, 1e-3, 1e-3, 1e-3, 0.7, 1, 4, 1e-5, 1e-5, 10) 
    R = diagcat(0.5, 0.5, 0.5, 0.5)

    cost_fn = 0                  # cost function
    g = ST[:, 0] - P[:n_states]  # constraints in the equation
    
    # try:
    '''-----------Formulate OCP as inequality constrained NLP------------'''

    # Obtain optimized and estimated state from MS, RK (Symbolic)
    for k in range(hznLen):
        st = ST[:, k]
        U_k = U[:, k]
        cost_fn = cost_fn + (st - P[n_states:]).T @ Q @ (st - P[n_states:]) + U_k.T @ R @ U_k
        st_opt = ST[:, k+1]
        st_est = Pred.rk4_integrator(Dyn_fp, st, U_k, hznStep)           # generates discrete system dynamics model from TF
        g = vertcat(g, st_opt - st_est)                                  # predicted state (MS) constraint

    # Path constraint equations (obstacle avoidance)
    for k in range(hznLen +1): 
        g = vertcat(g , (-sqrt( ((ST[0,k] - obst_st[0]) ** 2)  +
                                ((ST[1,k] - obst_st[1]) ** 2)  + 
                                ((ST[2,k] - obst_st[2]) ** 2)) + rob_rad + obst_rad))                           # obstacle with same radius as bot
    
    # Thrust smoothening constraint
    for k in range(hznLen-1):
        avg_rpm_next =  (U[0,k+1] + U[1,k+1] + U[2,k+1] + U[3,k+1])/4
        avg_rpm  = (U[0,k] + U[1,k] + U[2,k] + U[3,k])/4
        g = vertcat(g , ( avg_rpm_next - avg_rpm))                                      # difference of thrust average (delta_thrust)

    OPT_variables = vertcat( ST.reshape((-1, 1)),  U.reshape((-1, 1)) )
    nlp_prob = { 'f': cost_fn, 'x': OPT_variables, 'g': g, 'p': P }
    
    '''-----------------------Configure solver-----------------------------'''
    
    opts = {'ipopt'     : { 'max_iter': 1000, 'print_level': 0, 'acceptable_tol': 1e-8, 'acceptable_obj_change_tol': 1e-6},
            'print_time': 0 }

    solver = nlpsol('solver', 'ipopt', nlp_prob, opts)

    st_size = n_states * (hznLen+1)
    U_size = n_controls * hznLen

    '''-----------------------Define NLP bounds-----------------------------'''


    # Bounds on decision variables
    lbx = DM.zeros((st_size + U_size, 1))
    ubx = DM.zeros((st_size + U_size, 1))
    # State bounds
    lbx[0:  st_size: n_states] = 0;             ubx[0: st_size: n_states] = 2.5                 # x lower, upper bounds
    lbx[1:  st_size: n_states] = 0;             ubx[1: st_size: n_states] = 2.5                 # y bounds
    lbx[2:  st_size: n_states] = 0;             ubx[2: st_size: n_states] = 2.5                 # z bounds
    lbx[3:  st_size: n_states] = -1;            ubx[3:  st_size: n_states] = 1                  # qw bounds TODO find appropriate val
    lbx[4:  st_size: n_states] = -1;            ubx[4:  st_size: n_states] = 1                  # qx bounds
    lbx[5:  st_size: n_states] = -1;            ubx[5:  st_size: n_states] = 1                  # qy bounds
    lbx[6:  st_size: n_states] = -1;            ubx[6:  st_size: n_states] = 1                  # qz bounds
    lbx[7:  st_size: n_states] = v_min;         ubx[7:  st_size: n_states] = v_max              # u bounds
    lbx[8:  st_size: n_states] = v_min;         ubx[8:  st_size: n_states] = v_max              # v bounds
    lbx[9:  st_size: n_states] = v_min;         ubx[9:  st_size: n_states] = v_max              # w bounds
    lbx[10: st_size: n_states] = w_min;         ubx[10: st_size: n_states] = w_max              # p bounds TODO find appropriate val
    lbx[11: st_size: n_states] = w_min;         ubx[11: st_size: n_states] = w_max              # q bounds
    lbx[12: st_size: n_states] = w_min;         ubx[12: st_size: n_states] = w_max              # r bounds
    # Control bounds
    lbx[st_size    : : n_controls] = 0;         ubx[st_size     : : n_controls] = max_krpm        # w1 bounds
    lbx[st_size +1 : : n_controls] = 0;         ubx[st_size+1   : : n_controls] = max_krpm        # w2 bounds
    lbx[st_size +2 : : n_controls] = 0;         ubx[st_size+2   : : n_controls] = max_krpm        # w3 bounds
    lbx[st_size +3 : : n_controls] = 0;         ubx[st_size+3   : : n_controls] = max_krpm        # w4 bounds

    # Bounds on constraints
                   #MS,        Path,       Smoothen
    lbg = DM.zeros((st_size + (hznLen+1) + (hznLen-1), 1))
    ubg = DM.zeros((st_size + (hznLen+1) + (hznLen-1), 1))

    # MS constr: pred_st - optim_st = 0
    lbg[0 : st_size] = 0;                         ubg[0        : st_size] = 0

    # Path constr: -inf < Euclidian - sum(radii) < 0
    lbg[st_size : st_size + (hznLen+1)]   = -inf; ubg[st_size  : st_size+ (hznLen+1)] = 0                
    
    # Smoothening constraints Delta_rpm_avg < Delta_rpm_max
    lbg[st_size + (hznLen+1) : st_size + (hznLen+1) + (hznLen-1)] = -del_rpm_max;   
    ubg[st_size + (hznLen+1) : st_size + (hznLen+1) + (hznLen-1)] = del_rpm_max

    args = {    'lbg': lbg,                    # constraints lower bound
                'ubg': ubg,                    # constraints upper bound
                'lbx': lbx,
                'ubx': ubx}

    '''---------------------------------------------------------------'''

    t0 = 0
    state_init = DM(init_st)                # initial state
    state_target = DM(targ_st)              # target state

    t_step = np.array([])

    u0 = DM(np.full((n_controls, hznLen), hover_krpm))      # initial control
    X0 = repmat(state_init, 1, hznLen+1)     # initial state full

    mpc_iter = 0
    cat_states = DM2Arr(X0)
    cat_controls = DM2Arr(u0[:, 0])
    times = np.array([[0]])


    '''--------------------Execute MPC -----------------------------'''

    # STAGE 2 perform overtake
    setpoints = [0, 0, 0, int(0)]
    roll, pitch, yawRate, thrust_norm = calc_thrust_setpoint(state_init, u0[:, 0])
    print(f"Hover normaized RPM  {roll, pitch, yawRate, thrust_norm}")
    main_loop = time()                                                      # return time in sec
    while (norm_2(state_init - state_target) > 1e-1) and (mpc_iter * hznStep < sim_time):
        t1 = time()                                                         # start iter timer                                                                                                    
        args['p'] = vertcat( state_init,  state_target)
        # print(f"DEBUG Forward sim state \t {args['p']}" )
        # optimization variable current state
        args['x0'] = vertcat(   reshape(X0, n_states*(hznLen+1), 1),
                                reshape(u0, n_controls*hznLen, 1))

        sol = solver( x0=args['x0'], lbx=args['lbx'], ubx=args['ubx'], lbg=args['lbg'], ubg=args['ubg'], p=args['p'])

        X0 = reshape(sol['x'][ : n_states * (hznLen+ 1)], n_states, hznLen+1)
        u =  reshape(sol['x'][n_states * (hznLen+ 1): ], n_controls, hznLen)
        cat_states = np.dstack(( cat_states, DM2Arr(X0)))
        cat_controls = np.vstack(( cat_controls, DM2Arr(u[:, 0])))
        t_step = np.append(t_step, t0)
        t0, state_init, u0 = Sys.TimeStep(hznStep, t0, state_init, u, Dyn_fp)
        #print(f"MPC State {X0[:, 0]} at {np.round(t0,3)}s ")#, {u[:, 0]}")
        X0 = horzcat( X0[:, 1:], reshape(X0[:, -1], -1, 1))

        t2 = time()                                                     # stop iter timer
        times = np.vstack(( times, t2-t1))
        mpc_iter = mpc_iter + 1
        print(f'Soln Timestep : {round(t0,3)} s\r', end="")             # State {X0[:, 0]}')
        #print(f"DEBUG{X0[:, 0]}")
        roll, pitch, yawRate, thrust_norm = calc_thrust_setpoint(X0, u)
        setpoints = np.vstack( (setpoints, [roll, pitch, yawRate, thrust_norm]))
        print(f'set points : {roll}, {pitch}, {yawRate}, {thrust_norm}')
    '''---------------------Execute trajectory with CF setpoint tracking--------------------------'''
    main_loop_time = time()
    ss_error = norm_2(state_init - state_target)

    print('\n\n')
    print('Total time: ', main_loop_time - main_loop)
    print('avg iteration time: ', np.array(times).mean() * 1000, 'ms')
    print('final error: ', ss_error)
    
    return cat_controls, t_step, cat_states, times
    # except Exception as ex:
    #     print("MPC aborted due to Exception:", ex)