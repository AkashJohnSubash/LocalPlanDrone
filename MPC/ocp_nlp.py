from time import time, sleep
import numpy as np
from casadi import *
from MPC.common import *
from MPC.simulation_code import simulate3D, plot_dataset
from MPC.SysDynamics import SysDyn as Sys, Predictor as Pred
from MPC.measurement import Meas_St

def traj_commander(scf = None):
    # generates optimal trajectory using an NMPC cost function
    SysObj = Sys(hznLen)
    ST = SysObj.St;   U = SysObj.U;   P = SysObj.P

    TF_fp = SysObj.TransferFunction()

    # State, Control weight matrices
    # TODO investigate effect of weights
    Q = diagcat(120, 100, 100, 1e-3, 1e-3, 1e-3, 1e-3, 0.7, 1, 4, 1e-5, 1e-5, 10) 
    R = diagcat(0.06, 0.06, 0.06, 0.06)

    cost_fn = 0                  # cost function
    g = ST[:, 0] - P[:n_states]  # constraints in the equation
    
    try:
        '''-----------Formulate OCP as inequality constrained NLP------------'''

        # Obtain optimized and estimated state from MS, RK (Symbolic)
        for k in range(hznLen):
            st = ST[:, k]
            U_k = U[:, k]
            cost_fn = cost_fn + (st - P[n_states:]).T @ Q @ (st - P[n_states:]) + U_k.T @ R @ U_k
            st_opt = ST[:, k+1]
            st_est = Pred.rk4_integrator(TF_fp, st, U_k, hznStep)           # generates discrete system dynamics model from TF
            g = vertcat(g, st_opt - st_est)                                 # predicted state (MS) constraint

        # Path constraint equations (obstacle avoidance)
        for k in range(hznLen +1): 
            g = vertcat(g , (-sqrt( ((ST[0,k] - obst_st[0]) ** 2)  +
                                    ((ST[1,k] - obst_st[1]) ** 2)  + 
                                    ((ST[2,k] - obst_st[2]) ** 2)) + rob_rad + rob_rad)) # obstacle with same radius as bot

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
        lbx[0:  st_size: n_states] = 0;           ubx[0: st_size: n_states] = 2.5                   # x lower, upper bounds
        lbx[1:  st_size: n_states] = 0;           ubx[1: st_size: n_states] = 2.5                   # y bounds
        lbx[2:  st_size: n_states] = 0;           ubx[2: st_size: n_states] = 2.5                   # z bounds
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
        lbx[st_size    : : n_controls] = 0;         ubx[st_size     : : n_controls] = max_rpm        # w1 bounds
        lbx[st_size +1 : : n_controls] = 0;         ubx[st_size+1   : : n_controls] = max_rpm        # w2 bounds
        lbx[st_size +2 : : n_controls] = 0;         ubx[st_size+2   : : n_controls] = max_rpm        # w3 bounds
        lbx[st_size +3 : : n_controls] = 0;         ubx[st_size+3   : : n_controls] = max_rpm        # w4 bounds

        # Bounds on constraints
        lbg = DM.zeros((st_size + (hznLen+1) , 1))
        ubg = DM.zeros((st_size + (hznLen+1) , 1))

        lbg[0: st_size] = 0;                                    ubg[0: st_size] = 0                                     # Optim constr: pred_st - optim_st = 0                  
        lbg[st_size: st_size+ (hznLen+1)] = -inf;               ubg[st_size: st_size+ (hznLen+1)] = 0                   # Path constr: -inf < Euclidian - sum(radii) < 0

        # print("\nDEBUG1: St, U lower bound \n\n", lbx)
        # print("\nDEBUG1: St, U uppper bound \n\n", ubx)
        # print("\nDEBUG1: G(x) lower bound \n", lbg)
        # print("\nDEBUG1: G(x) uppper bound \n", ubg)

        args = { 'lbg': lbg,                    # constraints lower bound
                 'ubg': ubg,                    # constraints upper bound
                 'lbx': lbx,
                 'ubx': ubx}

        '''---------------------------------------------------------------'''

        t0 = 0
        state_init = DM(init_st)                # initial state
        state_target = DM(targ_st)              # target state

        t_step = np.array([])

        u0 = DM.zeros((n_controls, hznLen))      # initial control
        X0 = repmat(state_init, 1, hznLen+1)     # initial state full

        mpc_iter = 0
        cat_states = DM2Arr(X0)
        cat_controls = DM2Arr(u0[:, 0])
        times = np.array([[0]])


        '''--------------------Execute MPC -----------------------------'''
        # STAGE 1 ramp up thrust to hover
        if(scf):
            print("DEBUG : Ramp up to init position")
            # scf.cf.commander.send_position_setpoint(0,0, 4, 0)
            rampMotorsUp(scf.cf)

        # STAGE 2 perform overtake
        main_loop = time()                                                  # return time in sec
        while (norm_2(state_init - state_target) > 1e-1) and (mpc_iter * hznStep < sim_time):
            t1 = time()                                                     # start iter timer
                                                               
            print("DEBUG Measured state", Meas_St)
            args['p'] = vertcat( state_init,  state_target)
            # optimization variable current state
            args['x0'] = vertcat(   reshape(X0, n_states*(hznLen+1), 1),
                                    reshape(u0, n_controls*hznLen, 1))

            sol = solver( x0=args['x0'], lbx=args['lbx'], ubx=args['ubx'], lbg=args['lbg'], ubg=args['ubg'], p=args['p'])

            X0 = reshape(sol['x'][ : n_states * (hznLen+ 1)], n_states, hznLen+1)
            u =  reshape(sol['x'][n_states * (hznLen+ 1): ], n_controls, hznLen)

            cat_states = np.dstack(( cat_states, DM2Arr(X0)))
            cat_controls = np.vstack(( cat_controls, DM2Arr(u[:, 0])))
            t_step = np.append(t_step, t0)
            t0, state_init, u0 = Sys.TimeStep(hznStep, t0, state_init, u, TF_fp)
            
            X0 = horzcat( X0[:, 1:], reshape(X0[:, -1], -1, 1))

            t2 = time()                                                     # stop iter timer
            times = np.vstack(( times, t2-t1))
            mpc_iter = mpc_iter + 1
            print(f'Soln Timestep : {round(t0,3)} s\r', end="")             # State {X0[:, 0]}')
            #print(f"DEBUG{X0[:, 0]}")

            '''---------------------Execute trajectory with CF setpoint tracking--------------------------'''
            if(scf):
                roll, pitch, yawRate, thrust = calc_thrust_setpoint(X0, u)
                scf.cf.commander.send_setpoint(roll, pitch, yawRate, thrust)            #TODO why use this API instead of others ?

        main_loop_time = time()

        # STAGE 3 ramp down thrust to land
        if(scf):
            print("DEBUG : Landing gracefully ")
            rampMotorsDown(scf.cf, thrust)
        ss_error = norm_2(state_init - state_target)

        print('\n\n')
        print('Total time: ', main_loop_time - main_loop)
        print('avg iteration time: ', np.array(times).mean() * 1000, 'ms')
        print('final error: ', ss_error)

        '''-------------------- Visualize -----------------------------'''
        # Plot controls over the simulation period
        plot_dataset( cat_controls, t_step)
        # Plot position( State[0-3]) over simulation period TODO convert state angles (yaw) to euler to indicate heading
        simulate3D(cat_states, times)


    except Exception as ex:
        print("MPC aborted due to Exception:", ex)
    
def calc_thrust_setpoint(St, U):
    #TODO find documentation of this mapping, 
    # euler in deg from q1,      q2,       q3,       q4
    eul_deg = quat2eul([St[3, 0], St[4, 0], St[5, 0], St[6, 0]])

    lin_x  = eul_deg[1]                                         # theta
    lin_y  = -eul_deg[0]                                        # -phi
    lin_z  = krpm2pwm((U[0, 0] + U[1, 0]+ U[2, 0]+ U[3, 0])/4)  # convert average prop RPM to PWM
    ang_z  = St[12, 0] * 180 /pi                                # r in deg
    roll   = lin_y + ROLL_TRIM                                  # TODO calibrate !
    pitch  = lin_x + PITCH_TRIM
    yawrate = ang_z
    thrust = int(min(max(lin_z, 0.0), 60000))
    # print(f"\n DEBUG roll {roll}, pitch {pitch}, yawrate {yawrate}, thrust {thrust}")
    return roll, pitch, yawrate, thrust


def rampMotorsUp(cf):
    thrust_mult = 1
    thrust_step = 1000
    thrust = 0
    # Unlock startup thrust protection
    cf.commander.send_setpoint(0, 0, 0, 0)

    while thrust < hover_rpm:
        cf.commander.send_setpoint(0, 0, 0, thrust)
        sleep(0.1)
        thrust += thrust_step * thrust_mult
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    sleep(0.1)

def rampMotorsDown(cf, CurrentRpm):
    thrust_mult = -1
    thrust_step = 1000
    thrust = CurrentRpm
    # Unlock startup thrust protection
    cf.commander.send_setpoint(0, 0, 0, 0)

    while thrust > 0:
        #print(thrust)
        cf.commander.send_setpoint(0, 0, 0, thrust)
        sleep(0.1)
        thrust += thrust_step * thrust_mult
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    sleep(0.1)