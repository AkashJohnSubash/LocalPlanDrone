from time import time, sleep
from ocp_nlp import setup_nlp
import numpy as np
from common import *
#from sys_dynamics import SysDyn as Sys, Predictor as Pred
from measurement import init_comms, state_meas
from cflib.positioning.motion_commander import MotionCommander

def simulation():
    '''Simulate the trajectory with forward dynamics model (No hardware required)'''

    # Define solver, constraints, NLP
    constraint, model, solver = setup_nlp()   

    # dimensions
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    Nsim = int(sim_Smax * hznLen / stepTime)

    # define data structures
    t0 = 0
    t_step = np.array([])
    
    # States and controls defined as coloumn vectors
    s_0 = DM(np.copy(init_st)).T                                
    s_t = DM(np.copy(targ_st)).T
    u_0 = DM(np.copy(init_u)).T

    # initialize data structures
    mpc_iter = 0
    times = np.array([[0]])
    s0 = DM(repmat(s_0, 1, hznLen+1))
    u0 = DM(repmat(u_0, 1, hznLen))
    state_error = norm_2(s_0[0:3] - s_t[0:3])
    
    # nx * mpc_iter
    cat_states = s_0
    # nu * mpc_iter
    cat_controls = u_0
    #print("DEBUG0", cat_states)
    
    # Stage 1 : No take-off required for simulation

    # Stage 2 : Perform overtake
    setpoints = [0, 0, 0, int(0)]
    roll, pitch, yawRate, thrust_norm = calc_thrust_setpoint(s_0, u_0)
    print(f"Hover normaized RPM {roll, pitch, yawRate, thrust_norm}")
    main_loop = time()
    
    while (state_error > 1e-1) and (mpc_iter < sim_Smax):
        t1 = time()

        # Update initial condition
        # s_0 = solver.get(1, "x")
        #solver.set(0, "lbx", x0)
        #solver.set(0, "ubx", x0)

        # Solve the NLP using acados
        status = solver.solve()
        if status != 0:
            print(f"acados returned status {status} in closed loop iteration {mpc_iter}.")

        # Get solution
        s0 = np.reshape(solver.get(0, "x"), (1, nx))
        u0 = np.reshape(solver.get(0, "u"), (1, nu))
        
        # Append data to plotting list
        cat_states = np.concatenate((cat_states, s0), axis = 1)
        cat_controls = np.concatenate( (cat_controls, u0), axis = 1)
        t_step = np.append(t_step, t0)

        # Save state and Control for next iteration
        t0 = t0 + stepTime
        s_0 = solver.get(1, "x")
        solver.set(0, "lbx", s_0)
        solver.set(0, "ubx", s_0)
        #print(f"current state :{s0} \nnext state :{s_0}")
        # state_init = np.copy(s0[:,1]).flatten()
        # s0 = horzcat( s0[:, 1:], reshape(s0[:, -1], -1, 1))        

        # Generate API setpoint
        roll, pitch, yawRate, thrust_norm = calc_thrust_setpoint(s0[0], u0[0])
        print(f'Soln setpoints {mpc_iter}: {roll}, {pitch}, {yawRate}, {thrust_norm} at {round(t0,3)} s\t') 
        setpoints = np.vstack( (setpoints, np.array([roll, pitch, yawRate, thrust_norm], dtype="object")))

        # update iteration variables
        t2 = time()
        times = np.vstack(( times, t2-t1))
        mpc_iter = mpc_iter + 1
        state_error = norm_2(s_0[0:3]- s_t[0:3])

    main_loop_time = time()
    ss_error_mod = state_error
    print('\n\n')
    print(f'Total time: {main_loop_time - main_loop} ms')
    print(f'Avg iteration time: {np.array(times).mean() * 1000} ms')
    print(f'Final error model: {ss_error_mod}')

    # Stage 3 : No landing required for simulation
    
    return cat_controls, t_step, cat_states, times


def onboard(scf):

    '''Execute MPC to issue optimal controls (RPYT) to CF2.1'''

    args, solver = setup_nlp()                                  # define solver, constraints, NLP

    t0 = 0
    state_init = DM(np.copy(init_st))                           # initial state
    state_target = DM(np.copy(targ_st))                         # target state

    t_step = np.array([])

    u0 = DM(np.full((n_controls, hznLen), hov_rpm))             # initial control
    s0 = repmat(state_init, 1, hznLen+1)                        # initial state full

    mpc_iter = 0
    
    state_error = norm_2(state_init[0:3] - state_target[0:3])
    cat_states = DM2Arr(s0)
    cat_controls = DM2Arr(u0[:, 0])
    times = np.array([[0]])

    # start CF interaction
    init_comms(scf)
    sleep(3)

    # Stage 1 : Take off to init state
    mc = MotionCommander(scf)
    mc._reset_position_estimator()
    mc.take_off(height=0.5)
    print("Hover at takeoff height ")
    sleep(3)
    
    roll, pitch, yawRate, thrust_norm = calc_thrust_setpoint(state_init, u0[:, 0])
    setpoints = [0, 0, 0, thrust_norm]

    # Unlock thrust protection and hover
    scf.cf.commander.send_setpoint(0, 0, 0, 0)
    sleep(0.01)
    scf.cf.commander.send_setpoint(0, 0, 0, thrust_norm)
    
    # Stage 2 : Perform overtake
    main_loop = time()
    state_current = np.copy(state_init)
    while (state_error > 1e-1) and (mpc_iter  < sim_Smax):

        t1 = time()                                                  # start iter timer          
        args['p'] = vertcat( state_current,  state_target)                                                                                            
        # optimization variable current state
        args['x0'] = vertcat(   reshape(s0, n_states*(hznLen+1), 1),
                                reshape(u0, n_controls*hznLen, 1))

        sol = solver( x0=args['x0'], lbx=args['lbx'], ubx=args['ubx'], lbg=args['lbg'], ubg=args['ubg'], p=args['p'])

        #print(f"  Measured state, ctrl {mpc_iter -1}: \n {np.round(state_meas, 4)} at {round(t0,3)}" ) #\n {np.round(state_init, 4)} 

        s0 = reshape(sol['x'][ : n_states * (hznLen+ 1)], n_states, hznLen+1)
        u =  reshape(sol['x'][n_states * (hznLen+ 1): ], n_controls, hznLen)
        
        # Append data to plotting list
        cat_states = np.dstack(( cat_states, DM2Arr(s0)))
        cat_controls = np.vstack( (cat_controls, DM2Arr(u[:, 0])))
        t_step = np.append(t_step, t0)

        # Save state and Control for next iteration
        t0 = t0 + stepTime
        u0 = np.copy(u)
        #state_init = np.copy(s0[:,1]).flatten()
        s0 = horzcat(s0[:, 1:], reshape(s0[:, -1], -1, 1))   
        
        # update iteration variables
        t2 = time()
        times = np.vstack(( times, t2-t1))
        mpc_iter = mpc_iter + 1
        state_current = DM(np.copy(state_meas))
        state_error = norm_2(state_init[0:3] - state_target[0:3])

        # Issue setpoint command (RPYT)
        roll, pitch, yawRate, thrust_norm = calc_thrust_setpoint(s0[:, 0], u[:, 0])
        scf.cf.commander.send_setpoint(roll, pitch, yawRate, thrust_norm)
        #print(f'Soln setpoints {mpc_iter}: {roll}, {pitch}, {yawRate}, {thrust_norm} at {round(t0,3)} s\t') 
      
    # Stage 3 : Perform landing sequence
    mc.land()
    sleep(3)

    main_loop_time = time()
    ss_error_mod = norm_2(state_init - state_target)
    ss_error_real = norm_2(state_meas - state_target)

    print('\n\n')
    print('Total time: ', main_loop_time - main_loop)
    print('avg iteration time: ', np.array(times).mean() * 1000, 'ms')
    print('final error sim: ', ss_error_mod)
    print('final error real: ', ss_error_real)

    return cat_controls, t_step, cat_states, times