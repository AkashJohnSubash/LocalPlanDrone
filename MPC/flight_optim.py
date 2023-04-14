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
    Nsim = int(sim_Smax * N / stepTime)

    # define data structures
    t0 = 0
    t_step = np.array([])
    
    # States and controls defined as coloumn vectors
    s_0 = DM(np.copy(init_st))                                
    s_t = DM(np.copy(targ_st))
    u_0 = DM(np.copy(init_u))

    # initialize data structures
    mpc_iter = 0
    times = np.array([[0]])
    #u0 = DM(repmat(u_0, 1, N))
    state_error = norm_2(s_0[0:3] - s_t[0:3])
    
    # nx X N X mpc_iter (to plot state during entire Horizon)
    state_traj = repmat(s_0, 1, N) 
    # nu X mpc_iter
    control_traj = repmat(u_0, 1, N) 
    
    # Stage 1 : No take-off required for simulation

    # Stage 2 : Perform overtake
    setpoints = [0, 0, 0, int(0)]
    roll, pitch, yawRate, thrust_norm = calc_thrust_setpoint(s_0, u_0)
    main_loop = time()
    
    while (state_error > 1e-1) and (mpc_iter < sim_Smax):
        t1 = time()
        s_i = np.copy(s_0)
        s_N = np.copy(s_0)

        # Solve the NLP using acados
        status = solver.solve()
        if status != 0:
            print(f"acados returned status {status} in closed loop iteration {mpc_iter}.")

        # Get solution
        s_0 = np.reshape(solver.get(0, "x"), (nx, 1))
        u_0 = np.reshape(solver.get(0, "u"), (nu, 1))
        
        # Append data to plotting list
        for i in range(1, N ):
            s_i = np.reshape(solver.get(i, "x"), (nx, 1))
            s_N = np.concatenate((s_N, s_i), axis = 1)
        state_traj = np.dstack((state_traj, s_N))
        control_traj = np.concatenate((control_traj, u_0), axis = 1)
        t_step = np.append(t_step, t0)

        # Save state and Control for next iteration
        t0 = t0 + stepTime
        s_ini = solver.get(1, "x")
        solver.set(0, "lbx", s_ini)
        solver.set(0, "ubx", s_ini)    

        # Generate API setpoint
        roll, pitch, yawRate, thrust_norm = calc_thrust_setpoint(s_0, u_0)
        print(f'Soln setpoints {mpc_iter}: {roll}, {pitch}, {yawRate}, {thrust_norm} at {round(t0,3)} s\t') 
        
        # update iteration variables
        t2 = time()
        times = np.vstack(( times, t2-t1))
        mpc_iter = mpc_iter + 1
        state_error = norm_2(s_0[0:3]- s_t[0:3])

    main_loop_time = time()
    ss_error_mod = state_error
    print('\n\n')
    print(f'Total time: {(main_loop_time - main_loop)*1000} ms')
    print(f'Avg iteration time: {np.array(times).mean() * 1000} ms')
    print(f'Final error model: {ss_error_mod}')

    # Stage 3 : No landing required for simulation
    
    return control_traj, t_step, state_traj, times


def onboard(scf):

    '''Execute MPC to issue optimal controls (RPYT) to CF2.1'''

  # Define solver, constraints, NLP
    constraint, model, solver = setup_nlp()   

    # dimensions
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    Nsim = int(sim_Smax * N / stepTime)

    # define data structures
    t0 = 0
    t_step = np.array([])
    
    # States and controls defined as coloumn vectors
    s_0 = DM(np.copy(init_st))                                
    s_t = DM(np.copy(targ_st))
    u_0 = DM(np.copy(init_u))

    # initialize data structures
    mpc_iter = 0
    times = np.array([[0]])
    #u0 = DM(repmat(u_0, 1, N))
    state_error = norm_2(s_0[0:3] - s_t[0:3])
    
    # nx X N X mpc_iter (to plot state during entire Horizon)
    state_traj = repmat(s_0, 1, N) 
    # nu X mpc_iter
    control_traj = repmat(u_0, 1, N) 

    # start CF interaction
    init_comms(scf)
    sleep(3)

    # Stage 1 : Take off to init state
    mc = MotionCommander(scf)
    mc._reset_position_estimator()
    mc.take_off(height=0.5)
    print("Hover at takeoff height ")
    sleep(3)
    
    roll, pitch, yawRate, thrust_norm = calc_thrust_setpoint(s_0, u_0)

    # Unlock thrust protection and hover
    scf.cf.commander.send_setpoint(0, 0, 0, 0)
    sleep(0.01)
    scf.cf.commander.send_setpoint(0, 0, 0, thrust_norm)
    
    # Stage 2 : Perform overtake
    main_loop = time()
    state_current = np.copy(init_st)
    while (state_error > 1e-1) and (mpc_iter  < sim_Smax):
        t1 = time()
        s_i = np.copy(s_0)
        s_N = np.copy(s_0)

        # Solve the NLP using acados
        status = solver.solve()
        if status != 0:
            print(f"acados returned status {status} in closed loop iteration {mpc_iter}.")

        # Get solution
        s_0 = np.reshape(solver.get(0, "x"), (nx, 1))
        u_0 = np.reshape(solver.get(0, "u"), (nu, 1))
        
        # Append data to plotting list
        for i in range(1, N ):
            s_i = np.reshape(solver.get(i, "x"), (nx, 1))
            s_N = np.concatenate((s_N, s_i), axis = 1)
        state_traj = np.dstack((state_traj, s_N))
        control_traj = np.concatenate((control_traj, u_0), axis = 1)
        t_step = np.append(t_step, t0)

        # Save state and Control for initialization in next iteration
        t0 = t0 + stepTime
        state_current = np.copy(state_meas)
        solver.set(0, "lbx", state_current)
        solver.set(0, "ubx", state_current)    
        
        # Issue setpoint command (RPYT)
        roll, pitch, yawRate, thrust_norm = calc_thrust_setpoint(s_0, u_0)
        scf.cf.commander.send_setpoint(roll, pitch, yawRate, thrust_norm)
        print(f'Soln setpoints {mpc_iter}: {roll}, {pitch}, {yawRate}, {thrust_norm} at {round(t0,3)} s\t') 

        # update iteration variables
        t2 = time()
        times = np.vstack(( times, t2-t1))
        mpc_iter = mpc_iter + 1
        state_error = norm_2(s_0[0:3]- s_t[0:3])
      
    # Stage 3 : Perform landing sequence
    mc.land()
    sleep(3)

    main_loop_time = time()
    ss_error_real = norm_2(state_meas - s_t)

    print('\n\n')
    print(f'Total time: {(main_loop_time - main_loop)*1000} ms')
    print(f'Avg iteration time: {np.array(times).mean() * 1000} ms')
    print(f'Final error real: {ss_error_real}')

    return control_traj, t_step, state_traj, times