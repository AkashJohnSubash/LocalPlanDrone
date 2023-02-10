from time import time, sleep
from ocp_nlp import setup_nlp
import numpy as np
#from casadi import *
from common import *
from sys_dynamics import SysDyn as Sys, Predictor as Pred
from measurement import state_meas, pwm_req, pwm_set, set_rpyt, req_rpyt# att_cmp
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander

def simulation():
    '''Simulate the tajectory with forward dynamics model (No hardware required)'''

    dynamics_fp, args, solver = setup_nlp()                 # define solver, constraints, NLP

    t0 = 0
    state_init = DM(init_st)                                # initial state
    state_target = DM(targ_st)                              # target state

    t_step = np.array([])

    u0 = DM(np.full((n_controls, hznLen), hover_krpm))      # initial control
    X0 = repmat(state_init, 1, hznLen+1)                    # initial state full

    mpc_iter = 0
    cat_states = DM2Arr(X0)
    cat_controls = DM2Arr(u0[:, 0])
    times = np.array([[0]])

    '''--------------------Execute MPC for simulation-----------------------------'''

    # STAGE 2 perform overtake
    setpoints = [0, 0, 0, int(0)]
    roll, pitch, yawRate, thrust_norm = calc_thrust_setpoint(state_init, u0[:, 0])
    print(f"Hover normaized RPM  {roll, pitch, yawRate, thrust_norm}")
    main_loop = time()                                                      # return time in sec
    while (norm_2(state_init[0:3] - state_target[0:3]) > 1e-1) and (mpc_iter * hznStep < sim_time):
        t1 = time()                                                         # start iter timer                          
                                                                                                                       
        args['p'] = vertcat( state_init,  state_target)                     # optimization variable current state
        args['x0'] = vertcat(   reshape(X0, n_states*(hznLen+1), 1),
                                reshape(u0, n_controls*hznLen, 1))

        # Solve the NLP using IPOPT
        sol = solver( x0=args['x0'], lbx=args['lbx'], ubx=args['ubx'], lbg=args['lbg'], ubg=args['ubg'], p=args['p'])

        X0 = reshape(sol['x'][ : n_states * (hznLen+ 1)], n_states, hznLen+1)
        u =  reshape(sol['x'][n_states * (hznLen+ 1): ], n_controls, hznLen)

        # Append data to plotting list
        cat_states = np.dstack(( cat_states, DM2Arr(X0)))
        cat_controls = np.vstack( (cat_controls, DM2Arr(u[:, 0])))
        t_step = np.append(t_step, t0)

        # Save state and Control for next iteration
        t0 = t0 + hznLen
        u0 = np.copy(u)
        state_init = np.copy(X0[:,1])
        X0 = horzcat( X0[:, 1:], reshape(X0[:, -1], -1, 1))

        t2 = time()                                                     # stop iter timer
        times = np.vstack(( times, t2-t1))
        mpc_iter = mpc_iter + 1
        

        roll, pitch, yawRate, thrust_norm = calc_thrust_setpoint(X0[:, 1], u[:, 1])
        print(f'Soln Timestep {mpc_iter}: {roll}, {pitch}, {yawRate}, {thrust_norm} at {round(t0,3)} ms\t')
        # print(f'State {mpc_iter}: {X0[:, 1]}')  
        #setpoints = np.vstack( (setpoints, np.array([roll, pitch, yawRate, thrust_norm], dtype="object")))

    '''---------------------Execute trajectory with CF setpoint tracking--------------------------'''

    main_loop_time = time()
    ss_error_mod = norm_2(state_init - state_target)
    print('\n\n')
    print('Total time: ', main_loop_time - main_loop)
    print('Avg iteration time: ', np.array(times).mean() * 1000, 'ms')
    print('Final error model: ', ss_error_mod)
    
    return cat_controls, t_step, cat_states, times


def onboard(scf, realtime):
    '''Execute the tajectory on CF2.1 '''

    dynamics_fp, args, solver = setup_nlp()                 # define solver, constraints, NLP
    
    t0 = 0
    state_init = DM(np.copy(init_st))                       # initial state
    state_target = DM(np.copy(targ_st))                     # target state

    t_step = np.array([])
    setpoints = np.array([])

    u0 = DM(np.full((n_controls, hznLen), hover_krpm))      # initial control
    X0 = repmat(state_init, 1, hznLen+1)                    # initial state full

    mpc_iter = 0
    cat_states = DM2Arr(X0)
    cat_controls = DM2Arr(u0[:, 0])
    times = np.array([[0]])


    '''--------------------Feed MPC traj to CF using RPYT setpoints-----------------------------'''
    try:
        # STAGE 1 ramp up thrust to hover
        mc = MotionCommander(scf)
        mc._reset_position_estimator()
        mc.take_off(height=state_init[2])
        print("Execute TAKEOFF height ")
        sleep(2)
        #hl = PositionHlCommander(scf)
        #hl._activate_high_level_commander()
        #hl.take_off(height=0.5, velocity=v_max)
        # STAGE 2 perform overtake
        main_loop = time()                                                          # return time in sec
        hover_U = np.full((4,1), hover_krpm)
        roll, pitch, yawRate, thrust_norm = calc_thrust_setpoint(state_init, u0[:, 0])
        setpoints = [0, 0, 0, thrust_norm]
        #print(f"DEBUG: Hover for 2s with norm rpm {thrust_norm}")
        scf.cf.commander.send_setpoint(0, 0, 0, 0)
        
        # # Unlock startup thrust protection
        scf.cf.commander.send_setpoint(0, 0, 0, thrust_norm)
          
        print("Execute HOVER ")
        
        #sleep(3)                            
        state_pred0 = np.copy(state_meas)
        while (norm_2(state_init[0:3] - state_target[0:3]) > 1e-1) and (mpc_iter * hznStep < sim_time):
            #print(f"REQ RPYT {mpc_iter}: {req_rpyt}")
            #print(f"SET RPYT {mpc_iter}: {set_rpyt}")
            #print(f"SET RPYT {mpc_iter-1}: {set_rpyt}\n")

            t1 = time()                                                         # start iter timer          
            args['p'] = vertcat( state_init,  state_target)                                                                                            
            #print(f"DEBUG Measured state \t {state_meas}" )
            # optimization variable current state
            args['x0'] = vertcat(   reshape(X0, n_states*(hznLen+1), 1),
                                    reshape(u0, n_controls*hznLen, 1))

            sol = solver( x0=args['x0'], lbx=args['lbx'], ubx=args['ubx'], lbg=args['lbg'], ubg=args['ubg'], p=args['p'])

            X0 = reshape(sol['x'][ : n_states * (hznLen+ 1)], n_states, hznLen+1)
            u =  reshape(sol['x'][n_states * (hznLen+ 1): ], n_controls, hznLen)
            
            # Append data to plotting list
            cat_states = np.dstack(( cat_states, DM2Arr(X0)))
            cat_controls = np.vstack(( cat_controls, DM2Arr(u[:, 0])))
            t_step = np.append(t_step, t0)

            # Save state and Control for next iteration
            t0 = t0 + hznLen
            u0 = np.copy(u)
            state_init = np.copy(X0[:,1])
            
            X0 = horzcat( X0[:, 1:], reshape(X0[:, -1], -1, 1))
            # print(f'Soln Timestep : {round(t0,3)} s\r', end="")             # State {X0[:, 0]}')
            roll, pitch, yawRate, thrust_norm = calc_thrust_setpoint(state_init, u[:, 0])
            #print(f"DEBUG Measured Setpoint :ROLL, PITCH, YAWRATE, RPM}")
            #print(f"Controls {mpc_iter}: {u[:, 0]}")

            '''---------------------Execute trajectory with CF setpoint tracking--------------------------'''
                
            #print(f"Onboard Pos {mpc_iter}: {state_meas[:3]}")
            #print(f"Onboard setpoint {mpc_iter}: {set_rpyt} \n")
            #print(f"pwm set{mpc_iter}: {pwm_set}\n")
            #scf.cf.commander.send_setpoint(roll, pitch, yawRate, thrust_norm)
            # ''''TEST Position setpoint command'''
            #hl.go_to(X0[0, 0], X0[1, 0], X0[2, 0])
            #print(f"\nMPC RPYT setpoint {mpc_iter +1}: {roll}, {pitch}, {yawRate}, {thrust_norm}")
            #print(f"MPC Pos setpoint {mpc_iter +1}: {X0[:3]}")
            # print(f"REQ RPYT {mpc_iter +1}: {req_rpyt}")
            # print(f"SET RPYT {mpc_iter +1}: {set_rpyt}")
            #print(f"Expected Pos {mpc_iter +1}: {state_pred0[:3]}")
            #print(f"Error {norm_2(state_meas[0:3] - state_target[0:3])}")
            setpoints = np.vstack( (setpoints, np.array([roll, pitch, yawRate, thrust_norm], dtype="object")))
            t2 = time()                                                             # stop iter timer
            times = np.vstack(( times, t2-t1))
            mpc_iter = mpc_iter + 1

        #print("DEBUG \n Non real time execution \n")
        main_loop_time = time()

        # Offline setpoint commands
        if(realtime != True):
            print("Execute offline setpoints")
            for i in range(0 ,len(setpoints[:, 0])):
                #print(f"Measured State {i}: {np.round(state_meas, 2)}")
                #print(f"pwm req: {pwm_req}")
                #print(f"Onboard Pos {i}: {state_meas[:3]}")
                #print(f"Onboard setpoint {i}: {Ctrl_rpyt}")
                #print(f"pwm set{i}: {pwm_set}\n")
                print(f"Offline MPC Setpoint {i+1}: {setpoints[i, 0]}, {setpoints[i, 1]}, {setpoints[i, 2]}, {setpoints[i, 3]}")
                scf.cf.commander.send_setpoint(setpoints[i, 0], setpoints[i, 1], setpoints[i, 2], setpoints[i, 3])
                sleep(0.1)
        print("\nExecute HOVER2 \n")
        roll, pitch, yawRate, thrust_norm = calc_thrust_setpoint(state_init, hover_U)
        scf.cf.commander.send_setpoint(0, 0, 0, thrust_norm)
        # STAGE 4 ramp down thrust to land
        print("Execute LAND command")
        mc.land()
        #hl.land(velocity = v_max)
        sleep(3)

        main_loop_time = time()
        ss_error_mod = norm_2(state_init - state_target)
        ss_error_real = norm_2(state_meas - state_target)

        print('\n\n')
        print('Total time: ', main_loop_time - main_loop)
        #print('iteration time: ', times, )
        print('avg iteration time: ', np.array(times).mean() * 1000, 'ms')
        print('final error sim: ', ss_error_mod)
        print('final error real: ', ss_error_real)
    
    except Exception as Ex:
        print(f"Exception  during flight{Ex}")

    return cat_controls, t_step, cat_states, times