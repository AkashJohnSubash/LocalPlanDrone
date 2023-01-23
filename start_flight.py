from MPC import ocp_nlp

import argparse
import logging
import time
import sys
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger


# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
deck_attached_event = Event()
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def simple_log(scf, logconf):

    with SyncLogger(scf, logconf) as logger:
        for log_entry in logger:
            timestamp, logconf_name, data = log_entry

            print('[%d][%s]: %s' % (timestamp, logconf_name, data)) 
            # remove break to continuously read logs
            break

def argparse_init():
    '''
    Initialization for cmd line args
    '''
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--virtual", action='store_true', help="Only simulate the optimal trajectory")
    parser.add_argument("-l", "--lab", action='store_true', help="Laborotory flight")
    return parser

def deck_light(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Lighthouse deck is attached!')
    else:
        print('Lighthouse deck is NOT attached!')

# def reset_estimator(scf):
#         cf = scf.cf
#         cf.param.set_value('kalman.resetEstimation', '1')
#         time.sleep(0.1)
#         cf.param.set_value('kalman.resetEstimation', '0')

# def KfPos_cbk(timestamp, data, logconf):
#     x = data['kalman.stateX']
#     y = data['kalman.stateY']
#     z = data['kalman.stateZ']
#     print(f'KF pos: ({x}, {y}, {z}')

# def KfAtt_cbk(timestamp, data, logconf):
#     q0 = data['kalman.q0']
#     q1 = data['kalman.q1']
#     q2 = data['kalman.q2']
#     q3 = data['kalman.q3']
#     print(f'KF att: {q0}, {q1}, {q2}, {q3}\n')

# def print_state_kf(scf):
#     # Log data from the onboard Kalman filter via CfRadio
#     kal_pos = LogConfig(name='KalPos', period_in_ms=100)
#     kal_pos.add_variable('kalman.stateX', 'float')
#     kal_pos.add_variable('kalman.stateY', 'float')
#     kal_pos.add_variable('kalman.stateZ', 'float')

#     kal_att = LogConfig(name='KalAtt', period_in_ms=100)
#     kal_att.add_variable('kalman.q0', 'float')
#     kal_att.add_variable('kalman.q1', 'float')
#     kal_att.add_variable('kalman.q2', 'float')
#     kal_att.add_variable('kalman.q3', 'float')

#     scf.cf.log.add_config(kal_pos)
#     kal_pos.data_received_cb.add_callback(KfPos_cbk)
#     kal_pos.start()

#     scf.cf.log.add_config(kal_att)
#     kal_att.data_received_cb.add_callback(KfAtt_cbk)
#     kal_att.start()

# def LhPos_callback(timestamp, data, logconf):
#     x = data['lighthouse.x']
#     y = data['lighthouse.y']
#     z = data['lighthouse.z']
#     print(f'LH pos: ({x}, {y}, {z})')

# def print_state_lh(scf):
#     # Log data from the Lighthouse via CfRadio
#     Lh_pos = LogConfig(name='LhPos', period_in_ms=100)
#     Lh_pos.add_variable('lighthouse.x', 'float')
#     Lh_pos.add_variable('lighthouse.y', 'float')
#     Lh_pos.add_variable('lighthouse.z', 'float')
#     scf.cf.log.add_config(Lh_pos)
#     Lh_pos.data_received_cb.add_callback(LhPos_callback)
#     Lh_pos.start()

def StabPos_cbk(timestamp, data, logconf):
    x = data['stateEstimateZ.x']
    y = data['stateEstimateZ.y']
    z = data['stateEstimateZ.z']
    q = data['stateEstimateZ.quat']
    # print(f'ST pos: {x}, {y}, {z}, {q}')


def StabAtt_cbk(timestamp, data, logconf):
    qw = data['stateEstimate.qw']
    qx = data['stateEstimate.qx']
    qy = data['stateEstimate.qy']
    qz = data['stateEstimate.qz']
    
    #print(f'ST att: {qw}, {qx}, {qy}, {qz}\n')

def StabVel_cbk(timestamp, data, logconf):
    u = data['stateEstimateZ.vx']
    v = data['stateEstimateZ.vy']
    w = data['stateEstimateZ.vz']
    p = data['stateEstimateZ.rateRoll']
    q = data['stateEstimateZ.ratePitch']
    r = data['stateEstimateZ.rateYaw']
    
    # print(f'Lin vel: {u}, {v}, {w}\n')
    # print(f'Ang vel: {p}, {q}, {r}\n')

def print_state(scf):
    # Log data from the CF stabilizer via Radio
    stab_pos = LogConfig(name='StabPos', period_in_ms=100)
    stab_pos.add_variable('stateEstimateZ.x', 'float')
    stab_pos.add_variable('stateEstimateZ.y', 'float')
    stab_pos.add_variable('stateEstimateZ.z', 'float')
    #stab_pos.add_variable('stateEstimateZ.quat', 'float')

    # stab_att = LogConfig(name='StabAtt', period_in_ms=100)
    # stab_att.add_variable('stateEstimate.qw', 'float')
    # stab_att.add_variable('stateEstimate.qx', 'float')
    # stab_att.add_variable('stateEstimate.qy', 'float')
    # stab_att.add_variable('stateEstimate.qz', 'float')

    # stab_vel = LogConfig(name='StabVel', period_in_ms=100)
    # stab_vel.add_variable('stateEstimateZ.vx', 'float')
    # stab_vel.add_variable('stateEstimateZ.vy', 'float')
    # stab_vel.add_variable('stateEstimateZ.vz', 'float')
    # stab_vel.add_variable('stateEstimateZ.rateRoll', 'float')
    # stab_vel.add_variable('stateEstimateZ.ratePitch', 'float')
    # stab_vel.add_variable('stateEstimateZ.rateYaw', 'float')

    scf.cf.log.add_config(stab_pos)
    stab_pos.data_received_cb.add_callback(StabPos_cbk)
    stab_pos.start()
    
    # scf.cf.log.add_config(stab_att)
    # stab_att.data_received_cb.add_callback(StabAtt_cbk)
    # stab_att.start()

    # scf.cf.log.add_config(stab_vel)
    # stab_vel.data_received_cb.add_callback(StabVel_cbk)
    # stab_vel.start()

if __name__ == '__main__':
    parser = argparse_init()
    args = parser.parse_args()
    
    if args.virtual:
        ocp_nlp.traj_commander()
    
    elif args.lab:
        cflib.crtp.init_drivers()

        with SyncCrazyflie(uri, cf = Crazyflie(rw_cache='./cache')) as scf:
            # lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
            # lg_stab.add_variable('stabilizer.roll', 'float')
            # lg_stab.add_variable('stabilizer.pitch', 'float')
            # lg_stab.add_variable('stabilizer.yaw', 'float')
            # simple_log(scf, lg_stab)
            scf.cf.param.add_update_callback(group='deck', name='bcLighthouse4', cb=deck_light)
            time.sleep(1)
            #HlCom = PositionHlCommander(scf)
            
            # if not deck_attached_event.wait(timeout=5):
            #     print('No lighthouse deck detected!')
            #     sys.exit(1)

            #print_state_kf(scf)
            # print_state_lh(scf)
            print_state(scf)
            # Go to a coordinate
            # HlCom.take_off()
            ocp_nlp.traj_commander(scf)
            print("Trajectory executed")

            # finally:
            #     #HlCom.land()
            #     #print("Land drone")
            # TODO tear down the interfaces

    else :
        print("only commandline argument <-v> supported currently")
    print("Flight completed")