from MPC import ocp_nlp

import argparse

from cflib.utils import uri_helper
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


import numpy as np
from MPC.common import *
from MPC import measurement

# Only output errors from the logging framework
# logging.basicConfig(level=logging.ERROR)


def argparse_init():
    '''
    Initialization for cmd line args
    '''
    parser = argparse.ArgumentParser()
    parser.add_argument("-l", "--lab", action='store_true', help="Laborotory flight")
    return parser

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



if __name__ == '__main__':
    
    parser = argparse_init()
    args = parser.parse_args()
    
    
    if args.lab:
        # URI to the Crazyflie to connect to
        cflib.crtp.init_drivers()
        uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
        with SyncCrazyflie(uri, cf = Crazyflie(rw_cache='./cache')) as scf:
            measurement.init_drone(scf)
            ocp_nlp.traj_commander(scf)
    
    else:
        print("Simulated flight")
        ocp_nlp.traj_commander()
    print("Flight completed")