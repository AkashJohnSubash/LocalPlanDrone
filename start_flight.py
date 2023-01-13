import argparse
from MPC import ocp_nlp

import logging
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

import argparse

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E2')
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
    parser.add_argument("-c", "--connect", action='store_true', help="Connect to the drone then disconnects")
    parser.add_argument("-s", "--logsync", action='store_true', help="Synchronous log of roll, pitch and yaw")
    parser.add_argument("-a", "--logasync", action='store_true', help="Asynchronous log of roll, pitch and yaw")
    parser.add_argument("-p", "--param", action='store_true', help="asynchronous set of estimator parameter")
    parser.add_argument("-v", "--virtual", action='store_true', help="Only simulate the optimal trajectory")
    parser.add_argument("-l", "--lab", action='store_true', help="Laborotory flight")
    return parser

if __name__ == '__main__':
    parser = argparse_init()
    args = parser.parse_args()
    if args.virtual:
        ocp_nlp.gen_path(visualize = True)
    elif args.lab:
        cflib.crtp.init_drivers()
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
            lg_stab.add_variable('stabilizer.roll', 'float')
            lg_stab.add_variable('stabilizer.pitch', 'float')
            lg_stab.add_variable('stabilizer.yaw', 'float')
            simple_log(scf, lg_stab)
            ocp_nlp.gen_path(visualize = False)
    else :
        print("only commandline argument <-v> supported currently")
    print("Flight completed")