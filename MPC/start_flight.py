from MPC import ocp_sim, ocp_cf
from MPC.simulation_code import simulate3D, plot_dataset

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

if __name__ == '__main__':
    
    parser = argparse_init()
    args = parser.parse_args()
    
    
    if args.lab:
        # URI to the Crazyflie to connect to
        cflib.crtp.init_drivers()
        uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E007')
        print("Lab flight")
        with SyncCrazyflie(uri, cf = Crazyflie(rw_cache='./cache')) as scf:
            measurement.init_drone(scf)
            cat_U, t_step, cat_ST, times_ST = ocp_cf.traj_commander(scf)
    else:
        print("Simulated flight")
        cat_U, t_step, cat_ST, times_ST = ocp_sim.traj_commander()

        '''-------------------- Visualize -----------------------------'''
    # Plot controls over the simulation period
    plot_dataset( cat_U, t_step)
    # Plot position( State[0-3]) over simulation period TODO convert state angles (yaw) to euler to indicate heading
    simulate3D(cat_ST, times_ST)