import flight_optim
from visualize import plot_controls, plot_states

import argparse

from cflib.utils import uri_helper
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from common import *


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
            cat_U, t_step, cat_ST, times_ST = flight_optim.onboard(scf)
    else:
        print("Simulated flight")
        cat_U, t_step, cat_ST, times_ST = flight_optim.simulation()

        '''-------------------- Visualize -----------------------------'''
    # Plot controls, state (position) trajectory
    plot_controls(cat_U, t_step)
    # Plot position( State[0-3]) over simulation period
    plot_states(cat_ST, times_ST)