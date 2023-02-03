import flight_optim
from visualize import simulate3D, plot_dataset

import argparse

from cflib.utils import uri_helper
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig #TODO remove ?
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


from common import *
import measurement


def argparse_init():
    '''
    Initialization for cmd line args
    '''
    parser = argparse.ArgumentParser()
    parser.add_argument("-l", "--lab", action='store_true', help="Laborotory flight")
    parser.add_argument("-rt", "--RealTime", action='store_true', help="MPC computation in flight time ")
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
            cat_U, t_step, cat_ST, times_ST = flight_optim.onboard(scf, realtime = args.RealTime)
    else:
        print("Simulated flight")
        cat_U, t_step, cat_ST, times_ST = flight_optim.simulation()

        '''-------------------- Visualize -----------------------------'''
    # Plot controls over the simulation period
    plot_dataset(cat_U, t_step)
    # Plot position( State[0-3]) over simulation period TODO convert state angles (yaw) to euler to indicate heading
    simulate3D(cat_ST, times_ST)