import argparse
from MPC import mpc_traj

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
    return parser

if __name__ == '__main__':
    parser = argparse_init()
    args = parser.parse_args()
    print(args.virtual)
    if args.virtual:
        mpc_traj.gen_path()
    else :
        print("only commandline argument <-v> supported currently")
    print("Flight completed")