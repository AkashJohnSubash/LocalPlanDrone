from time import time, sleep
import numpy as np
import sys

from cflib.crazyflie.log import LogConfig
from threading import Event

from common import init_st, quatDecompress

deck_attached_event = Event()
state_meas =  np.zeros(13)
att_quat = np.zeros(4)

def init_comms(scf):
    
    scf.cf.param.add_update_callback(group='deck', name='bcLighthouse4', cb=deck_light_cbk)
    start_state_rx(scf)
    sleep(1)

def deck_light_cbk(_, value_str):
    
    value = int(value_str)
    if value:
        deck_attached_event.set()
    else:
        print('NO deck attached! Either lighthouse/ multiranger needed')
        sys.exit(1)

def start_state_rx(scf):
    '''Log data from the CF stabilizer via Radio'''

    # Define variables in group
    stabZ = LogConfig(name='StabZ', period_in_ms=20)
    stabZ.add_variable('stateEstimateZ.x', 'int16_t')
    stabZ.add_variable('stateEstimateZ.y', 'int16_t')
    stabZ.add_variable('stateEstimateZ.z', 'int16_t')
    stabZ.add_variable('stateEstimateZ.quat', 'uint32_t')
    stabZ.add_variable('stateEstimateZ.vx', 'int16_t')
    stabZ.add_variable('stateEstimateZ.vy', 'int16_t')
    stabZ.add_variable('stateEstimateZ.vz', 'int16_t')
    stabZ.add_variable('stateEstimateZ.rateRoll', 'int16_t')
    stabZ.add_variable('stateEstimateZ.ratePitch', 'int16_t')
    stabZ.add_variable('stateEstimateZ.rateYaw', 'int16_t')

    #register callbacks for group
    scf.cf.log.add_config(stabZ)
    stabZ.data_received_cb.add_callback(StabZ_cbk)
    stabZ.start()
    


def StabZ_cbk(timestamp, data, logconf):
    '''Callback function to decode state data from stateEstimateZ group'''

    state_meas[0] = data['stateEstimateZ.x']/1000
    state_meas[1] = data['stateEstimateZ.y']/1000
    state_meas[2] = data['stateEstimateZ.z']/1000
    
    state_meas[7] = data['stateEstimateZ.vx']/1000
    state_meas[8] = data['stateEstimateZ.vy']/1000
    state_meas[9] = data['stateEstimateZ.vz']/1000
    
    state_meas[10] = data['stateEstimateZ.rateRoll']/1000
    state_meas[11] = data['stateEstimateZ.ratePitch']/1000
    state_meas[12] = data['stateEstimateZ.rateYaw']/1000
    
    att_quat = np.copy(quatDecompress(np.uint32(data['stateEstimateZ.quat'])))
    state_meas[3] = att_quat[3]                         # qx
    state_meas[4] = att_quat[0]                         # qy
    state_meas[5] = att_quat[1]                         # qz
    state_meas[6] = att_quat[2]                         # qw
    