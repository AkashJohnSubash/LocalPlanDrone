from time import time, sleep
import numpy as np
import sys

from cflib.crazyflie.log import LogConfig
from threading import Event

from common import init_st

deck_attached_event = Event()
state_meas =  init_st
pwm_set =  np.zeros(4)
pwm_req =  np.zeros(4)


def init_drone(scf):
    scf.cf.param.add_update_callback(group='deck', name='bcLighthouse4', cb=deck_light_cbk)
    start_state_rx(scf)
    sleep(1)

def deck_light_cbk(_, value_str):
    value = int(value_str)
    if value:
        deck_attached_event.set()
    else:
        print('Lighthouse deck is NOT attached!')
        sys.exit(1)

def start_state_rx(scf):

    #Log data from the CF stabilizer via Radio
    stabZ = LogConfig(name='StabZ', period_in_ms=100)
    # stabZ.add_variable('stateEstimateZ.x', 'int16_t')
    # stabZ.add_variable('stateEstimateZ.y', 'int16_t')
    # stabZ.add_variable('stateEstimateZ.z', 'int16_t')
    # stabZ.add_variable('stateEstimateZ.quat', 'float')
    stabZ.add_variable('stateEstimateZ.vx', 'int16_t')
    stabZ.add_variable('stateEstimateZ.vy', 'int16_t')
    stabZ.add_variable('stateEstimateZ.vz', 'int16_t')
    stabZ.add_variable('stateEstimateZ.rateRoll', 'int16_t')
    stabZ.add_variable('stateEstimateZ.ratePitch', 'int16_t')
    stabZ.add_variable('stateEstimateZ.rateYaw', 'int16_t')
    
    stabPos = LogConfig(name='StabPos', period_in_ms=100)
    stabPos.add_variable('stateEstimate.x', 'float')
    stabPos.add_variable('stateEstimate.y', 'float')
    stabPos.add_variable('stateEstimate.z', 'float')
    
    stabAtt = LogConfig(name='StabAtt', period_in_ms=100)
    stabAtt.add_variable('stateEstimate.qw', 'float')
    stabAtt.add_variable('stateEstimate.qx', 'float')
    stabAtt.add_variable('stateEstimate.qy', 'float')
    stabAtt.add_variable('stateEstimate.qz', 'float')

    # stabLvel = LogConfig(name='StabPos', period_in_ms=100)
    # stabLvel.add_variable('stateEstimate.vx', 'float')
    # stabLvel.add_variable('stateEstimate.vy', 'float')
    # stabLvel.add_variable('stateEstimate.vz', 'float')
    
    # stabAvel = LogConfig(name='StabPos', period_in_ms=100)
    # stabAvel.add_variable('stateEstimate.rateRoll', 'float')
    # stabAvel.add_variable('stateEstimate.ratePitch', 'float')
    # stabAvel.add_variable('stateEstimate.rateYaw', 'float')

    # mot_set = LogConfig(name='MotSet', period_in_ms=100)
    # mot_set.add_variable('motor.m1', 'int32_t')
    # mot_set.add_variable('motor.m2', 'int32_t')
    # mot_set.add_variable('motor.m3', 'int32_t')
    # mot_set.add_variable('motor.m4', 'int32_t')

    # mot_req = LogConfig(name='MotReq', period_in_ms=100)
    # mot_req.add_variable('motor.m1req', 'int32_t')
    # mot_req.add_variable('motor.m2req', 'int32_t')
    # mot_req.add_variable('motor.m3req', 'int32_t')
    # mot_req.add_variable('motor.m4req', 'int32_t')

    scf.cf.log.add_config(stabZ)
    stabZ.data_received_cb.add_callback(StabZ_cbk)
    stabZ.start()
    
    scf.cf.log.add_config(stabPos)
    stabPos.data_received_cb.add_callback(StabPos_cbk)
    stabPos.start()

    scf.cf.log.add_config(stabAtt)
    stabAtt.data_received_cb.add_callback(StabAtt_cbk)
    stabAtt.start()

    # scf.cf.log.add_config(stabLvel)
    # stabLvel.data_received_cb.add_callback(StabLVel_cbk)
    # stabLvel.start()

    # scf.cf.log.add_config(stabAvel)
    # stabAvel.data_received_cb.add_callback(StabAVel_cbk)
    # stabAvel.start()

    # scf.cf.log.add_config(mot_req)
    # mot_req.data_received_cb.add_callback(MotReq_cbk)
    # mot_req.start()
    
    # scf.cf.log.add_config(mot_set)
    # mot_set.data_received_cb.add_callback(MotSet_cbk)
    # mot_set.start()

def StabZ_cbk(timestamp, data, logconf):
    '''Callback function to decode position data from stateEstimateZ group'''

    # state_meas[0] = data['stateEstimateZ.x']/1000
    # state_meas[1] = data['stateEstimateZ.y']/1000
    # state_meas[2] = data['stateEstimateZ.z']/1000
    
    # state_meas[3] = 1#data['stateEstimateZ.q']
    # state_meas[4] = 0#data['stateEstimateZ.q']
    # state_meas[5] = 0#data['stateEstimateZ.q']
    # state_meas[6] = 0#data['stateEstimateZ.q']

    state_meas[7] = data['stateEstimateZ.vx']/1000
    state_meas[8] = data['stateEstimateZ.vy']/1000
    state_meas[9] = data['stateEstimateZ.vz']/1000
    
    state_meas[10] = data['stateEstimateZ.rateRoll']/1000
    state_meas[11] = data['stateEstimateZ.ratePitch']/1000
    state_meas[12] = data['stateEstimateZ.rateYaw']/1000
    
    # q_d = np.zeros(4)
    #q = np.uint32(data['stateEstimateZ.quat'])
    #quatdecompress(q, q_d)
    #print(f'ST  pos:{Meas_St[0]}, {Meas_St[1]}, {Meas_St[2]}')

def StabPos_cbk(timestamp, data, logconf):
    '''Callback function to decode attitude from stateEstimate group'''
    state_meas[0] = data['stateEstimate.x']
    state_meas[1] = data['stateEstimate.y']
    state_meas[2] = data['stateEstimate.z']

def StabAtt_cbk(timestamp, data, logconf):  
    state_meas[3] = data['stateEstimate.qw']
    state_meas[4] = data['stateEstimate.qx']
    state_meas[5] = data['stateEstimate.qy']
    state_meas[6] = data['stateEstimate.qz'] 

# def StabLVel_cbk(timestamp, data, logconf):    
#     state_meas[7] = data['stateEstimate.vx']
#     state_meas[8] = data['stateEstimate.vy']
#     state_meas[9] = data['stateEstimate.vz']
    
# def StabAVel_cbk(timestamp, data, logconf):
#     state_meas[10] = data['stateEstimate.rateRoll']
#     state_meas[11] = data['stateEstimate.ratePitch']
#     state_meas[12] = data['stateEstimate.rateYaw']
    
    #print(f'ST att: {Meas_St[3]}, {Meas_St[4]}, {Meas_St[5]}, {Meas_St[6]}\n')
    
# def MotReq_cbk(timestamp, data, logconf):
#     '''Callback function to decode rates from stateEstimateZ group'''

#     pwm_req[0] = data['motor.m1req']
#     pwm_req[1] = data['motor.m2req']
#     pwm_req[2] = data['motor.m3req']
#     pwm_req[3] = data['motor.m4req']

#     print(f"DEBUG req: {pwm_req}")
    
# def MotSet_cbk(timestamp, data, logconf):
#     '''Callback function to decode rates from stateEstimateZ group'''

#     pwm_set[0] = data['motor.m1']
#     pwm_set[1] = data['motor.m2']
#     pwm_set[2] = data['motor.m3']
#     pwm_set[3] = data['motor.m4']
    
#     print(f"DEBUG set: {pwm_set}")