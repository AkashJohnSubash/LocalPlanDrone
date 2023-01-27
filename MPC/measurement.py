from time import time, sleep
import numpy as np
import sys

from cflib.crazyflie.log import LogConfig
from threading import Event


deck_attached_event = Event()
state_meas =  np.zeros(13)
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

    # Log data from the CF stabilizer via Radio
    stab_pos = LogConfig(name='StabZ', period_in_ms=100)
    stab_pos.add_variable('stateEstimateZ.x', 'int16_t')
    stab_pos.add_variable('stateEstimateZ.y', 'int16_t')
    stab_pos.add_variable('stateEstimateZ.z', 'int16_t')
    stab_pos.add_variable('stateEstimateZ.vx', 'int16_t')
    stab_pos.add_variable('stateEstimateZ.vy', 'int16_t')
    stab_pos.add_variable('stateEstimateZ.vz', 'int16_t')
    stab_pos.add_variable('stateEstimateZ.rateRoll', 'int16_t')
    stab_pos.add_variable('stateEstimateZ.ratePitch', 'int16_t')
    stab_pos.add_variable('stateEstimateZ.rateYaw', 'int16_t')
    #stab_pos.add_variable('stateEstimateZ.quat', 'float')

    stab_att = LogConfig(name='StabAtt', period_in_ms=100)
    stab_att.add_variable('stateEstimate.qw', 'float')
    stab_att.add_variable('stateEstimate.qx', 'float')
    stab_att.add_variable('stateEstimate.qy', 'float')
    stab_att.add_variable('stateEstimate.qz', 'float')

    # stab_vel = LogConfig(name='StabVel', period_in_ms=100)
    # stab_vel.add_variable('stateEstimateZ.vx', 'int16_t')
    # stab_vel.add_variable('stateEstimateZ.vy', 'int16_t')
    # stab_vel.add_variable('stateEstimateZ.vz', 'int16_t')
    # stab_vel.add_variable('stateEstimateZ.rateRoll', 'int16_t')
    # stab_vel.add_variable('stateEstimateZ.ratePitch', 'int16_t')
    # stab_vel.add_variable('stateEstimateZ.rateYaw', 'int16_t')

    mot_set = LogConfig(name='MotSet', period_in_ms=100)
    mot_set.add_variable('motor.m1', 'int32_t')
    mot_set.add_variable('motor.m2', 'int32_t')
    mot_set.add_variable('motor.m3', 'int32_t')
    mot_set.add_variable('motor.m4', 'int32_t')

    mot_req = LogConfig(name='MotReq', period_in_ms=100)
    mot_req.add_variable('motor.m1req', 'int32_t')
    mot_req.add_variable('motor.m2req', 'int32_t')
    mot_req.add_variable('motor.m3req', 'int32_t')
    mot_req.add_variable('motor.m4req', 'int32_t')

    scf.cf.log.add_config(stab_pos)
    stab_pos.data_received_cb.add_callback(StabZ_cbk)
    stab_pos.start()
    
    #scf.cf.log.add_config(stab_att)
    #stab_att.data_received_cb.add_callback(StabAtt_cbk)
    #stab_att.start()

    #scf.cf.log.add_config(stab_vel)
    #stab_vel.data_received_cb.add_callback(StabVel_cbk)
    #stab_vel.start()

    #scf.cf.log.add_config(mot_req)
    #mot_req.data_received_cb.add_callback(MotReq_cbk)
    #mot_req.start()
    
    #scf.cf.log.add_config(mot_set)
    #mot_set.data_received_cb.add_callback(MotSet_cbk)
    #mot_set.start()

def StabZ_cbk(timestamp, data, logconf):
    '''Callback function to decode position data from stateEstimateZ group'''

    # q_d = np.zeros(4)
    state_meas[0] = data['stateEstimateZ.x']/1000
    state_meas[1] = data['stateEstimateZ.y']/1000
    state_meas[2] = data['stateEstimateZ.z']/1000
    
    state_meas[7] = data['stateEstimateZ.vx']/1000
    state_meas[8] = data['stateEstimateZ.vy']/1000
    state_meas[9] = data['stateEstimateZ.vz']/1000
    
    state_meas[10] = data['stateEstimateZ.rateRoll']/1000
    state_meas[11] = data['stateEstimateZ.ratePitch']/1000
    state_meas[12] = data['stateEstimateZ.rateYaw']/1000
    
    #q = np.uint32(data['stateEstimateZ.quat'])
    #quatdecompress(q, q_d)
    #print(f'ST  pos:{Meas_St[0]}, {Meas_St[1]}, {Meas_St[2]}')

def StabAtt_cbk(timestamp, data, logconf):
    '''Callback function to decode attitude from stateEstimate group'''

    state_meas[3] = np.around(data['stateEstimate.qw'], 5)
    state_meas[4] = np.around(data['stateEstimate.qx'], 5)
    state_meas[5] = np.around(data['stateEstimate.qy'], 5)
    state_meas[6] = np.around(data['stateEstimate.qz'], 5) 
    
    #print(f'ST att: {Meas_St[3]}, {Meas_St[4]}, {Meas_St[5]}, {Meas_St[6]}\n')

# def StabVel_cbk(timestamp, data, logconf):
#     '''Callback function to decode rates from stateEstimateZ group'''

#     state_meas[7] = data['stateEstimateZ.vx']/1000
#     state_meas[8] = data['stateEstimateZ.vy']/1000
#     state_meas[9] = data['stateEstimateZ.vz']/1000
    
#     state_meas[10] = data['stateEstimateZ.rateRoll']/1000
#     state_meas[11] = data['stateEstimateZ.ratePitch']/1000
#     state_meas[12] = data['stateEstimateZ.rateYaw']/1000
    
def MotReq_cbk(timestamp, data, logconf):
    '''Callback function to decode rates from stateEstimateZ group'''

    pwm_req[0] = data['motor.m1req']
    pwm_req[1] = data['motor.m2req']
    pwm_req[2] = data['motor.m3req']
    #pwm_req[3] = data['motor.m4req']
    
def MotSet_cbk(timestamp, data, logconf):
    '''Callback function to decode rates from stateEstimateZ group'''

    pwm_set[0] = data['motor.m1']
    pwm_set[1] = data['motor.m2']
    pwm_set[2] = data['motor.m3']
    #pwm_set[3] = data['motor.m4']
    
# def get_state(scf):

#     # scf.cf.log.add_config(stab_vel)
#     # stab_vel.data_received_cb.add_callback(StabVel_cbk)
#     # stab_vel.start()