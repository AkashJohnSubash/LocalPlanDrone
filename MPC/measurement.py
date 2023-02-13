from time import time, sleep
import numpy as np
import sys

from cflib.crazyflie.log import LogConfig
from threading import Event

from common import init_st, quatDecompress

deck_attached_event = Event()
state_meas =  np.zeros(13)
set_rpyt = np.zeros(4)
req_rpyt = np.zeros(4)
att_quat = np.zeros(4)
pwm_set =  np.zeros(4)
pwm_req =  np.zeros(4)
att_eul = np.zeros(3)

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

    '''Define variables in group'''

    
    # stabPos = LogConfig(name='StabPos', period_in_ms=100)
    # stabPos.add_variable('stateEstimate.x', 'float')
    # stabPos.add_variable('stateEstimate.y', 'float')
    # stabPos.add_variable('stateEstimate.z', 'float')
    
    # Kal = LogConfig(name='Kalman', period_in_ms=100)
    # Kal.add_variable('kalman.stateX', 'float')
    # Kal.add_variable('kalman.stateY', 'float')
    # Kal.add_variable('kalman.stateZ', 'float')
    
    stabAttq = LogConfig(name='StabAtt', period_in_ms=50)
    stabAttq.add_variable('stateEstimate.qw', 'float')
    stabAttq.add_variable('stateEstimate.qx', 'float')
    stabAttq.add_variable('stateEstimate.qy', 'float')
    stabAttq.add_variable('stateEstimate.qz', 'float')

    # stabAtte = LogConfig(name='StabAtt', period_in_ms=50)
    # stabAtte.add_variable('stateEstimate.roll', 'float')
    # stabAtte.add_variable('stateEstimate.pitch', 'float')
    # stabAtte.add_variable('stateEstimate.yaw', 'float')
    
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

    
    # ctrl_req = LogConfig(name='CtrlReq', period_in_ms=20)
    # ctrl_req.add_variable('controller.roll', 'float')
    # ctrl_req.add_variable('controller.pitch', 'float')
    # ctrl_req.add_variable('controller.yawRate', 'float')
    # ctrl_req.add_variable('controller.actuatorThrust', 'float')

    # ctrl_set = LogConfig(name='CtrlSet', period_in_ms=20)
    # ctrl_set.add_variable('controller.cmd_roll', 'float')
    # ctrl_set.add_variable('controller.cmd_pitch', 'float')
    # ctrl_set.add_variable('controller.cmd_yaw', 'float')
    # ctrl_set.add_variable('controller.cmd_thrust', 'float')
    # ctrl_set.add_variable('controller.r_roll', 'float')
    # ctrl_set.add_variable('controller.r_pitch', 'float')

    '''register callbacks for group'''

    # scf.cf.log.add_config(stabPos)
    # stabPos.data_received_cb.add_callback(StabPos_cbk)
    # stabPos.start()

    # scf.cf.log.add_config(Kal)
    # Kal.data_received_cb.add_callback(kalman_cbk)
    # Kal.start()
    
    # scf.cf.log.add_config(stabAttq)
    # stabAttq.data_received_cb.add_callback(StabAttq_cbk)
    # stabAttq.start()

    # scf.cf.log.add_config(stabAtte)
    # stabAtte.data_received_cb.add_callback(StabAtte_cbk)
    # stabAtte.start()


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

    # scf.cf.log.add_config(ctrl_req)
    # ctrl_req.data_received_cb.add_callback(CtrlPidReq_cbk)
    # ctrl_req.start()
    
    # scf.cf.log.add_config(ctrl_set)
    # ctrl_set.data_received_cb.add_callback(CtrlPidSet_cbk)
    # ctrl_set.start()

    #Log data from the CF stabilizer via Radio
    stabZ = LogConfig(name='StabZ', period_in_ms=50)
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

    scf.cf.log.add_config(stabZ)
    stabZ.data_received_cb.add_callback(StabZ_cbk)
    stabZ.start()
    

''''Deine Callback functions below'''

def StabZ_cbk(timestamp, data, logconf):
    '''Callback function to decode position data from stateEstimateZ group'''

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
    # q_d = np.zeros(4)
    #q = np.uint32(data['stateEstimateZ.quat'])
    #quatdecompress(q, q_d)
    #print(f'ST  pos:{state_meas[0]}, {state_meas[1]}, {state_meas[2]}')

# def StabPos_cbk(timestamp, data, logconf):
#     '''Callback function to decode attitude from stateEstimate group'''
#     state_meas[0] = data['stateEstimate.x']
#     state_meas[1] = data['stateEstimate.y']
#     state_meas[2] = data['stateEstimate.z']

# def kalman_cbk(timestamp, data, logconf):
#     '''Callback function to decode attitude from stateEstimate group'''
#     state_meas[0] = data['kalman.stateX']
#     state_meas[1] = data['kalman.stateY']
#     state_meas[2] = data['kalman.stateZ']
#     print(f'Kalman  pos:{state_meas[0]}, {state_meas[1]}, {state_meas[2]}')


def StabAttq_cbk(timestamp, data, logconf):  
    att_quat[0] = data['stateEstimate.qx']
    att_quat[1] = data['stateEstimate.qy']
    att_quat[2] = data['stateEstimate.qz']
    att_quat[3] = data['stateEstimate.qw'] 
    #print("quat",np.round(np.array([state_meas[3], state_meas[4], state_meas[5], state_meas[6]]), 5))

# def StabAtte_cbk(timestamp, data, logconf):  
#     att_eul[0] = data['stateEstimate.roll']
#     att_eul[1] = data['stateEstimate.pitch']
#     att_eul[2] = data['stateEstimate.yaw']
#     #print("eul", np.round(np.array([att_eul[0], att_eul[1], att_eul[2]]),5))

# def CtrlPidReq_cbk(timestamp, data, logconf):
#     req_rpyt[0] = data['controller.roll']
#     req_rpyt[1] = data['controller.pitch']
#     req_rpyt[2] = data['controller.yawRate']
#     req_rpyt[3] = data['controller.actuatorThrust']
#     print("REQ RPYT :", req_rpyt)

# def CtrlPidSet_cbk(timestamp, data, logconf):
#     set_rpyt[0] = data['controller.cmd_roll']
#     set_rpyt[1] = data['controller.cmd_pitch']
#     set_rpyt[2] = data['controller.cmd_yaw']
#     set_rpyt[3] = data['controller.cmd_thrust']
#     print("SET RPYT :", set_rpyt)

# def StabLVel_cbk(timestamp, data, logconf):    
#     state_meas[7] = data['stateEstimate.vx']
#     state_meas[8] = data['stateEstimate.vy']
#     state_meas[9] = data['stateEstimate.vz']
    
# def StabAVel_cbk(timestamp, data, logconf):
#     state_meas[10] = data['stateEstimate.rateRoll']
#     state_meas[11] = data['stateEstimate.ratePitch']
#     state_meas[12] = data['stateEstimate.rateYaw']
    
#     #print(f'ST att: {Meas_St[3]}, {Meas_St[4]}, {Meas_St[5]}, {Meas_St[6]}\n')
    
# def MotReq_cbk(timestamp, data, logconf):
#     '''Callback function to decode rates from stateEstimateZ group'''

#     pwm_req[0] = data['motor.m1req']
#     pwm_req[1] = data['motor.m2req']
#     pwm_req[2] = data['motor.m3req']
#     pwm_req[3] = data['motor.m4req']

#     #print(f"DEBUG req: {pwm_req}")
    
# def MotSet_cbk(timestamp, data, logconf):
#     '''Callback function to decode rates from stateEstimateZ group'''

#     pwm_set[0] = data['motor.m1']
#     pwm_set[1] = data['motor.m2']
#     pwm_set[2] = data['motor.m3']
#     pwm_set[3] = data['motor.m4']
    
# #     print(f"DEBUG set: {pwm_set}")