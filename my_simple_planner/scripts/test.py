#! /usr/bin/env python
#-*- coding: UTF-8 -*- 

import math
import rospy
from uav_simulator.srv import *
from uav_simulator.msg import *
import numpy as np
import os
import pickle

import SAC

# Test param
kLoadProgress = False

# DRL param
kPolicy = "SAC"
kStateDim = 52
kActionDim = 2
kMaxEpisode = 1000
kMaxStepSize = 100
# uav param
kMaxLinearVelocity = 1.0
kMaxAngularVelocity = 1.0
kStepTime = 0.1

# variable
episode_begin = 0
success_num = 0
crash_num = 0
agent = None

# file url
url = os.path.dirname(os.path.realpath(__file__)) + '/data/'

# initialize agent
kwargs = {
    'state_dim': kStateDim,
    'action_dim': kActionDim,
    'load_buffer_flag': False,
    'load_actor_flag': True,
    'load_critic_flag': False,
    'load_log_alpha_flag': False,
    'load_optim_flag': False,
    'fix_actor_flag': True,
    'use_priority': False
}

if (kPolicy == "SAC"):
    agent = SAC.SAC(**kwargs)

def save(episode, success_num, crash_num):
    save_file = open(url + 'temp_test.bin',"wb")
    pickle.dump(episode,save_file)
    pickle.dump(success_num,save_file)
    pickle.dump(crash_num, save_file)
    save_file.close()
    
def load():
    load_file = open(url + 'temp_test.bin',"rb")
    episode=pickle.load(load_file)
    success_num=pickle.load(load_file)
    crash_num = pickle.load(load_file)
    print("Restore episode = %d, success num = %d, crash num = %d." % (episode+1, success_num, crash_num))
    return episode, success_num, crash_num

#
uav_pos_queue = []

def IsInTrap(new_state):
    global uav_pos_queue
    if len(uav_pos_queue ) > 5: uav_pos_queue  = uav_pos_queue[1:]
    uav_pos_queue .append([new_state.pose.position.x, new_state.pose.position.y,new_state.pose.position.z])
    if (len(uav_pos_queue) < 5): return False
    _std = np.std(np.array(uav_pos_queue), 0)
    _limit = 0.02
    return _std[0] < _limit and _std[1] < _limit and _std[2] < _limit

if __name__ == '__main__':

    # initialize ros
    rospy.init_node("testing_node")

    # raw data
    _reset_map_client = rospy.ServiceProxy("reset_map", ResetMap)
    _step_client = rospy.ServiceProxy("step", Step)

    _control_publisher = rospy.Publisher("control", Control, queue_size=1)
    _state_publisher = rospy.Publisher("state", StateStamped, queue_size=1)
    _state_vector_publisher = rospy.Publisher("state_vector", StateVectorStamped, queue_size=1)

    # wait for service
    print("Wait for /reset_map service.")
    _reset_map_client.wait_for_service()
    print("/reset_map service is available.")

    # load data if true
    if kLoadProgress:
        episode_begin, success_num, crash_num = load()
        episode_begin += 1

    # start to test
    for episode in range(episode_begin, kMaxEpisode):

        print("=====================================")
        print("=========== Episode %d ===============" % (episode))
        print("=====================================")

        # reset world
        _reset_map_req = ResetMapRequest()
        resp = _reset_map_client.call(_reset_map_req)

        _s0 = resp.state
        _s0_vector = resp.state_vector

        for step in range(kMaxStepSize):

            # choose action
            _a0_vector = agent.act(_s0_vector)

            # get rid of trap
            # if IsInTrap(_s0): break

            # publish control
            _control_msg = Control()
            _control_msg.linear_velocity = (_a0_vector[0] + 1) / 2 * kMaxLinearVelocity
            _control_msg.yaw_rate = _a0_vector[1] * kMaxAngularVelocity
            _control_publisher.publish(_control_msg)
            # publish state vector
            _state_vector_msg = StateVectorStamped()
            _state_vector_msg.header.stamp = rospy.Time.now()
            _state_vector_msg.data = list(_s0_vector)
            _state_vector_publisher.publish(_state_vector_msg)
            # publish state
            _state_msg = StateStamped()
            _state_msg.header.stamp  = rospy.Time.now()
            _state_msg.state  = _s0
            _state_publisher.publish(_state_msg)

            # step
            _step_req = StepRequest()
            _step_req.control = _control_msg
            _step_req.step_time = kStepTime
            _step_resp = _step_client.call(_step_req)

            _s1 = _step_resp.state
            _s1_vector = _step_resp.state_vector

            _is_crash = _step_resp.is_crash
            _is_arrive = _step_resp.is_arrive
            _is_out_range = _step_resp.is_out_range

            _done = (_is_arrive or _is_crash)

            # other
            _s0 = _s1
            _s0_vector = _s1_vector

            if _is_crash == True:
                    crash_num += 1
                    print("Crashed!")
            elif _is_arrive == True:
                    success_num += 1
                    print("Arrived")

            # other
            if _done: break
            if _is_out_range: break
            if rospy.is_shutdown(): break

        print('[' + str(episode+1) + '] success_num = %d, crash_num = %d' %(success_num, crash_num))

        if rospy.is_shutdown(): break

        save(episode, success_num, crash_num)

    rospy.spin()