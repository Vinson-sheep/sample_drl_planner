#! /usr/bin/env python
#-*- coding: UTF-8 -*- 

import math
import rospy
from uav_simulator.srv import *
from uav_simulator.msg import *
from my_simple_planner.msg import *
import numpy as np
import os
import pickle

# import DDPG
# import TD3
import SAC

# Test param
kLoadProgress = True

# DRL param
kPolicy = "SAC" # DDPG or TD3 or SAC
kStateDim = 44
kActionDim = 2
kMaxEpisode = 1000
kMaxStepSize = 100
# uav param
kMaxLinearVelicity = 1.0
kMaxAngularVeclity = 1.0
kStepTime = 0.1
# map param
kTargetDistance = 5
kSafeRadius = 0.5
kLengthX = 15
kLengthY = 15
kNumObsMax = 20
kNumObsMin = 5
kRadiusObsMax = 2.0
kRadiusObsMin = 0.25
kCrashLimit = 0.25
kArriveLimit = 0.25
KIntegrateDt = 0.02
kAccelerateRate = 1.0
# sensor param
kRangeMax = 5.0
kRangeMin = 0.15
kAngleMax = 2 * math.pi / 3
kAngleMin = -2 * math.pi / 3
kNumLaser = 40

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

# if (policy == "TD3"):
#     agent = TD3.TD3(**kwargs)
# if (policy == "DDPG"):
#     agent = DDPG.DDPG(**kwargs)
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

def GetStateVector(state):
    _result = list()
    # add obstacle info
    for i in range(0, len(state.scan.ranges)):
        interval_length = state.scan.range_max - state.scan.range_min
        _result.append((state.scan.ranges[i] - interval_length/2) / (interval_length/2))
    # add velocity info
    _linear_velocity = ((state.twist.linear.x)**2 + (state.twist.linear.y)**2)**(0.5)
    _linear_velocity = (_linear_velocity - kMaxLinearVelicity/2) / (kMaxLinearVelicity/2)
    _result.append(_linear_velocity)
    _angle_velicity = state.twist.angular.z / kMaxAngularVeclity
    _result.append(_angle_velicity)
    # add target info
    _target_distance = (state.target_distance - kTargetDistance/2) / (kTargetDistance/2)
    _result.append(_target_distance)
    _target_angle = state.target_angle / math.pi
    _result.append(_target_angle)

    return np.array(_result)

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
    _set_uav_pose_client = rospy.ServiceProxy("set_uav_pose", SetUavPose)

    _reward_publisher = rospy.Publisher("reward", Reward, queue_size=1)
    _control_publisher = rospy.Publisher("control", Control, queue_size=1)
    _state_vector_publisher = rospy.Publisher("state_vector", StateVector, queue_size=1)

    print("Wait for /reset_map service.")

    # wait for service
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

        _reset_map_req.sparam.angle_max = kAngleMax
        _reset_map_req.sparam.angle_min = kAngleMin
        _reset_map_req.sparam.num_laser = kNumLaser
        _reset_map_req.sparam.range_max = kRangeMax
        _reset_map_req.sparam.range_min = kRangeMin

        _reset_map_req.param.arrive_limit = kArriveLimit
        _reset_map_req.param.crash_limit = kCrashLimit
        _reset_map_req.param.intergrate_dt = KIntegrateDt
        _reset_map_req.param.accelerate_rate = kAccelerateRate
        _reset_map_req.param.target_distance = kTargetDistance
        _reset_map_req.param.safe_radius = kSafeRadius
        _reset_map_req.param.length_x = kLengthX
        _reset_map_req.param.length_y = kLengthY
        _reset_map_req.param.num_obs_max = kNumObsMax
        _reset_map_req.param.num_obs_min = kNumObsMin
        _reset_map_req.param.radius_obs_max = kRadiusObsMax
        _reset_map_req.param.radius_obs_min = kRadiusObsMin
        resp = _reset_map_client.call(_reset_map_req)

        _s0 = resp.state
        _s0_vector = GetStateVector(_s0)

        for step in range(kMaxStepSize):

            # choose action
            _a0_vector = agent.act(_s0_vector)

            # get rid of trap
            # if IsInTrap(_s0): break

            # DEBUG
            _control_msg = Control()
            _control_msg.linear_velocity = (_a0_vector[0] + 1) / 2 * kMaxLinearVelicity
            _control_msg.yaw_rate = _a0_vector[1] * kMaxAngularVeclity
            _control_publisher.publish(_control_msg)

            _state_vector_msg = StateVector()
            _state_vector_msg.header.stamp = rospy.Time.now()
            _state_vector_msg.data = list(_s0_vector)
            _state_vector_publisher.publish(_state_vector_msg)

            # step
            _step_req = StepRequest()
            _step_req.control = _control_msg
            _step_req.step_time = kStepTime
            _step_resp = _step_client.call(_step_req)

            _s1 = _step_resp.state
            _s1_vector = GetStateVector(_s1)  

            _is_crash = _step_resp.is_crash
            _is_arrive = _step_resp.is_arrive

            _done = (_is_arrive or _is_crash)


            if _is_crash == True:
                    crash_num += 1
                    print("Crashed!")
            elif _is_arrive == True:
                    success_num += 1
                    print("Arrived")

            # other
            _s0 = _s1
            _s0_vector = _s1_vector

            if _done: break
            if rospy.is_shutdown(): break

            # if uav is out of range
            if (_s0.pose.position.x < -kLengthX/2 or _s0.pose.position.x > kLengthX/2):
                break
            if (_s0.pose.position.y < -kLengthY/2 or _s0.pose.position.y > kLengthY/2):
                break

        print('[' + str(episode+1) + '] success_num = %d, crash_num = %d' %(success_num, crash_num))

        if rospy.is_shutdown(): break

        save(episode, success_num, crash_num)

    rospy.spin()