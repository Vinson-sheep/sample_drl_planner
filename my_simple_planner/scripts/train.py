#! /usr/bin/env python
#-*- coding: UTF-8 -*- 

import math
import rospy
from uav_simulator.srv import *
from uav_simulator.msg import *
from my_simple_planner.msg import *
import numpy as np
import os
import threading
from tensorboardX import SummaryWriter

import SAC

# Train param
kLoadPorgress = True
kLoadBuffer = True
kLoadActor = True
kLoadCritic = True
kLoadLogAlpha = True
kLoadOptim = True

kFixActorFlag = False
kUsePriority = True

# DRL param
kPolicy = "SAC"
kStateDim = 44
kActionDim = 2
kMaxEpisode = 700
kMaxStepSize = 300
kInitEpisode = 5
K = 1
# uav param
kMaxLinearVelicity = 1.0
kMaxAngularVeclity = 1.0
kStepTime = 0.2
# map param
kTargetDistance = 8
kSafeRadius = 0.5
kLengthX = 15
kLengthY = 15
kNumObsMax = 10
kNUmObsMin = 5
kRadiusObsMax = 2.0
kRadiusObsMin = 0.25
kCrashLimit = 0.25
kArriveLimit = 0.25
KIntegrateDt = 0.02
# sensor param
kRangeMax = 5.0
kRangeMin = 0.15
kAngleMax = 2 * math.pi / 3
kAngleMin = -2 * math.pi / 3
kNumLaser = 40

# variable
episode_rewards = np.array([])
episode_times = np.array([])
step_rewards = np.array([])
actor_losses = np.array([])
critic_losses = np.array([])
alpha_losses = np.array([])
alphas = np.array([])
agent = None

url = os.path.dirname(os.path.realpath(__file__)) + '/data/'
writer = SummaryWriter(url + '../../log')

# initialize agent
kwargs = {
    'state_dim': kStateDim,
    'action_dim': kActionDim,
    'load_buffer_flag': kLoadBuffer,
    'load_actor_flag': kLoadActor,
    'load_critic_flag': kLoadCritic,
    'load_log_alpha_flag': kLoadLogAlpha,
    'load_optim_flag': kLoadOptim,
    'fix_actor_flag': kFixActorFlag,
    'use_priority': kUsePriority
}

if (kPolicy == "SAC"):
    agent = SAC.SAC(**kwargs)

class saveThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):

        print("*Saving. Please don't close the window!")

        begin_time = rospy.Time.now()
        # save data
        np.save(url + "episode_rewards.npy", episode_rewards)
        np.save(url + "episode_times.npy", episode_times)
        np.save(url + "step_rewards.npy", step_rewards)
        np.save(url + "actor_losses.npy", actor_losses)
        np.save(url + "critic_losses.npy", critic_losses)
        if kPolicy == "SAC": np.save(url + "alpha_losses.npy", alpha_losses)
        if kPolicy == "SAC": np.save(url + "alphas.npy", alphas)
        # save model
        agent.save()
        # print
        save_time = (rospy.Time.now() - begin_time).to_sec()
        writer.add_scalar("DEBUG/save_time", save_time, global_step=episode_rewards.size-1)  
        
        print("Saved. Time consumed = %f seconds." % (save_time))


class learnThread(threading.Thread):
    
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):

        global actor_losses
        global critic_losses
        global alpha_losses
        global alphas

        # agent learn
        begin_time = rospy.Time.now()
        for i in range(K): agent.update()
        learn_time = (rospy.Time.now() - begin_time).to_sec()
        # log
        actor_losses = np.append(actor_losses, agent.actor_loss)
        writer.add_scalar("Loss/actor_loss", agent.actor_loss, global_step=actor_losses.size-1) 
        critic_losses = np.append(critic_losses, agent.critic_loss)
        writer.add_scalar("Loss/critic_loss", agent.critic_loss, global_step=critic_losses.size-1) 
        if kPolicy == "SAC":
            alpha_losses = np.append(alpha_losses, agent.alpha_loss)
            writer.add_scalar("Loss/alpha_loss", agent.alpha_loss, global_step=alpha_losses.size-1) 
            alphas = np.append(alphas, agent.alpha.item())
            writer.add_scalar("Loss/alpha", agent.alpha.item(), global_step=alphas.size-1) 

        if step_rewards.size % 100 == 0:
            print("Learned. Time consumed = %f seconds." % (learn_time))


def loadData():

    global episode_rewards
    global episode_times
    global step_rewards
    global actor_losses
    global critic_losses
    global alpha_losses
    global alphas
    episode_rewards = np.load(url + "episode_rewards.npy")
    episode_times = np.load(url + "episode_times.npy")
    step_rewards = np.load(url + "step_rewards.npy")
    actor_losses = np.load(url + "actor_losses.npy")
    critic_losses = np.load(url + "critic_losses.npy")

    for i in range(episode_rewards.size): writer.add_scalar("Performance/episode_reward", episode_rewards[i], global_step=i)  
    for i in range(episode_times.size): writer.add_scalar("Performance/episode_time", episode_times[i], global_step=i)  
    for i in range(step_rewards.size): writer.add_scalar("Performance/step_reward", step_rewards[i], global_step=i)  
    for i in range(actor_losses.size): writer.add_scalar("Loss/actor_loss", actor_losses[i], global_step=i)  
    for i in range(critic_losses.size): writer.add_scalar("Loss/critic_loss", critic_losses[i], global_step=i)  

    if kPolicy == "SAC": 
        alpha_losses = np.load(url + "alpha_losses.npy")
        alphas = np.load(url + "alphas.npy")

        for i in range(alpha_losses.size):  writer.add_scalar("Loss/alpha_loss", alpha_losses[i], global_step=i) 
        for i in range(alphas.size):  writer.add_scalar("Loss/alpha", alphas[i], global_step=i) 

    print("1. Restore episode: %d" % (episode_rewards.size))
    print("2. Restore step: %d" % (step_rewards.size))
    

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
    _target_distance = (state.target_distance - kTargetDistance/2) / kTargetDistance/2
    _result.append(_target_distance)
    _target_angle = (state.target_angle - math.pi/2) / (math.pi/2)
    _result.append(_target_angle)

    return np.array(_result)

def GetReward(cur_state, next_state, dt, step_count, is_arrival, is_crash):
    # distance reward
    _distance_reward = (cur_state.target_distance - next_state.target_distance)*(5 / dt)*1.6*7 
    # arrival reward
    _arrival_reward = 0
    if (is_arrival): _arrival_reward = 100
    # crash reward
    _crash_reward = 0
    if (is_crash): _crash_reward = -100
    # laser reward
    _ranges = (np.array(next_state.scan.ranges) - next_state.scan.range_min) / (next_state.scan.range_max - next_state.scan.range_min)
    _ranges = -1.0* np.exp(-_ranges / 0.1)
    _laser_reward = max(np.sum(_ranges), -100)    
    # step reward
    _step_reward = -step_count * 0.04
    # total reward
    _total_reward = _distance_reward + _arrival_reward + _crash_reward + _laser_reward + _step_reward

    _reward_msg = Reward()
    _reward_msg.distance_reward = _distance_reward
    _reward_msg.arrive_reward = _arrival_reward
    _reward_msg.crash_reward = _crash_reward
    _reward_msg.laser_reward = _laser_reward
    _reward_msg.step_punish_reward = _step_reward
    _reward_msg.total_reward = _total_reward

    return _reward_msg

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
    rospy.init_node("training_node")

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
    if kLoadPorgress: loadData()

    episode_begin = episode_rewards.size

    # start to train
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
        _reset_map_req.param.target_distance = kTargetDistance
        _reset_map_req.param.safe_radius = kSafeRadius
        _reset_map_req.param.length_x = kLengthX
        _reset_map_req.param.length_y = kLengthY
        _reset_map_req.param.num_obs_max = kNumObsMax
        _reset_map_req.param.num_obs_min = kNUmObsMin
        _reset_map_req.param.radius_obs_max = kRadiusObsMax
        _reset_map_req.param.radius_obs_min = kRadiusObsMin
        resp = _reset_map_client.call(_reset_map_req)

        _s0 = resp.state
        _s0_vector = GetStateVector(_s0)

        episode_reward = 0
        episode_begin_time = rospy.Time.now()

        for step in range(kMaxStepSize):

            step_begin_time = rospy.Time.now()

            # choose action
            _a0_vector = agent.act(_s0_vector)

            # get rid of trap
            if IsInTrap(_s0): _a0_vector[0] += 0.2

            # DEBUG
            _control_msg = Control()
            _control_msg.linear_velocity = (_a0_vector[0] + 1) / 2 * kMaxLinearVelicity
            _control_msg.yaw_rate = _a0_vector[1] * kMaxAngularVeclity
            _control_publisher.publish(_control_msg)

            _state_vector_msg = StateVector()
            _state_vector_msg.header.stamp = rospy.Time.now()
            _state_vector_msg.data = list(_s0_vector)
            _state_vector_publisher.publish(_state_vector_msg)

            # agent learn
            if episode < kInitEpisode: agent.fix_actor_flag = True
            else: agent.fix_actor_flag = False
            
            learnThread().start()

            # step
            _step_req = StepRequest()
            _step_req.control = _control_msg
            _step_req.step_time = kStepTime
            _step_resp = _step_client.call(_step_req)

            _s1 = _step_resp.state
            _s1_vector = GetStateVector(_s1)  

            _is_crash = _step_resp.is_crash
            _is_arrive = _step_resp.is_arrive

            _r1_msg = GetReward(_s0, _s1, kStepTime, step, _is_arrive, _is_crash)
            _reward_publisher.publish(_r1_msg)
            _r1 =  _r1_msg.total_reward

            _done = (_is_arrive or _is_crash)

            # save transition
            agent.put(_s0_vector, _a0_vector, _r1, _s1_vector, _done)

            # plot and save
            step_rewards = np.append(step_rewards, _r1)
            writer.add_scalar("Performance/step_reward", _r1, global_step=step_rewards.size-1)  
            writer.add_scalar("DEBUG/step_time", (rospy.Time.now() - step_begin_time).to_sec(), global_step=step_rewards.size-1)  

            # other
            episode_reward += _r1
            _s0 = _s1
            _s0_vector = _s1_vector

            if _done: break
            if rospy.is_shutdown(): break

            # if uav is out of range
            if (_s0.pose.position.x < -kLengthX/2 or _s0.pose.position.x > kLengthX/2):
                break
            if (_s0.pose.position.y < -kLengthY/2 or _s0.pose.position.y > kLengthY/2):
                break

        episode_time = (rospy.Time.now() - episode_begin_time).to_sec()
        episode_rewards = np.append(episode_rewards, episode_reward)
        episode_times = np.append(episode_times, episode_time)
        writer.add_scalar("Performance/episode_reward", episode_reward, global_step=episode_rewards.size-1)  
        writer.add_scalar("Performance/episode_time", episode_time, global_step=episode_times.size-1)  

        if rospy.is_shutdown(): break

        saveThread().start()

    rospy.spin()