#! /usr/bin/env python
#-*- coding: UTF-8 -*- 

import rospy
from my_simple_planner.srv import *
import SAC

kPolicy = "SAC"

agent = None

kwargs = {
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

def GetActionCB(req):
    resp = GetActionResponse() 
    resp.action  = agent.act(req.state)
    resp.success = True
    return resp

if __name__ == '__main__':

    # initialize ros
    rospy.init_node("local_planner")
    
    # service
    _get_action_servicer = rospy.Service("get_action", GetAction, GetActionCB)

    rospy.spin()
