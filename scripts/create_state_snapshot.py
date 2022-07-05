#!/usr/bin/env python

## A script that creates state snapshot for simulated world in Gazebo.
# @ingroup utilities
# @file create_state_snapshot.py

# Copyright (c) 2021, Robot Control and Pattern Recognition Group,
# Institute of Control and Computation Engineering
# Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Dawid Seredynski
#

import roslib; roslib.load_manifest('rcprg_ros_utils')

import rospy
import math
import numpy as np
import PyKDL
import std_srvs.srv
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *

if __name__ == "__main__":
    rospy.init_node('create_state_snapshot', anonymous=False)

    rospy.sleep(0.5)

    world_prop = None
    print 'Waiting for ROS services...'
    rospy.wait_for_service('/gazebo/get_world_properties')
    rospy.wait_for_service('/gazebo/get_model_properties')
    rospy.wait_for_service('/gazebo/get_joint_properties')
    rospy.wait_for_service('/gazebo/pause_physics')
    rospy.wait_for_service('/gazebo/unpause_physics')
    try:
        get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        get_model_properties = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
        get_joint_properties = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
        pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', std_srvs.srv.Empty)
        unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', std_srvs.srv.Empty)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

#    pause_physics()
#    print 'Simulation is paused.'

    world_prop = get_world_properties()

    file_lines = []
    for model_name in world_prop.model_names:
        print 'model_name:', model_name
        model_prop = get_model_properties(model_name)
        print 'joint_names:', model_prop.joint_names
        for joint_name in model_prop.joint_names:
            joint_prop = get_joint_properties(model_name + '::' + joint_name)
            if math.isnan(joint_prop.position[0]):
                continue
            if joint_prop.type == GetJointPropertiesResponse.REVOLUTE or \
                    joint_prop.type == GetJointPropertiesResponse.CONTINUOUS or \
                    joint_prop.type == GetJointPropertiesResponse.PRISMATIC:
                print '    ' + joint_name + ' ' + str(joint_prop.position) + 'type:' + str(joint_prop.type)
                file_lines.append( model_name + ' ' + joint_name + ' ' + str(joint_prop.position[0]) )
            else:
                print '    ' + joint_name + ' of type ' + str(joint_prop.type) + ' is not supported'
    print 'You can now save the Gazebo world using Gazebo client (File->Save World As).'
    with open('state_snapshot.txt', 'w') as f:
        for line in file_lines:
            f.write(line + '\n')