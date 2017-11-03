#!/usr/bin/env python

## Publishes transform of object in Gazebo simulator.
# @ingroup utilities
# @file gazebo_publish_ros_tf_object.py
# @namespace scripts.gazebo_publish_ros_tf_object Publishes transform of object in Gazebo simulator.

# Copyright (c) 2017, Robot Control and Pattern Recognition Group,
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

import roslib; roslib.load_manifest('rcprg_gazebo_utils')

import sys
import rospy
import tf
import tf2_ros

from geometry_msgs.msg import *
from gazebo_msgs.srv import *

class GazeboTfPublisher:
    def __init__(self, link_name, frame_id):
        self._link_name = link_name
        self._frame_id = frame_id

        self._service_name = '/gazebo/get_link_state'
        self.get_link_state = None
        self._br = tf2_ros.TransformBroadcaster()

    def spin(self):
        while not rospy.is_shutdown():
            req = GetLinkStateRequest()
            req.link_name = self._link_name
            req.reference_frame = "world"
            if self.get_link_state == None:
                try:
                    rospy.wait_for_service(self._service_name, 4.0)
                    self.get_link_state = rospy.ServiceProxy(self._service_name, GetLinkState)
                    print "connected to ROS service " + self._service_name
                except rospy.exceptions.ROSException as e:
                    print e
                    print "Could not connect to ROS service " + self._service_name + ", retrying..."
                    continue
            try:
                resp = self.get_link_state(req)
            except rospy.service.ServiceException as e:
                self.get_link_state = None
                print e
                print "Could not call to ROS service " + self._service_name + ", retrying..."
                continue

            if resp.success == False:
                print "success:", resp.success, ", status:", resp.status_message
                raise Exception(self._service_name)
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "world"
            t.child_frame_id = self._frame_id
            t.transform.translation.x = resp.link_state.pose.position.x
            t.transform.translation.y = resp.link_state.pose.position.y
            t.transform.translation.z = resp.link_state.pose.position.z
            t.transform.rotation = resp.link_state.pose.orientation
            self._br.sendTransform(t)
            try:
                rospy.sleep(0.1)
            except:
                break

def printUsage():
    print "usage: gazebo_publish_ros_tf_object.py model_name::link_name frame_id"

if __name__ == "__main__":

    rospy.init_node('gazebo_publish_ros_tf_object', anonymous=True)
    rospy.sleep(0.5)

    try:
        link_name = rospy.get_param("~link_name")
        frame_id = rospy.get_param("~frame_id")
    except KeyError as e:
        print "Some ROS parameters are not provided:"
        print e
        exit(1)

    if len(sys.argv) != 3:
        printUsage()
        exit(1)

    pub = GazeboTfPublisher(link_name, frame_id)

    pub.spin()

