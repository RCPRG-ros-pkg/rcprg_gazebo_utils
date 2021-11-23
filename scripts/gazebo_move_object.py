#!/usr/bin/env python

## Provides interactive 6D pose marker and allows moving object in Gazebo.
# @ingroup utilities
# @file gazebo_move_object.py

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
import math
import copy
import tf

from std_msgs.msg import ColorRGBA
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from tf.transformations import * 
import tf_conversions.posemath as pm
import PyKDL
from cartesian_trajectory_msgs.msg import *
import actionlib
from gazebo_msgs.srv import *

class IntMarkers6D:
    def __init__(self, link_name):
        self.link_name = link_name

        rospy.wait_for_service('/gazebo/set_link_state', 4.0)
        self.set_link_state = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)

        rospy.wait_for_service('/gazebo/get_link_state', 4.0)
        get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

        req = GetLinkStateRequest()
        req.link_name = self.link_name
        req.reference_frame = "torso_base"
        resp = get_link_state(req)
        if resp.success == False:
            print "success:", resp.success, ", status:", resp.status_message
            raise Exception("/gazebo/get_link_state")

        print "IntMarkers6D init ok"

        # create an interactive marker server on the topic namespace simple_marker
        self.server = InteractiveMarkerServer('gazebo_move_object_' + self.link_name.replace(":", "_"))

        self.T_B_M = PyKDL.Frame(PyKDL.Vector(1,1,1))

        self.insert6DofGlobalMarker(self.T_B_M)

        self.server.applyChanges();

    def setLinkState(self, T_B_M):
        req = SetLinkStateRequest()
        req.link_state.link_name = self.link_name
        req.link_state.pose.position.x = T_B_M.p.x()
        req.link_state.pose.position.y = T_B_M.p.y()
        req.link_state.pose.position.z = T_B_M.p.z()
        qx, qy, qz, qw = T_B_M.M.GetQuaternion()
        req.link_state.pose.orientation.x = qx
        req.link_state.pose.orientation.y = qy
        req.link_state.pose.orientation.z = qz
        req.link_state.pose.orientation.w = qw
        req.link_state.twist.linear.x = 0
        req.link_state.twist.linear.y = 0
        req.link_state.twist.linear.z = 0
        req.link_state.twist.angular.x = 0
        req.link_state.twist.angular.y = 0
        req.link_state.twist.angular.z = 0
        req.link_state.reference_frame = "torso_base"
        resp = self.set_link_state(req)
        if resp.success == False:
            print "success:", resp.success, ", status:", resp.status_message

    def erase6DofMarker(self):
        self.server.erase(self.link_name+'_position_marker')
        self.server.applyChanges();

    def insert6DofGlobalMarker(self, T_B_M):
        int_position_marker = InteractiveMarker()
        int_position_marker.header.frame_id = 'torso_base'
        int_position_marker.name = self.link_name+'_position_marker'
        int_position_marker.scale = 0.2
        int_position_marker.pose = pm.toMsg(T_B_M)

        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.ROTATE_AXIS,'x'));
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.ROTATE_AXIS,'y'));
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.ROTATE_AXIS,'z'));
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.MOVE_AXIS,'x'));
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.MOVE_AXIS,'y'));
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.MOVE_AXIS,'z'));

        box = self.createAxisMarkerControl(Point(0.15,0.015,0.015), Point(0.0, 0.0, 0.0) )
        box.interaction_mode = InteractiveMarkerControl.BUTTON
        box.name = 'button'
        int_position_marker.controls.append( box )
        self.server.insert(int_position_marker, self.processFeedback);
        self.server.applyChanges();

    def processFeedback(self, feedback):
        if ( feedback.marker_name == self.link_name+'_position_marker' ):
            T_B_M = pm.fromMsg(feedback.pose)
            self.setLinkState(T_B_M)
            print "pose:", T_B_M.p.x(), T_B_M.p.y(), T_B_M.p.z()

    def createAxisMarkerControl(self, scale, position):
        markerX = Marker()
        markerX.type = Marker.ARROW
        markerX.scale = scale
        markerX.pose.position = position
        ori = quaternion_about_axis(0, [0, 1 ,0])
        markerX.pose.orientation = Quaternion(ori[0], ori[1], ori[2], ori[3])
        markerX.color = ColorRGBA(1,0,0,1)
        markerY = Marker()
        markerY.type = Marker.ARROW
        markerY.scale = scale
        markerY.pose.position = position
        ori = quaternion_about_axis(math.pi/2.0, [0, 0 ,1])
        markerY.pose.orientation = Quaternion(ori[0], ori[1], ori[2], ori[3])
        markerY.color = ColorRGBA(0,1,0,1)
        markerZ = Marker()
        markerZ.type = Marker.ARROW
        markerZ.scale = scale
        markerZ.pose.position = position
        ori = quaternion_about_axis(-math.pi/2.0, [0, 1 ,0])
        markerZ.pose.orientation = Quaternion(ori[0], ori[1], ori[2], ori[3])
        markerZ.color = ColorRGBA(0,0,1,1)
        control = InteractiveMarkerControl()
        control.always_visible = True;
        control.markers.append( markerX );
        control.markers.append( markerY );
        control.markers.append( markerZ );
        return control

    def createInteractiveMarkerControl6DOF(self, mode, axis):
        control = InteractiveMarkerControl()
        control.orientation_mode = InteractiveMarkerControl.FIXED
        if mode == InteractiveMarkerControl.ROTATE_AXIS:
            control.name = 'rotate_';
        if mode == InteractiveMarkerControl.MOVE_AXIS:
            control.name = 'move_';
        if axis == 'x':
            control.orientation = Quaternion(1,0,0,1)
            control.name = control.name+'x';
        if axis == 'y':
            control.orientation = Quaternion(0,1,0,1)
            control.name = control.name+'x';
        if axis == 'z':
            control.orientation = Quaternion(0,0,1,1)
            control.name = control.name+'x';
        control.interaction_mode = mode
        return control

def printUsage():
    print "usage: gazebo_move_object model_name::link_name"

if __name__ == "__main__":

    rospy.init_node('gazebo_move_object', anonymous=True)

    if len(sys.argv) != 2:
        printUsage()
        exit(1)

    int_markers = IntMarkers6D(sys.argv[1])

    rospy.spin()

