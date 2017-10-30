#!/usr/bin/env python

## Publishes transform of object in Gazebo simulator.
# @ingroup utilities
# @file gazebo_publish_ros_tf_object.py
# @namespace scripts.gazebo_publish_ros_tf_object Publishes transform of object in Gazebo simulator.

import roslib; roslib.load_manifest('velma_controller')

import sys
import rospy
import tf
import tf2_ros

#from visualization_msgs.msg import *
from geometry_msgs.msg import *
#from tf.transformations import * 
from gazebo_msgs.srv import *

class GazeboTfPublisher:
    def __init__(self, link_name, frame_id):
        self._link_name = link_name
        self._frame_id = frame_id

        rospy.wait_for_service('/gazebo/get_link_state', 4.0)
        self.get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        self._br = tf2_ros.TransformBroadcaster()

    def spin(self):
        while not rospy.is_shutdown():
            req = GetLinkStateRequest()
            req.link_name = self._link_name
            req.reference_frame = "world"
            resp = self.get_link_state(req)
            if resp.success == False:
                print "success:", resp.success, ", status:", resp.status_message
                raise Exception("/gazebo/get_link_state")
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "world"
            t.child_frame_id = self._frame_id
            t.transform.translation.x = resp.link_state.pose.position.x
            t.transform.translation.y = resp.link_state.pose.position.y
            t.transform.translation.z = resp.link_state.pose.position.z
            t.transform.rotation = resp.link_state.pose.orientation
            self._br.sendTransform(t)

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

