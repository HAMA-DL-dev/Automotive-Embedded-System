#!/usr/bin/env python
from __future__ import division, print_function, absolute_import

import roslib
import rospy
import numpy as np

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Vector3Stamped, TransformStamped
from sensor_msgs.msg import BatteryState
import time

import tf_conversions
import tf2_ros

class odom2tf_node(object):
	def __init__(self):
		self.state_sub = rospy.Subscriber('/qcar/state', Vector3Stamped, self.callback_state, queue_size=10)
		self.odom_tf_broadcaster = tf2_ros.TransformBroadcaster()

		#################### for odom
		self.yaw = 0.0
		self.x = 0.0
		self.y = 0.0


#-------------------------------------------------------------------------------------------------
	def callback_state(self,state):
		self.x = state.vector.x
		self.y = state.vector.y
		self.yaw = state.vector.z
		
		t = TransformStamped()
		t.header.stamp = rospy.Time.now()
		t.header.frame_id = "odom"
		t.child_frame_id = "base_link"
		t.transform.translation.x = self.x
		t.transform.translation.y = self.y
		t.transform.translation.z = 0.0
		q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.yaw)
		t.transform.rotation.x = q[0]
		t.transform.rotation.y = q[1]
		t.transform.rotation.z = q[2]
		t.transform.rotation.w = q[3]
		self.odom_tf_broadcaster.sendTransform(t)

		t = TransformStamped()
		t.header.stamp = rospy.Time.now()
		t.header.frame_id = "base_link"
		t.child_frame_id = "lidar"
		t.transform.translation.x = 0.12
		t.transform.translation.y = 0
		t.transform.translation.z = 0.15
		q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
		t.transform.rotation.x = q[0]
		t.transform.rotation.y = q[1]
		t.transform.rotation.z = q[2]
		t.transform.rotation.w = q[3]
		self.odom_tf_broadcaster.sendTransform(t)
		#############################################

if __name__ == '__main__':
	rospy.init_node('odom2tf')
	r = odom2tf_node()

	rospy.spin()
	
