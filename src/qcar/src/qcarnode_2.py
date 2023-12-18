#!/usr/bin/env python3
from __future__ import division, print_function, absolute_import

import roslib
import rospy
import numpy as np
from qcar.product_QCar import QCar
from qcar.q_interpretation import *

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Vector3Stamped, TransformStamped, Twist
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
import time

# import tf_conversions
# import tf2_ros


class QcarNode(object):

	def __init__(self):
		super().__init__()
		self.battery_pub_ = rospy.Publisher('/qcar/battery_state', BatteryState, queue_size=10)
		self.carvel_pub_ = rospy.Publisher('/qcar/velocity', Vector3Stamped, queue_size=10)
		self.state_pub = rospy.Publisher('/qcar/state', Vector3Stamped, queue_size=10)
		self.odom_pub = rospy.Publisher('/qcar/odom', Odometry, queue_size=10)
		self.myCar = QCar()
		self.sample_time = 0.05
		self.cmd_sub_ = rospy.Subscriber('/qcar/user_command', Vector3Stamped, self.process_cmd, queue_size=100)
		self.cmd_sub_2 = rospy.Subscriber('/cmd_vel', Twist, self.process_cmd_ctrl, queue_size=100)
		#################### for odom
		self.yaw = 0.0
		self.x = 0.0
		self.y = 0.0
		
		self.encoderCounts_prev = 0
		self.dt = self.sample_time
		self.time_ex = None
		self.dist_ex = None
		self.dist = 0.0
		self.command = np.array([0, 0])
		self.yaw_array = []
		##################### Main Function
		while not rospy.is_shutdown():
			self.main_function()
			time.sleep(self.sample_time)
		else:
			print("shutdown qcarnode")
			self.myCar.terminate()
		# self.odom_tf_broadcaster = tf.TransformBroadcaster()

#-------------------------------------------------------------------------------------------------
	def main_function(self):
		
		# Generate Commands
		LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])

		# talk to QCar
		current, batteryVoltage, encoderCounts = self.myCar.read_write_std(self.command, LEDs)   
		gyro, accel = self.myCar.read_IMU()
		self.yaw_array.append(gyro[2])

		# if len(self.yaw_array)>100:
			# print('yaw_bias : ',np.mean(self.yaw_array))  # -0.002
		# print('gyro : ', gyro) #(x,y,z axis)
		# print('accel : ', accel)  

		battery_state = BatteryState()
		battery_state.header.stamp = rospy.Time.now() 
		battery_state.header.frame_id = 'battery_voltage'
		battery_state.voltage = batteryVoltage
		self.battery_pub_.publish(battery_state)

		longitudinal_car_speed = basic_speed_estimation((encoderCounts-self.encoderCounts_prev)/self.dt)
		self.encoderCounts_prev = encoderCounts
		
		
		velocity_state = Vector3Stamped()
		velocity_state.header.stamp = rospy.Time.now() 
		velocity_state.header.frame_id = 'car_velocity'
		velocity_state.vector.x = float(np.cos(self.yaw) * longitudinal_car_speed)
		velocity_state.vector.y = float(np.sin(self.yaw) * longitudinal_car_speed)
		self.carvel_pub_.publish(velocity_state)
		# print(longitudinal_car_speed)
		# print(self.yaw)
		################################## for odom
		if self.time_ex is None:
			self.time_ex = rospy.Time.now()
		else:
			dt = (rospy.Time.now() - self.time_ex)
			self.dt = dt.to_sec() + 1e-9*dt.to_nsec()
			self.time_ex = rospy.Time.now()
			self.x += velocity_state.vector.x* self.dt
			self.y += velocity_state.vector.y * self.dt
			if abs(longitudinal_car_speed) > 0.01:
				self.yaw += (gyro[2])*self.dt*0.48
		state = Vector3Stamped()
		state.header.stamp = rospy.Time.now() 
		state.header.frame_id = 'odom'
		state.vector.x = self.x
		state.vector.y = self.y
		state.vector.z = self.yaw
		self.state_pub.publish(state)
		
		odometry = Odometry()
		odometry.header.frame_id = 'odom'
		odometry.header.stamp = rospy.Time.now()
		odometry.pose.pose.position.x = self.x
		odometry.pose.pose.position.y = self.y
		odometry.pose.pose.orientation.x = 0
		odometry.pose.pose.orientation.y = 0
		odometry.pose.pose.orientation.z = np.sin(self.yaw/2)
		odometry.pose.pose.orientation.w = np.cos(self.yaw/2)
		odometry.twist.twist.linear.x = velocity_state.vector.x
		odometry.twist.twist.linear.y = velocity_state.vector.y
		odometry.twist.twist.angular.z = (gyro[2])
		self.odom_pub.publish(odometry)

	def process_cmd(self, sub_cmd):
		vel_cmd = sub_cmd.vector.x
		str_cmd = sub_cmd.vector.y - 0.01
		self.command = np.array([vel_cmd, str_cmd])
	def process_cmd_ctrl(self, sub_cmd):
		vel_cmd = sub_cmd.linear.x
		str_cmd = sub_cmd.angular.z 
		self.command = np.array([vel_cmd, str_cmd])

if __name__ == '__main__':
	rospy.init_node('qcar_node')
	r = QcarNode()

	rospy.spin()
	
