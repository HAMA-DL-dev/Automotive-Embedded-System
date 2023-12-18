#!/usr/bin/env python3
from __future__ import division, print_function, absolute_import

import roslib
import rospy
import numpy as np
from qcar.product_QCar import QCar
from qcar.q_interpretation import *

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import BatteryState
import time

class QcarNode(object):

	def __init__(self):
		super().__init__()
		self.battery_pub_ = rospy.Publisher('/qcar/battery_state', BatteryState, queue_size=10)
		self.carvel_pub_ = rospy.Publisher('/qcar/velocity', Vector3Stamped, queue_size=10)
		self.state_pub = rospy.Publisher('/qcar/state', Vector3Stamped, queue_size=10)
		self.myCar = QCar()
		self.sample_time = 0.001
		self.cmd_sub_ = rospy.Subscriber('/qcar/user_command', Vector3Stamped, self.process_cmd, queue_size=100)

		#################### for odom
		self.yaw = 0.0
		self.x = 0.0
		self.y = 0.0

		self.dt = 0.00
		self.time_ex = None
		self.dist_ex = None
		self.dist = 0.0

		self.yaw_array = []
		#####################

		# self.odom_tf_broadcaster = tf.TransformBroadcaster()

#-------------------------------------------------------------------------------------------------
	def process_cmd(self, sub_cmd):
		vel_cmd = sub_cmd.vector.x
		str_cmd = sub_cmd.vector.y - 0.01
		command = np.array([vel_cmd, str_cmd])
		
		# Generate Commands
		LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])

		# talk to QCar
		current, batteryVoltage, encoderCounts = self.myCar.read_write_std(command, LEDs)   
		# gyro, accel = self.myCar.read_IMU()
		# self.yaw_array.append(gyro[2])

		# if len(self.yaw_array)>100:
			# print('yaw_bias : ',np.mean(self.yaw_array))  # -0.002
		# print('gyro : ', gyro) #(x,y,z axis)
		# print('accel : ', accel)  

		battery_state = BatteryState()
		battery_state.header.stamp = rospy.Time.now() 
		battery_state.header.frame_id = 'battery_voltage'
		battery_state.voltage = batteryVoltage
		self.battery_pub_.publish(battery_state)

		longitudinal_car_speed = basic_speed_estimation(encoderCounts)
		if self.dist_ex is None:
			self.dist_ex = longitudinal_car_speed
		else:
			self.dist = longitudinal_car_speed-self.dist_ex
			self.dist_ex = longitudinal_car_speed

		
		velocity_state = Vector3Stamped()
		velocity_state.header.stamp = rospy.Time.now() 
		velocity_state.header.frame_id = 'car_velocity'
		velocity_state.vector.x = float(np.cos(command[1]) * longitudinal_car_speed)
		velocity_state.vector.y = float(np.sin(command[1]) * longitudinal_car_speed)
		self.carvel_pub_.publish(velocity_state)
	
		################################## for odom
		if self.time_ex is None:
			self.time_ex = rospy.Time.now()
		else:
			dt = (rospy.Time.now() - self.time_ex)
			self.dt = dt.to_sec() + 1e-9*dt.to_nsec()
			self.time_ex = rospy.Time.now()
		
			# self.yaw +=(gyro[2]+0.00045)*self.dt
			self.yaw +=1.25*self.dist/0.3*(command[1]+0.063)
			self.x += self.dist*np.cos(self.yaw)
			self.y += self.dist*np.sin(self.yaw)

		state = Vector3Stamped()
		state.header.stamp = rospy.Time.now() 
		state.header.frame_id = 'odom'
		state.vector.x = self.x
		state.vector.y = self.y
		state.vector.z = self.yaw
		# self.state_pub.publish(state)

		# _quat = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
		# print('pos', (self.x, self.y, self.yaw, self.dt))
		# self.odom_tf_broadcaster.sendTransform((self.x, self.y, 1.5),
        #                                               _quat,
        #                                               rospy.get_rostime(),
        #                                               "/base_link", "/odom")
		
		#############################################
			
		time.sleep(self.sample_time)


if __name__ == '__main__':
	rospy.init_node('qcar_node')
	r = QcarNode()

	rospy.spin()
	
