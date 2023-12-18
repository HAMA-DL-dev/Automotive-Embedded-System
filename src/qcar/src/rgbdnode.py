#!/usr/bin/env python3
from __future__ import division, print_function, absolute_import

import roslib
import rospy
import numpy as np
import cv2
from qcar.q_essential import Camera3D

from sensor_msgs.msg import Image
from std_msgs.msg import String

import sys
sys.path.append("~/ros1/src/vision_opencv/cv_bridge/python")
from cv_bridge import CvBridge, CvBridgeError
import cv_bridge

print(cv_bridge.__file__)


class RGBDNode(object):
	def __init__(self):
		super().__init__()
		self.rgbd_color_pub = rospy.Publisher('/qcar/rgbd_color', Image, queue_size=10)
		self.rgbd_depth_pub = rospy.Publisher('/qcar/rgbd_depth', Image, queue_size=10)
		self.bridge = CvBridge()
		
		#Initialize CV sensors
		# frame_width_RGB=1920, frame_height_RGB=1080, frame_rate_RGB=30.0, frame_width_depth=1280, frame_height_depth=720, frame_rate_depth=15.0
		# frame_width_RGB=1280, frame_height_RGB=720
		rgbd = Camera3D(mode='RGB&DEPTH', frame_width_RGB=640, frame_height_RGB=480, frame_rate_RGB=30.0, frame_width_depth=640, frame_height_depth=480, frame_rate_depth=30.0)

		#Get readings
		try:
			while True:
				rgbd.read_RGB()
				rgbd.read_depth(dataMode='m')
				
				# self.rate_pub.publish(msg)
				self.process_color_data(self.rgbd_color_pub, rgbd.image_buffer_RGB)
				self.process_depth_data(self.rgbd_depth_pub, rgbd.image_buffer_depth_m)

		except KeyboardInterrupt:
			print("User interrupted!")

		finally: 

			# Terminate the LIDAR object
			rgbd.terminate()
#--------------------------------------------------------------------------------------------------------------
	def process_color_data(self, cam_info, img_data):
		
		pub_img = self.bridge.cv2_to_imgmsg(img_data, "bgr8")	
		pub_img.header.stamp =  rospy.Time.now()
		pub_img.header.frame_id = 'RGBD_color_input'
		cam_info.publish(pub_img)

	def process_depth_data(self, cam_info, img_data):
		pub_img = self.bridge.cv2_to_imgmsg(img_data, "32FC1")
		pub_img.header.stamp =  rospy.Time.now()
		pub_img.header.frame_id = 'RGBD_depth_input'
		cam_info.publish(pub_img)

if __name__ == '__main__':
	rospy.init_node('rgbd_node', disable_signals=True)
	r = RGBDNode()

	rospy.spin()
		
