#!/usr/bin/env python

# from __future__ import division, print_function, absolute_import
import rospy
import numpy as np
import sys, select, os
import struct
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3Stamped
import time

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.05

if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

def getKey(settings):
    if os.name == 'nt':
      return msvcrt.getch()
    tty.setraw(sys.stdin.fileno())
    rlist,_,_ = select.select([sys.stdin],[],[],0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def checkLinearLimitVelocity(vel):
    if vel > 0.5:
        vel=0.5
    return vel

def checkAngularLimitVelocity(vel):
    vel = np.clip(vel, -0.063-0.35,-0.063+0.3)
    return vel

class CommandNode(object):
    def __init__(self):
        
        self.test = 0
        self.msg = None
        self.cmd_pub_ = rospy.Publisher('/qcar/user_command', Vector3Stamped, queue_size=100)
        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)                
        self.status = 0
        self.target_linear_vel   = 0.0
        self.target_angular_vel  = -0.063
        msg=Vector3Stamped()
        msg.vector.x= float(self.target_linear_vel) 
        msg.vector.y = float(self.target_angular_vel)
        self.cmd_pub_.publish(msg)
        self.control_linear_vel  = 0.0
        self.control_angular_vel = 0.0

        r = rospy.Rate(100)
        with open("/dev/input/js0", "rb") as f:
            while not rospy.is_shutdown():
                start_t = time.time()
                a = f.read(8)
                t, value, code, index = struct.unpack("<Ihbb", a)
                self.process_command(t,value,code,index)
                r.sleep()

    def process_command(self,t,key_value,key_code,key_index):
        print(key_value, key_code, key_index)
        if key_code == 2 and key_index == 3 :
		if key_value > 30000:
			self.target_angular_vel = -0.35
		elif key_value > 10000 and key_value < 30000:
			self.target_angular_vel = -0.1
		elif key_value < 10000 and key_value >-10000:
			self.target_angular_vel = 0.0
		elif key_value < -10000 and key_value >-30000:
			self.target_angular_vel = 0.1
		else:
			self.target_angular_vel = 0.30
	elif key_code == 2 and key_index == 4:

		if key_value < -30000:
			self.target_linear_vel = 0.1 

		elif key_value < -2000:
			self.target_linear_vel = 0.1
		elif key_value > -2000 and key_value <2000:
			self.target_linear_vel = 0
		else: 
			self.target_linear_vel = -0.1
	msg=Vector3Stamped()
        msg.vector.x= float(self.target_linear_vel) 
        msg.vector.y = float(self.target_angular_vel)
        self.cmd_pub_.publish(msg)
        
        

if __name__ == '__main__':

    rospy.init_node('command_node')
    r = CommandNode()
    
    rospy.spin()
