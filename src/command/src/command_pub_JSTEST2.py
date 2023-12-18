#!/usr/bin/env python

# from __future__ import division, print_function, absolute_import
import rospy
import numpy as np
import sys, select, os
import struct

import pygame
from geometry_msgs.msg import Vector3Stamped
import time

if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.05

# with open("/dev/input/js0", "rb") as f:

#     a = f.read(8)
#     t, value, code, index = struct.unpack("<Ihbb", a) # 4 bytes, 2 bytes, 1 byte, 1 byte
        # t: time in ms
        # index: button/axis number (0 for x-axis)
        # code: 1 for buttons, 2 for axis
        # value: axis position, 0 for center, 1 for buttonpress, 0 for button release
        # print(value)
        # print("t: {:10d} ms, value: {:6d}, code: {:1d}, index: {:1d}".format(t, value, code, index))


def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)


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

def checkLinearLimitVelocity(vel):

    if vel > 0.5:
        vel=0.5
    # if turtlebot3_model == "burger":
    #   vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    # elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
    #   vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    # else:
    #   vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    if np.abs(vel)<0.03 and abs(vel)>=1e-6:
        vel = np.sign(vel)*0.03
    return vel

def checkAngularLimitVelocity(vel):

    vel = np.clip(vel, -0.43, 0.31)
    # if turtlebot3_model == "burger":
    #   vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    # elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
    #   vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    # else:
    #   vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    
    
    return vel


class CommandNode(object):
    def __init__(self):
        
        self.test = 0
        self.msg = None
        self.cmd_pub_ = rospy.Publisher('/qcar/user_command', Vector3Stamped, queue_size=100)
		
        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)

        self.key_cnt = 0
        self.ex_key =' '
        
        self.status = 0
        self.target_linear_vel   = 0.0
        self.target_angular_vel  = -0.06
        self.control_linear_vel  = 0.0
        self.control_angular_vel = 0.0

        r = rospy.Rate(100)
        with open("/dev/input/js0", "rb") as f:
            while not rospy.is_shutdown():

                start_t = time.time()
                a = f.read(8)
                t, value, code, index = struct.unpack("<Ihbb", a) # 4 bytes, 2 bytes, 1 byte, 1 byte
                
            #key = getKey(self.settings)
            # key_value = value
            # key_code = code
            # key_index = index

                # print(code)
                # print("---------------------------------------------------")
                # print("t: {:10d} ms, value: {:6d}, code: {:1d}, index: {:1d}".format(t, value, code, index))

                self.process_command(t,value,code,index)
                #if (key == '\x03'):
                #    break
                r.sleep()

    def process_command(self,t,key_value,key_code,key_index):
        # if key=='' and self.key_cnt<7:
        #     if self.ex_key == 'a' or self.ex_key =='d':
        #         key = self.ex_key
        #     self.key_cnt +=1
        # print(key_value, key_code, key_index)
        if  key_code == 2 and key_index == 3 :
            #  print(key_value, key_code, key_index)
            #  print("t: {:10d} ms, value: {:6d}, code: {:1d}, index: {:1d}".format(t,key_value, key_code, key_index))
            #  self.ex_key = key
             self.target_linear_vel = checkLinearLimitVelocity(-0.5/100000*key_value)
             self.status = self.status + 1
            #  print(vels(self.target_linear_vel,self.target_angular_vel))
       
        elif key_code == 2 and key_index == 0  :
            # self.ex_key = key
            self.target_angular_vel = checkAngularLimitVelocity(-0.5/55000*key_value-0.06)
            self.status = self.status + 1
            # print(vels(self.target_linear_vel,self.target_angular_vel))

        
        elif key_code == 1 and key_index == 3 :
            self.target_linear_vel   = 0.0
            self.control_linear_vel  = 0.0
            self.target_angular_vel  = -0.06
            self.control_angular_vel = 0

        # elif key=='':
        #     self.ex_key = key
        #     self.key_cnt = 0
        #     self.target_linear_vel   += 0.0
        #     self.control_linear_vel  += 0.0
        #     self.target_angular_vel  += 0
        #     self.control_angular_vel += 0

        if self.status == 20 :
            # print(msg)
            self.status = 0

        # for event in pygame.event.get():
        #     if event.type == pygame. :
        #         pressed = pygame.key.get_pressed()
        #         buttons = [pygame.key.name(k) for k,v in enumerate(pressed) if v]
        #         test=buttons[0]
        
        msg=Vector3Stamped()
        # if test == "left":
        #     self.test -= 0.05
        # elif test=="right":
        #     self.test += 0.05
        
        # self.test = np.clip(self.test, -0.5, 0.5)

        msg.vector.x= float(self.target_linear_vel) 
        msg.vector.y = float(self.target_angular_vel)
        # float(self.target_angular_vel)
        self.cmd_pub_.publish(msg)

if __name__ == '__main__':

    rospy.init_node('command_node')
    r = CommandNode()
    
    rospy.spin()
