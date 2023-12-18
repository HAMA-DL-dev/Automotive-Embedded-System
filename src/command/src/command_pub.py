#!/usr/bin/python3
'''
#!/usr/bin/env python3
'''
# from __future__ import division, print_function, absolute_import
import rospy
import numpy as np
import sys, select, os

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
ANG_VEL_STEP_SIZE = 0.01

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %.3f\t angular vel %.3f " % (target_linear_vel,target_angular_vel)


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

    vel = np.clip(vel, -0.063-0.5, -0.063+0.5)
    vel = np.clip(vel, -0.063-0.3,-0.063+0.25) # Track minimum curvature 정도로 limit
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
        self.target_angular_vel  = 0.0
        self.control_linear_vel  = -0.063
        self.control_angular_vel = 0.0
        
        while True:
            key = getKey(self.settings)
            self.process_command(key)
            if (key == '\x03'):
                break
            time.sleep(0.01)

    def process_command(self,key):
        
        # w_left = 4/5
        # w_right = 1

        if self.target_angular_vel < -0.063:
            w = 1
        elif self.target_angular_vel > -0.063:
            w = 4/5
        elif self.target_angular_vel == -0.063 and key =='a':
            w = 4/5
        elif self.target_angular_vel == -0.063 and key =='d':
            w = 1 

        # if key=='' and self.key_cnt<7:
        #     if self.ex_key == 'a' or self.ex_key =='d':
        #         key = self.ex_key
        #     self.key_cnt +=1

        if key == 'w' :
            self.ex_key = key
            self.target_linear_vel = checkLinearLimitVelocity(self.target_linear_vel + LIN_VEL_STEP_SIZE)
            self.status = self.status + 1
            print(vels(self.target_linear_vel,self.target_angular_vel))
        elif key == 'x' :
            self.ex_key = key
            self.target_linear_vel = checkLinearLimitVelocity(self.target_linear_vel - LIN_VEL_STEP_SIZE)
            self.status = self.status + 1
            print(vels(self.target_linear_vel,self.target_angular_vel))
        elif key == 'a' :
            self.ex_key = key
            #self.target_angular_vel = checkAngularLimitVelocity(self.target_angular_vel + ANG_VEL_STEP_SIZE)
            self.target_angular_vel = checkAngularLimitVelocity(self.target_angular_vel + w * ANG_VEL_STEP_SIZE)
            self.status = self.status + 1
            print(vels(self.target_linear_vel,self.target_angular_vel))
        elif key == 'd' :
            self.ex_key = key
            # self.target_angular_vel = checkAngularLimitVelocity(self.target_angular_vel - ANG_VEL_STEP_SIZE)
            self.target_angular_vel = checkAngularLimitVelocity(self.target_angular_vel - w *ANG_VEL_STEP_SIZE)
            self.status = self.status + 1
            print(vels(self.target_linear_vel,self.target_angular_vel))
        elif key == 's' or key == ' ':
            self.target_linear_vel   = 0.0
            self.control_linear_vel  = 0.0
            self.target_angular_vel  = -0.063
            self.control_angular_vel = 0
            print(vels(self.target_linear_vel,self.target_angular_vel))

        elif key=='':
            self.ex_key = key
            self.key_cnt = 0
            self.target_linear_vel   += 0.0
            self.control_linear_vel  += 0.0
            self.target_angular_vel  += 0
            self.control_angular_vel += 0

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