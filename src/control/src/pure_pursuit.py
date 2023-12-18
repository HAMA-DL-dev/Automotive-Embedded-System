#!/usr/bin/python3
'''
#!/usr/bin/env python3
'''
# from __future__ import division, print_function, absolute_import
import rospy
import numpy as np
import sys, select, os

import pygame
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Vector3Stamped
from custom_msgs.msg import waypoints
import time
import math

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

LIN_VEL = 0.05
ANG_VEL = 0.0

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
    
    if np.abs(vel)<0.03 and abs(vel)>=1e-6:
        vel = np.sign(vel)*0.03
    return vel

def checkAngularLimitVelocity(vel):

    vel = np.clip(vel, -0.063-0.5, -0.063+0.5)
    vel = np.clip(vel, -0.063-0.3,-0.063+0.25) # Track minimum curvature 정도로 limit

    return vel


class ControlNode(object):
    def __init__(self):

        self.test = 0
        self.msg = None
        self.waypoint_sub = rospy.Subscriber('/Scan_to_PointCloud/waypoints_geo',waypoints,self.subWaypoints,queue_size=100)
        self.waypoint_sub = rospy.Subscriber('/Scan_to_PointCloud/front',PointCloud2,self.CheckFront,queue_size=100)
        self.cmd_pub_ = rospy.Publisher('/qcar/user_command', Vector3Stamped, queue_size=100)
		
        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)

        self.key_cnt = 0
        self.ex_key =' '
        self.enable_key = 0
        self.status = 0
        self.stop = 0
        
        self.mode = "STOP"

        self.min = 1.0
        self.max = 0.0
        self.target_linear_vel   = 0.0
        self.target_angular_vel  = 0.0
        self.control_linear_vel  = -0.063
        self.control_angular_vel = 0.0

        ## params for pure pursuit ## 
        self.laser2wheel = 0.15 # [m]  ;
        self.ld = 0.0           # [m]  ; rear wheel to waypoint
        self.L = 0.39           # [m]  ; overall length
        self.R = 0.462          # [m]  ; turning radius
        self.wheelbase = 0.26   # [m]  ; wheel base 
        self.delta = 0.0        # [rad]; 
        self.alpha = 0.0        # [rad]; 
        
        while not rospy.is_shutdown():
            print("Mode : {} || Target Angular Velocity : {:.3f}\r".format(self.mode,self.delta))    
            key = getKey(self.settings)
            self.pure_pursuit(key)
            if (key == '\x03'):
                break
            time.sleep(0.01)

    def CheckFront(self,msg):
        if len(msg.data) > 2:
            self.stop = 1

    def subWaypoints(self,msg):

        if (len(msg.points) != 0):
            self.ld=msg.points[0].x
            self.lookahead = 0.5
            self.radius = (self.ld**2 + self.lookahead**2) / (self.ld * 2)
            self.delta = -self.wheelbase / self.radius

        # for i in msg.points:
        #     if abs(i.x) == float("inf") or abs(i.y) == float("inf"):continue
        #     self.ld = i.x
        #     self.ld *= -50.0
        #     self.alpha = math.atan((i.x/i.y))

        #     # lookahead = 0.3
        #     # radius = (self.ld**2 + lookahead**2) / (self.ld * 2)
        #     # self.delta = 0.26 / radius

        #     self.delta = self.ld * (-3)
        # print("Mode : {} || Target Angular Velocity : {:.3f}\r".format(self.mode,self.delta))
    
        

    def pure_pursuit(self,key):

        if self.target_angular_vel < -0.063:
            w = 1
        elif self.target_angular_vel > -0.063:
            w = 4/5
        elif self.target_angular_vel == -0.063 and key =='a':
            w = 4/5
        elif self.target_angular_vel == -0.063 and key =='d':
            w = 1 

        if key == 's':
            self.mode = "GO"
            self.enable_key = 1

        elif key == ' ':
            self.mode = "STOP"
            self.enable_key = 0
            self.target_linear_vel   = 0.0
            self.control_linear_vel  = 0.0
            self.target_angular_vel  = -0.063
            self.control_angular_vel = 0

        if(self.enable_key == 1):
            # if self.stop == 1:
                # self.target_linear_vel   = 0.0
                # self.control_linear_vel  = 0.0
                # self.target_angular_vel  = self.delta
                # self.control_angular_vel = 0
            # else:
            self.mode = "GO"
            self.ex_key = key
            self.target_linear_vel = 0.08 # 0.06 0.055
            self.target_angular_vel = self.delta

        msg=Vector3Stamped()
        msg.vector.x= float(self.target_linear_vel) 
        msg.vector.y = float(self.target_angular_vel)
        self.cmd_pub_.publish(msg)

if __name__ == '__main__':

    rospy.init_node('command_node')
    r = ControlNode()
    
    rospy.spin()