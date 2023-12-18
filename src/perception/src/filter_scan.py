#! /usr/bin/env python

# from mmc_msgs.msg import object_msg, object_array_msg, chassis_msg, localization2D_msg, dataset_array_msg, dataset_msg, lane_array_msg, lane_msg, can_future_msg, can_future_array_msg
from sensor_msgs.msg import LaserScan
import sensor_msgs
import laser_geometry.laser_geometry as lg
from sklearn.cluster import DBSCAN
import rospy
import numpy as np
import time
# import matplotlib.pyplot as plt
import math 
import pickle
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point32, PolygonStamped, Polygon, Vector3, Pose, Quaternion, Point, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Float32, Float64, Header, ColorRGBA, UInt8, Int32, String

class scan_filtering(object):
    def __init__(self):
        rospy.init_node('lidar_dbscan')

        self.sub_scan = rospy.Subscriber('/scan_filtered', LaserScan, self.callback, queue_size=1)
        self.rviz_pub_object  = rospy.Publisher('/rviz/lane_line', MarkerArray, queue_size=1)
        self.filtered_scan  = rospy.Publisher('/scan/filtered', LaserScan, queue_size=1)
        
        r = rospy.Rate(10)

        curr_time = rospy.Time.now()

        while not rospy.is_shutdown():
            r.sleep()


    def callback(self,msg):    
        self.point_filtered = LaserScan()
        # self.point_filtered.header.frame_id = msg.header.frame_id  
        # self.point_filtered.angle_increment = msg.angle_increment
        # self.point_filtered.time_increment = msg.time_increment
        # self.point_filtered.ranges = msg.ranges
        # self.point_filtered.intensities = msg.intensities
        # self.point_filtered.angle_min = -math.pi / 4
        # self.point_filtered.angle_max = math.pi / 4
        # self.point_filtered.range_min = 0.0
        # self.point_filtered.range_max = 1.0          
        self.point_filtered = msg 
        self.filtered_scan.publish(self.point_filtered)
        
    

if __name__ == '__main__':
    try:
        do_scan = scan_filtering()

    except rospy.ROSInterruptException:
        rospy.logerr('Shutdown')
