#!/usr/bin/env python3

import rospy
import math
import numpy as np
from numpy import nan

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class Distance_Detection:
    def __init__(self):
        #self.depth_information
        self.front_avg_array = np.zeros(10)
        self.front_ranges = 0
        self.front_empty_flag = 0
        self.front_count = 0

        self.back_avg_array = np.zeros(10)
        self.back_ranges = 0
        self.back_empty_flag = 0
        self.back_count = 0

        #Initializing the Subscriber
        self.LaserScanFront_sub = rospy.Subscriber('object_detection/scan_front', LaserScan, self.LaserScanFront_callback)
        self.LaserScanBack_sub = rospy.Subscriber('object_detection/scan_back', LaserScan, self.LaserScanBack_callback)

        #Initializing the Publisher

        self.front_range_Pub = rospy.Publisher('object_detection/front_dist', Float32, queue_size = 10)
        self.back_range_Pub = rospy.Publisher('object_detection/back_dist', Float32, queue_size = 10)

        #Initializing the rolling window average node
        self.distance_Timer = rospy.Timer(rospy.Duration(0.005), self.distance_Timer_callback)
    
    def LaserScanFront_callback(self,msg):
        self.front_empty_flag = 0
        self.front_ranges = msg.ranges
        self.front_ranges = [x for x in self.front_ranges if str(x) != 'nan'] #Removing NaN array population
        #print(self.front_ranges)
        if len(self.front_ranges) == 0:
            self.front_empty_flag = 1
            return self.front_empty_flag
        self.front_minRange = np.min(self.front_ranges)
        if self.front_count == 10:
            self.front_count = 0
        if self.front_count < 10:
            self.front_avg_array[self.front_count] = self.front_minRange
            self.front_count += 1

    def LaserScanBack_callback(self,msg):
        self.back_empty_flag = 0
        self.back_ranges = msg.ranges
        self.back_ranges = [x for x in self.back_ranges if str(x) != 'nan'] #Removing NaN array population
        #print(self.back_ranges)
        if len(self.back_ranges) == 0:
            self.back_empty_flag = 1
            return self.back_empty_flag
        self.back_minRange = np.min(self.back_ranges)
        if self.back_count == 10:
            self.back_count = 0
        if self.back_count < 10:
            self.back_avg_array[self.back_count] = self.back_minRange
            self.back_count += 1

    
    def distance_Timer_callback(self,event):
        #For Front Camera
        if self.front_empty_flag == 1:
            self.front_avg_minRange = 0
            rospy.logwarn('Objects are too close for front sensor readings ...')
            front_msg = Float32()
            front_msg.data = self.front_avg_minRange
            self.front_range_Pub.publish(front_msg)
            #return self.front_avg_minRange
        if self.front_empty_flag == 0:
            self.front_avg_minRange = np.mean(self.front_avg_array)
            front_msg = Float32()
            front_msg.data = self.front_avg_minRange
            self.front_range_Pub.publish(front_msg)
            #return self.front_avg_minRange
        
        #For Back Camera
        if self.back_empty_flag == 1:
            self.back_avg_minRange = 0
            rospy.logwarn('Objects are too close for rear sensor readings ...')
            back_msg = Float32()
            back_msg.data = self.back_avg_minRange
            self.back_range_Pub.publish(back_msg)
            #return self.back_avg_minRange
        if self.back_empty_flag == 0:
            self.back_avg_minRange = np.mean(self.back_avg_array)
            back_msg = Float32()
            back_msg.data = self.back_avg_minRange
            self.back_range_Pub.publish(back_msg)
            #return self.back_avg_minRange

if __name__ == '__main__':
    rospy.init_node('object_detection')
    rospy.loginfo('The object detection node has been started.')

    OD = Distance_Detection()
    rospy.spin()

