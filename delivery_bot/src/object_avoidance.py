#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np

class Object_Avoidance:
    def __init__(self):
        self.minDist = 0.5 #Minimum distance from EVA that an object can be
        self.maxDist = 1.5 #Maximum distance to consider when normalizing
        self.front_coeff = 1 #Front coefficient - 1 = Full command 0 = Zeroed command
        self.back_coeff = 1  #Back coefficient - 1 = Full command 0 = Zeroed command

        self.cmdVelRocos = Twist()
        self.cmdVel = Twist()
        #Creating the necessary subscribers
        self.cmdVelRocos_sub = rospy.Subscriber('/cmd_vel_rocos', Twist, self.cmdVelRocos_callback)

        self.frontDist_sub = rospy.Subscriber('/object_detection/front_dist', Float32, self.frontDist_callback)
        self.backDist_sub = rospy.Subscriber('/object_detection/back_dist', Float32, self.backDist_callback)
        self.cmdVel_pub = rospy.Publisher('/cmd_vel',Twist, queue_size = 1)

    def normalizer(self, min, max, value):
        self.coeff = (value - min) / (max - min)

        if self.coeff > 1:
            self.coeff = 1
        if self.coeff < 0:
            self.coeff = 0
        return np.cbrt(self.coeff)
    
    def cmdVelRocos_callback(self,msg):
        self.cmdVelRocos = msg #cmdVelRocos is the cmd_vel x command coming from rocos
        

        #print("The altered linear velocity: {}".format(self.cmdVelRocos.linear.x * self.normal_coeff))
        #Rear Detection
        if self.cmdVelRocos.linear.x < 0: 
            self.cmdVelRocos.linear.x *= self.back_coeff
            print("The full cmd_vel published command {}".format(self.cmdVelRocos))
            print("The Back Detection coefficient is {}".format(self.back_coeff))

            self.cmdVel_pub.publish(self.cmdVelRocos)
            return self.cmdVelRocos

        #Front Detection
        if self.cmdVelRocos.linear.x > 0:
            self.cmdVelRocos.linear.x *= self.front_coeff
            print("The full cmd_vel published command {}".format(self.cmdVelRocos))
            print("The Front Detection coefficient is {}".format(self.front_coeff))

            self.cmdVel_pub.publish(self.cmdVelRocos)
            return self.cmdVelRocos

        #No Command
        else:
            self.cmdVel_pub.publish(self.cmdVelRocos)

    def frontDist_callback(self,msg):
        self.front_dist = msg.data
        self.front_coeff = self.normalizer(self.minDist, self.maxDist, self.front_dist)
        return self.front_coeff

    def backDist_callback(self,msg):
        self.back_dist = msg.data
        self.back_coeff = self.normalizer(self.minDist, self.maxDist, self.back_dist)
        return self.back_coeff
    # def objectDist_callback(self, msg):
        
    #     self.distance = msg.data
    #     self.normal_coeff = self.normalizer(self.minDist, self.maxDist, self.distance)
    #     return self.normal_coeff

if __name__ == '__main__':
    rospy.init_node('object_avoidance')
    rospy.loginfo('Starting the Object Avoidance node...')
    
    OA = Object_Avoidance()

    rospy.loginfo('The Object Avoidance node has started.')
    rospy.spin()
    #rospy.Rate(1/0.150)
