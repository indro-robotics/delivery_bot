#! /usr/bin/env python3

import rospy
import serial.rs485
import serial
from geometry_msgs.msg import Twist
from hunter_msgs.msg import HunterStatus
from std_msgs.msg import Int16

from door_server import door_Control
from signal_control import signal_Control
from lights_server import light_Control

class deliveryBot(door_Control, signal_Control, light_Control):
    def __init__(self):
        #INITIALIZING THE SERIAL INTERFACE
        self.serial_interface = serial.Serial('/dev/ttyUSB0', 
            9600, timeout=0.05, 
            stopbits=serial.STOPBITS_ONE, 
            bytesize=serial.EIGHTBITS, 
            parity=serial.PARITY_NONE)
        
        #INITIALIZING ROS NODES
        door_Control.__init__(self)
        signal_Control.__init__(self)
        light_Control.__init__(self)

        ############ Timer for door actuation service and control ##########
        rospy.Timer(rospy.Duration(0.1), self.doorActuation_timer_callback)
        ############ Timer for light toggling control ######################
        rospy.Timer(rospy.Duration(0.1), self.lightControl_timer_callback)

    def doorActuation_timer_callback(self, event):
        self.doorActuation(self.serial_interface)
    
    def lightControl_timer_callback(self,event):
        self.lights_Toggle(self.serial_interface)

if __name__ =='__main__':
    rospy.init_node('robot_control')
    rospy.loginfo('Starting the control node...')
    EVA = deliveryBot()

    rospy.loginfo('The control node has been started.')
    rospy.spin()
