#! /usr/bin/env python3

import rospy
import serial.rs485
import serial
import time

from geometry_msgs.msg import Twist
from hunter_msgs.msg import HunterStatus

from door_server import door_Control
from signal_control import signal_Control

class deliveryBot(door_Control, signal_Control):
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

        ############ Timer for door actuation service and control ##########
        rospy.Timer(rospy.Duration(0.1), self.doorActuation_timer_callback)
        ############ Timer for braking and signals service and control #####
        rospy.Timer(rospy.Duration(0.1), self.signalControl_timer_callback)


        ###Creating Lights Always On From Boot Feature
        self.RR_on = 'FE 05 00 02 FF 00 39 F5'
        self.RR_off = 'FE 05 00 02 00 00 78 05'

        self.RL_on = 'FE 05 00 03 FF 00 68 35'
        self.RL_off = 'FE 05 00 03 00 00 29 C5'

        self.FR_on = 'FE 05 00 04 FF 00 D9 F4'
        self.FR_off = 'FE 05 00 04 00 00 98 04'

        self.FL_on = 'FE 05 00 05 FF 00 88 34'
        self.FL_off = 'FE 05 00 05 00 00 C9 C4'

        #Back Lights On
        self.serial_interface.write(bytearray.fromhex(self.RR_on))
        time.sleep(0.05)
        self.serial_interface.write(bytearray.fromhex(self.RL_on))
        time.sleep(0.05)
        self.serial_interface.write(bytearray.fromhex(self.FL_on))
        time.sleep(0.05)
        self.serial_interface.write(bytearray.fromhex(self.FR_on))
        time.sleep(0.05)
    


    def doorActuation_timer_callback(self, event):
        self.doorActuation(self.serial_interface)
    def signalControl_timer_callback(self,event):
        pass


if __name__ =='__main__':
    rospy.init_node('robot_control')
    rospy.loginfo('Starting the control node...')
    EVA = deliveryBot()

    rospy.loginfo('The control node has been started.')
    rospy.spin()
