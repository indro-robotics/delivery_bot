#! /usr/bin/env python3

import time
import rospy
import serial
import serial.rs485

from geometry_msgs.msg import Twist
from hunter_msgs.msg import HunterStatus
from std_msgs.msg import Int16

class signal_Control():
    def __init__(self):
        
        #INITIALIZING SERIAL 
        self.BL_on = 'FE 05 00 02 FF 00 39 F5'
        self.BL_off = 'FE 05 00 02 00 00 78 05'

        self.BR_on = 'FE 05 00 03 FF 00 68 35'
        self.BR_off = 'FE 05 00 03 00 00 29 C5'

        self.FL_on = 'FE 05 00 04 FF 00 D9 F4'
        self.FL_off = 'FE 05 00 04 00 00 98 04'

        self.FR_on = 'FE 05 00 05 FF 00 88 34'
        self.FR_off = 'FE 05 00 05 00 00 C9 C4'
        self.cmd_vel = Twist()
        self.hunter_status = HunterStatus()


        #Initializing signalling flags
        self.right_signal_flag = 0 #0 = off, 1 = on
        self.left_signal_flag = 0 #0 = off, 1 = on
        self.off_count = 0 #So that only turn them off once
        self.lightPos = 0 #0 is neither - 1 is right - 2 is left
        self.lightON = 0
        self.sig_count = 0 #To know to only check subscriber once at start
        self.lightCond = 0 #Initializing lightCond

        #Initializing braking flags
        self.brake_count = 0
        self.light_start = 0 #0 = BL 1 = BR

        self.cmdVel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmdVel_callback)
        self.hunterStatus_sub = rospy.Subscriber('/hunter_status', HunterStatus, self.hunterStatus_callback)
        self.lightCond_sub = rospy.Subscriber('/light_Cond', Int16, self.lightCond_callback)

        #Initializing the signal timers
        self.signals_timer = rospy.Timer(rospy.Duration(0.025), self.signals_timer_callback)
        self.left_light_timer = rospy.Timer(rospy.Duration(0.5), self.left_light_timer_callback)
        self.right_light_timer = rospy.Timer(rospy.Duration(0.5), self.right_light_timer_callback)
        self.brake_timer = rospy.Timer(rospy.Duration(0.03), self.brake_timer_callback)

    def hunterStatus_callback(self, msg):
        self.hunter_status = msg
        self.velocity_last = self.hunter_status.linear_velocity

    def cmdVel_callback(self,msg):
        self.cmd_vel = msg

    def lightCond_callback(self, msg):
        self.lightCond = msg.data
    
    def signals_timer_callback(self,event):
        if self.cmd_vel.angular.z <= -0.2: #If robot is turning right
            self.off_count = 0 #Has not been triggered off
            self.lightPos = 1 # 0 is neither - 1 is right - 2 is left
            self.right_signal_flag = 1

            if self.lightCond == 0 and self.sig_count == 0: #if light initially off
                self.lightON = 0 #Light is off
                self.sig_count += 1
                return self.right_signal_flag, self.off_count, self.lightPos, self.lightON, self.sig_count
            if self.lightCond == 1 and self.sig_count == 0: #if light initially on
                self.lightON = 1 # Light is on
                self.sig_count += 1
                return self.right_signal_flag, self.off_count, self.lightPos, self.lightON, self.sig_count

        if self.cmd_vel.angular.z >= 0.2:
            self.off_count = 0 #Has not been triggered off
            self.lightPos = 2 # 0 is neither - 1 is right - 2 is left
            self.left_signal_flag = 1

            if self.lightCond == 0 and self.sig_count == 0: #if light initially off
                self.lightON = 0 #Light is off
                self.sig_count += 1
                return self.left_signal_flag, self.off_count, self.lightPos, self.lightON, self.sig_count
            if self.lightCond == 1 and self.sig_count == 0: #if light initially on
                self.lightON = 1 #Light is on
                self.sig_count += 1
                return self.left_signal_flag, self.off_count, self.lightPos, self.lightON, self.sig_count

        if -0.2 < self.cmd_vel.angular.z < 0.2:
            self.right_signal_flag = 0 #RESET FLAGS
            self.left_signal_flag = 0 #RESET FLAGS
            self.sig_count = 0 #Reset count for lightCond and lightON
            if self.lightCond == 0 and self.off_count <10:
                if self.lightPos == 1: #if right signal was toggled
                    self.serial_interface.write(bytearray.fromhex(self.FR_off)) #turn off front right light
                    self.off_count += 1
                    return self.off_count

                if self.lightPos == 2: #if left signal was toggled
                    self.serial_interface.write(bytearray.fromhex(self.FL_off))
                    self.off_count += 1
                    return self.off_count
            if self.lightCond == 1 and self.off_count < 10:
                if self.lightPos == 1: #if right signal was toggled
                    self.serial_interface.write(bytearray.fromhex(self.FR_on)) #turn off front right light
                    self.off_count += 1
                    return self.off_count

                if self.lightPos == 2: #if left signal was toggled
                    self.serial_interface.write(bytearray.fromhex(self.FL_on))
                    self.off_count += 1
                    return self.off_count

    def left_light_timer_callback(self,event):
        if self.left_signal_flag ==1:
            if self.lightON == 0: #If light is off
                self.serial_interface.write(bytearray.fromhex(self.FL_on)) #Turn it on
                self.lightON = 1 #Light is now on
                rospy.logwarn('Signalling Left...')
                return self.lightON
            if self.lightON == 1: #If light is on
                self.serial_interface.write(bytearray.fromhex(self.FL_off)) #Turn it off
                self.lightON = 0 #Light is now off
                return self.lightON
        else:
            return

    def right_light_timer_callback(self, event):
        if self.right_signal_flag ==1:
            if self.lightON == 0: #If light is off
                self.serial_interface.write(bytearray.fromhex(self.FR_on)) #Turn it on
                self.lightON = 1 #Light is now on
                rospy.logwarn('Signalling Right...')
                return self.lightON
            if self.lightON == 1: #If light is on
                self.serial_interface.write(bytearray.fromhex(self.FR_off)) #Turn it off
                self.lightON = 0 #Light is now off
                return self.lightON
        else:
            return
    
    def brake_timer_callback(self,event):
        if self.cmd_vel.linear.x == 0: #If robot is stationary or reversing
            if self.lightCond == 0 and self.brake_count == 0: #If the lights are turned off
                if self.light_start == 0:
                    self.serial_interface.write(bytearray.fromhex(self.BL_on))
                    self.light_start = 1
                    return
                if self.light_start == 1:
                    self.serial_interface.write(bytearray.fromhex(self.BR_on))
                    self.light_start = 0
                    self.brake_count = 1
                    rospy.logwarn('Braking...')
                    return
            if self.lightCond == 1:
                self.brake_count = 0
                return
        if self.cmd_vel.linear.x != 0:
            if self.lightCond == 0 and self.brake_count == 1:
                if self.light_start == 0:
                    self.serial_interface.write(bytearray.fromhex(self.BL_off))
                    self.light_start = 1
                    return
                if self.light_start == 1:
                    self.serial_interface.write(bytearray.fromhex(self.BR_off))
                    self.light_start = 0
                    self.brake_count = 0
                    rospy.logwarn('Driving...')
                    return