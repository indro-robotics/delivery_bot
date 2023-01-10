#! /usr/bin/env python3

import time
import rospy
import serial
import serial.rs485

from geometry_msgs.msg import Twist
from hunter_msgs.msg import HunterStatus

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
        self.Rcycle_flag = 0 #0 = havent completed cycle, 1 = have completed cycle
        self.Lcycle_flag = 0
        self.signals_off_flag = 0 #signals are originally off
        self.stop_sigs = 0
        self.FR_flag = 0
        self.RR_flag = 0
        self.FL_flag = 0
        self.RL_flag = 0
        self.off_count = 0

        self.brakeCount = 0



        self.cmdVel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmdVel_callback)
        self.hunterStatus_sub = rospy.Subscriber('/hunter_status', HunterStatus, self.hunterStatus_callback)

        #Initializing the signal timers
        self.signals_timer = rospy.Timer(rospy.Duration(0.5), self.signals_timer_callback)
        self.left_light_timer = rospy.Timer(rospy.Duration(0.035), self.left_light_timer_callback)
        self.right_light_timer = rospy.Timer(rospy.Duration(0.035), self.right_light_timer_callback)
        self.stop_sigs_timer = rospy.Timer(rospy.Duration(0.05), self.stop_sigs_callback)
        #self.brake_lights_timer_callback = rospy.Timer(rospy.Duration(0.05), self.brake_lights_timer_callback)

    def hunterStatus_callback(self, msg):
        self.hunter_status = msg
        self.velocity_last = self.hunter_status.linear_velocity
    def cmdVel_callback(self,msg):
        self.cmd_vel = msg
    
    def signals_timer_callback(self,event):
            #flag for when signals should be on:
            #print('RR_flag: {} , RL_flag: {}, FR_flag: {}, FL_flag: {}'.format(self.RR_flag, self.RL_flag, self.FR_flag, self.FL_flag))
            if self.cmd_vel.angular.z <= -0.2: #If robot is turning right
                self.off_count = 0
                if self.Rcycle_flag == 0: #if a cycle hasn't been completed --> turn on signals
                    self.right_signal_flag = 1
                    return self.right_signal_flag, self.off_count
                if self.Rcycle_flag == 1:
                    self.right_signal_flag = 0
                    return self.right_signal_flag, self.off_count

            if self.cmd_vel.angular.z >= 0.2:
                self.off_count = 0
                if self.Lcycle_flag == 0:
                    self.left_signal_flag = 1
                    return self.left_signal_flag, self.off_count
                if self.Lcycle_flag == 1:
                    self.left_signal_flag = 0
                    return self.left_signal_flag, self.off_count
            
    def stop_sigs_callback(self,event):
        if self.cmd_vel.angular.z >-0.2 and self.cmd_vel.angular.z < 0.2 and self.cmd_vel.linear.x > 0:
            if self.RR_flag == 1:
                self.off_count += 1
                self.serial_interface.write(bytearray.fromhex(self.RR_off))
                if self.off_count == 5:
                    self.RR_flag = 0
                    self.off_count = 0
                    return
            if self.RL_flag == 1:
                self.serial_interface.write(bytearray.fromhex(self.RL_off))
                self.off_count += 1
                if self.off_count == 5:
                    self.RL_flag = 0
                    self.off_count = 0
                    return
            if self.FR_flag == 1:
                self.serial_interface.write(bytearray.fromhex(self.FR_off))
                self.off_count += 1
                if self.off_count == 5:
                    self.FR_flag = 0
                    self.off_count = 0
                    return
            if self.FL_flag == 1:
                self.serial_interface.write(bytearray.fromhex(self.FL_off))
                self.off_count += 1
                if self.off_count == 5:
                    self.FL_flag = 0
                    self.off_count = 0
                    return
            else:
                pass

    def left_light_timer_callback(self,event):
        #Turning left signals on
        if self.left_signal_flag == 1 and self.Lcycle_flag == 0:
            self.stop_sigs = False
            if self.RL_flag == 0 and self.FL_flag == 0: # if RR is off and FR is off
                self.serial_interface.write(bytearray.fromhex(self.RL_on)) # turn RR light on
                self.RL_flag = 1
                return self.RL_flag, self.stop_sigs

            if self.RL_flag == 1 and self.FL_flag == 0: #if RR is on and FR is off
                self.serial_interface.write(bytearray.fromhex(self.FL_on))
                self.FL_flag = 1 # designate the FR to be on
                return self.FL_flag, self.stop_sigs

            if self.RL_flag == 1 and self.FL_flag == 1: # if RR and FR are both on
                self.Lcycle_flag = 1
                return self.Lcycle_flag, self.stop_sigs

        if self.left_signal_flag == 0 and self.Lcycle_flag == 1:
            if self.RL_flag == 1 and self.FL_flag == 1:
                self.serial_interface.write(bytearray.fromhex(self.RL_off))
                self.RL_flag = 0
                return self.RL_flag, self.stop_sigs

            if self.RL_flag == 0 and self.FL_flag == 1:
                self.serial_interface.write(bytearray.fromhex(self.FL_off))
                self.FL_flag =0
                return self.FL_flag, self.stop_sigs
                
            if self.RL_flag == 0 and self.FL_flag == 0:
                self.Lcycle_flag = 0
                return self.Lcycle_flag, self.stop_sigs

    def right_light_timer_callback(self, event):
        if self.right_signal_flag == 1 and self.Rcycle_flag == 0:
            self.stop_sigs = False
            if self.RR_flag == 0 and self.FR_flag == 0: # if RR is off and FR is off
                self.serial_interface.write(bytearray.fromhex(self.RR_on)) # turn RR light on
                self.RR_flag = 1
                return self.RR_flag, self.stop_sigs

            if self.RR_flag == 1 and self.FR_flag == 0: #if RR is on and FR is off
                self.serial_interface.write(bytearray.fromhex(self.FR_on))
                self.FR_flag = 1 # designate the FR to be on
                return self.FR_flag, self.stop_sigs


            if self.RR_flag == 1 and self.FR_flag == 1: # if RR and FR are both on
                self.Rcycle_flag = 1
                return self.Rcycle_flag, self.stop_sigs


        if self.right_signal_flag == 0 and self.Rcycle_flag == 1:
            if self.RR_flag == 1 and self.FR_flag == 1:
                self.serial_interface.write(bytearray.fromhex(self.RR_off))
                self.RR_flag = 0
                return self.RR_flag, self.stop_sigs


            if self.RR_flag == 0 and self.FR_flag == 1:
                self.serial_interface.write(bytearray.fromhex(self.FR_off))
                self.FR_flag =0
                return self.FR_flag, self.stop_sigs

                
            if self.RR_flag == 0 and self.FR_flag == 0:
                self.Rcycle_flag = 0
                return self.Rcycle_flag, self.stop_sigs
    # def brake_lights_timer_callback(self,event):
    #     if self.cmd_vel.linear.x <= 0:
    #         #print('got here')
    #         if self.brakeCount == 0:
    #             if self.RR_flag == 0:
    #                 self.serial_interface.write(bytearray.fromhex(self.RR_on))
    #                 self.RR_flag = 1
    #                 return self.RR_flag
    #             if self.RL_flag == 0:
    #                 self.serial_interface.write(bytearray.fromhex(self.RL_on))
    #                 self.RL_flag = 1
    #                 return self.RL_flag
    #             if self.RR_flag == 1 and self.RL_flag == 1:
    #                 self.brakeCount += 1
    #                 return self.brakeCount
    #     else:
    #         print('got here')
    #         if self.RR_flag == 1:
    #             self.serial_interface.write(bytearray.fromhex(self.RR_off))
    #             self.RR_flag = 0
    #             self.brakeCount = 0
    #             return self.brakeCount
    #         if self.RL_flag == 1:
    #             self.serial_interface.write(bytearray.fromhex(self.RL_off))
    #             self.RL_flag = 0
    #             self.brakeCount = 0
    #             return self.brakeCount
    #         if self.RR_flag == 0 and self.RL_flag == 0:
    #             self.brakeCount = 0
    #             return self.brakeCount
