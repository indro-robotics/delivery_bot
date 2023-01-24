#! /usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
import serial
import serial.rs485
import time

from std_msgs.msg import Int16
from delivery_bot.srv import door


class door_Control:
    def __init__(self):

        #SETTING GPIO PARAMETERS
        self.IN1 = 22
        self.IN2 = 23
        self.curr_IN1 = GPIO.LOW
        self.curr_IN2 = GPIO.LOW
        self.ENABLE = 27
        self.open_flag = 0
        self.close_flag = 0

        #INITIALIZING GPIO PORTS
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.IN1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN2, GPIO.OUT, initial=GPIO.LOW) 
        GPIO.setup(self.ENABLE, GPIO.OUT, initial= GPIO.HIGH)

        #SETTING SERIAL PARAMETERS
        self.handle_flag = 0
        self.queue_command = 0
        self.doorPos = ''
        self.doorLatch_flag = 0 #0 = off, 1 = unlock, 2 = lock, 3 = complete
        self.doorPos_flag = 0 #0 = off, 1 = continuous polling, 2 = batch polling
        self.Pos_count = 0
        self.Latch_count = 0
        self.in_progFlag = 0

        self.boot_complete_flag = 0
        self.boot_count = 0
        self.boot_handle_flag = 0
        self.startup_flag = 1
        self.startup_count = 0

        #CLOSING THE DOOR FROM START
        GPIO.output(self.IN2, GPIO.HIGH)

        #SETTING LIGHT VARIABLES
        self.FL_on = 'FE 05 00 04 FF 00 D9 F4'
        self.FL_off = 'FE 05 00 04 00 00 98 04'

        self.FR_on = 'FE 05 00 05 FF 00 88 34'
        self.FR_off = 'FE 05 00 05 00 00 C9 C4'

        #INITIALIZING THE SERVER
        service = rospy.Service('/door_control', door, self.handle_door_control)

        #INITIALIZING SERIAL TIMERS
        self.doorPos_timer = rospy.Timer(rospy.Duration(0.5), self.doorPos_timer_callback)
        self.lights_timer = rospy.Timer(rospy.Duration(0.03), self.startup_lights_timer_callback)
        self.startup_timer = rospy.Timer(rospy.Duration(0.5), self.startup_timer_callback)
    
    def handle_door_control(self,cmd):
        if cmd.command == 1:
            self.queue_command += 1
            self.handle_flag = 1
            return 'Door toggled....'
        if cmd.command == 0:
            self.queue_command = 0
            self.handle_flag = 0
            return 'Door reset...'

    def doorActuation(self, serial_interface):
        self.serial_interface = serial_interface
        if self.handle_flag == 1 and self.queue_command == 1 and self.in_progFlag == 0:

            self.serial_interface.write(bytearray.fromhex('FE 02 00 00 00 06 EC 07'))
            self.doorPos = self.serial_interface.readline().hex()
            if self.doorPos == 'fe02010c9199' or self.doorPos == 'fe020101505c' or self.doorPos == 'fe020104905f':
                self.in_progFlag = 1
                self.prev_state = 0 #0 = previously closed - 1 = previously open
                rospy.loginfo('Door closed, opening...')
                self.open(serial_interface)
                return

            elif self.doorPos == 'fe020103d19d' or self.doorPos == 'fe020102105d':
                self.in_progFlag = 1
                self.prev_state = 1 #0 = previously closed - 1 = previously open
                rospy.loginfo('Door open, closing...')
                self.close(serial_interface)
                return
            else:
                self.in_progFlag = 1
                self.prev_state = 0 #0 = previously closed - 1 = previously open
                rospy.loginfo('Door in intermediate position, opening...')
                self.open(serial_interface)
                return
    
    def open(self,serial_interface):
        GPIO.output(self.IN1, GPIO.HIGH)
        return

    def close(self,serial_interface):
        GPIO.output(self.IN2, GPIO.HIGH)
        return

    def doorPos_timer_callback(self,event):
        if self.in_progFlag == 1:
            self.serial_interface.write(bytearray.fromhex('FE 02 00 00 00 06 EC 07'))
            self.doorPos = self.serial_interface.readline().hex()

            if self.doorPos =='fe02010c9199' or self.doorPos=='fe020104905f':
                if self.prev_state == 1:
                    rospy.loginfo('Door is fully closed.')
                    GPIO.output(self.IN2, GPIO.LOW)
                    self.queue_command = 0
                    self.handle_flag = 0
                    self.in_progFlag = 0
                    self.prev_state = 0
                    return
            
            if self.doorPos == 'fe020103d19d' and self.prev_state == 0:
                rospy.loginfo('Door is fully open.')
                GPIO.output(self.IN1, GPIO.LOW)
                self.queue_command = 0
                self.handle_flag = 0
                self.in_progFlag = 0
                self.prev_state = 1
                return

        if self.startup_flag == 1:
            self.serial_interface.write(bytearray.fromhex('FE 02 00 00 00 06 EC 07'))
            self.doorPos = self.serial_interface.readline().hex()
            if self.doorPos =='fe02010c9199' or self.doorPos=='fe020104905f':
                rospy.loginfo('Door is fully closed.')
                GPIO.output(self.IN2, GPIO.LOW)
                self.queue_command = 0
                self.handle_flag = 0
                self.startup_flag = 0
                self.boot_complete_flag = 1
                self.prev_state = 0
                return

    def startup_lights_timer_callback(self,event):
        if self.boot_complete_flag == 1:
            if self.boot_handle_flag == 1:
                if self.light_condition == 0:
                    if self.light_start == 0:
                        self.serial_interface.write(bytearray.fromhex(self.FL_on))
                        self.light_start = 1
                        return
                    if self.light_start == 1:
                        self.serial_interface.write(bytearray.fromhex(self.FR_on))
                        self.light_start = 0
                        self.toggle_Flag = 0
                        self.light_condition = 1
                        return
                if self.light_condition == 1:
                    if self.light_start == 0:
                        self.serial_interface.write(bytearray.fromhex(self.FL_off))
                        self.light_start = 1
                        return
                    if self.light_start == 1:
                        self.serial_interface.write(bytearray.fromhex(self.FR_off))
                        self.light_start = 0
                        self.light_condition = 0
                        self.boot_handle_flag = 0
                        return
    def startup_timer_callback(self,event):
        if self.boot_complete_flag == 1:
            if self.boot_count < 5:
                self.boot_handle_flag = 1
                self.boot_count += 1
                return
            if self.boot_count == 5:
                self.boot_complete_flag = 0
                self.boot_count = 0
                rospy.loginfo('BOOT COMPLETE')
                self.toggle_Flag = 1
                return self.toggle_Flag
