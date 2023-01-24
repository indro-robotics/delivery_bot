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
        self.open_flag = 0
        self.close_flag = 0

        #INITIALIZING GPIO PORTS
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.IN1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN2, GPIO.OUT, initial=GPIO.LOW)

        #SETTING SERIAL PARAMETERS
        self.handle_flag = 0
        self.queue_command = 0
        self.doorPos = ''
        self.doorLatch_flag = 0 #0 = off, 1 = unlock, 2 = lock, 3 = complete
        self.doorPos_flag = 0 #0 = off, 1 = continuous polling, 2 = batch polling
        self.Pos_count = 0
        self.Latch_count = 0

        #INITIALIZING THE SERVER
        service = rospy.Service('/door_control', door, self.handle_door_control)

        #INITIALIZING SERIAL TIMERS
        self.doorPos_timer = rospy.Timer(rospy.Duration(0.1), self.doorPos_timer_callback)
        self.doorLatch_timer = rospy.Timer(rospy.Duration(0.15), self.doorLatch_timer_callback)
    
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
        print(self.doorPos)
        if self.handle_flag == 1 and self.queue_command == 1:
            self.doorPos_flag = 2
            while True:
                if self.doorPos_flag == 3:
                    self.doorPos_flag = 1
                    if self.doorPos == 'fe020101505c':
                        rospy.loginfo('Door closed, opening...')
                        self.open(serial_interface)
                        break

                    elif self.doorPos == 'fe020102105d':
                        rospy.loginfo('Door open, closing...')
                        self.close(serial_interface)
                        break
                    else:
                        rospy.loginfo('Door in intermediate position, opening...')
                        self.open(serial_interface)
                        break



    
    def open(self,serial_interface):
        print('Attempting to open')
        self.doorLatch_flag = 1
        while True:
            if self.doorLatch_flag == 3:
                GPIO.output(self.IN1, GPIO.HIGH)
                if self.doorPos =='fe020102105d':
                    GPIO.output(self.IN1, GPIO.LOW)
                    rospy.loginfo('Door is fully open\n')
                    self.doorPos_flag = 0
                    
                    self.handle_flag = 0
                    self.queue_command = 0

                    self.doorLatch_flag = 4
                    break
  
        return
    def close(self,serial_interface):

        self.doorLatch_flag = 1
        while True:
            if self.doorLatch_flag == 3:
                GPIO.output(self.IN2, GPIO.HIGH)
                if self.doorPos =='fe020101505c':
                    GPIO.output(self.IN2, GPIO.LOW)
                    rospy.loginfo('Door is fully closed\n')
                    self.doorPos_flag = 0

                    self.handle_flag = 0
                    self.queue_command = 0

                    self.doorLatch_flag = 4
                    break
        return


    def doorLatch_timer_callback(self,event):
        if self.doorLatch_flag == 1: #If the request is to unlock the latch
            self.Latch_count += 1
            self.serial_interface.write(bytearray.fromhex('FE 05 00 00 FF 00 98 35'))
            if self.Latch_count == 10:
                self.doorLatch_flag = 3
                self.Latch_count = 0
                return
        if self.doorLatch_flag == 2: #If the request is to lock the latch
            self.Latch_count += 1
            self.serial_interface.write(bytearray.fromhex('FE 05 00 00 00 00 D9 C5'))
            if self.Latch_count == 10:
                self.doorLatch_flag = 3
                self.Latch_count = 0
                return
        if self.doorLatch_flag == 4: #If the request is to lock the latch
            self.Latch_count += 1
            self.serial_interface.write(bytearray.fromhex('FE 05 00 00 00 00 D9 C5'))
            if self.Latch_count == 10:
                
                self.doorLatch_flag = 0 #0 = off, 1 = unlock, 2 = lock, 3 = complete
                self.doorPos_flag = 0 #0 = off, 1 = continuous polling, 2 = batch polling
                self.Pos_count = 0
                self.Latch_count = 0

    def doorPos_timer_callback(self,event):
        if self.doorPos_flag == 1:
            self.serial_interface.write(bytearray.fromhex('FE 02 00 00 00 06 EC 07'))
            self.doorPos = self.serial_interface.readline().hex()
            
        if self.doorPos_flag == 2:
            self.Pos_count += 1
            self.serial_interface.write(bytearray.fromhex('FE 02 00 00 00 06 EC 07'))
            self.doorPos = self.serial_interface.readline().hex()
            if self.Pos_count == 20:
                self.doorPos_flag = 3
                self.Pos_count = 0
                return



        
        
