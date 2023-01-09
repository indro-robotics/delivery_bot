#! /usr/bin/env python3

import rospy

from std_msgs.msg import Int16
from delivery_bot.srv import lights
from delivery_bot.srv import door

class light_Control():
    def __init__(self):

        #SETTING SERIAL COMMANDS
        self.BL_on = 'FE 05 00 02 FF 00 39 F5'
        self.BL_off = 'FE 05 00 02 00 00 78 05'

        self.BR_on = 'FE 05 00 03 FF 00 68 35'
        self.BR_off = 'FE 05 00 03 00 00 29 C5'

        self.FL_on = 'FE 05 00 04 FF 00 D9 F4'
        self.FL_off = 'FE 05 00 04 00 00 98 04'

        self.FR_on = 'FE 05 00 05 FF 00 88 34'
        self.FR_off = 'FE 05 00 05 00 00 C9 C4'

        #SETTING FLAGS
        self.light_handle_flag = 0
        self.light_queue_command = 0
        self.light_start = 0 #LIGHT NUMBERING: BL=0 BR=1 FL=2 FR=3
        self.light_condition = 0 #0 = lights off 1 = lights on
        self.toggle_Flag = 0

        #INITIALIZING THE SERVER

        service = rospy.Service('/light_control', door, self.handle_light_control)

        self.lights_timer = rospy.Timer(rospy.Duration(0.025), self.lights_timer_callback)
    def handle_light_control(self,cmd):
        if cmd.command == 1: #If Lights are set to be toggled
            self.light_queue_command += 1
            self.light_handle_flag = 1
            self.light_start = 0
            return 'Lights toggled...'
        if cmd.command == 0:
            self.light_queue_command = 0
            self.light_handle_flag = 0
            return 'Lights reset...'

    def lights_Toggle(self,serial_interface):
        self.serial_interface = serial_interface
        if self.light_handle_flag == 1 and self.light_queue_command == 1:
            self.toggle_Flag = 1
            #self.light_start = 0

    def lights_timer_callback(self,event):
        if self.toggle_Flag == 1:
            if self.light_condition == 0: #Want to turn the lights on
                if self.light_start == 0:
                    self.serial_interface.write(bytearray.fromhex(self.BL_on))
                    self.light_start = 1
                    return
                if self.light_start == 1:
                    self.serial_interface.write(bytearray.fromhex(self.BR_on))
                    self.light_start = 2
                    return
                if self.light_start == 2:
                    self.serial_interface.write(bytearray.fromhex(self.FL_on))
                    self.light_start = 3
                    return
                if self.light_start == 3:
                    self.serial_interface.write(bytearray.fromhex(self.FR_on))
                    self.light_start = 0
                    self.toggle_Flag = 0
                    self.light_condition = 1

                    self.light_queue_command = 0
                    self.light_handle_flag = 0
                    rospy.loginfo('Lights toggled on.')
                    return


            if self.light_condition == 1:
                if self.light_start == 0:
                    self.serial_interface.write(bytearray.fromhex(self.BL_off))
                    self.light_start = 1
                    return
                if self.light_start == 1:
                    self.serial_interface.write(bytearray.fromhex(self.BR_off))
                    self.light_start = 2
                    return
                if self.light_start == 2:
                    self.serial_interface.write(bytearray.fromhex(self.FL_off))
                    self.light_start = 3
                    return
                if self.light_start == 3:
                    self.serial_interface.write(bytearray.fromhex(self.FR_off))
                    self.light_start = 0
                    self.toggle_Flag = 0
                    self.light_condition = 0

                    self.light_queue_command = 0
                    self.light_handle_flag = 0

                    rospy.loginfo('Lights toggled off.')
                    return
                
                return
