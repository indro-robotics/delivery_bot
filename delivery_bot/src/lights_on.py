import time
import rospy
import serial
import serial.rs485


class Lights():
    def __init__(self):
        self.serial_interface = serial.Serial('/dev/ttyUSB0', 
            9600, timeout=0.05, 
            stopbits=serial.STOPBITS_ONE, 
            bytesize=serial.EIGHTBITS, 
            parity=serial.PARITY_NONE)
        
        self.RR_on = 'FE 05 00 02 FF 00 39 F5'
        self.RR_off = 'FE 05 00 02 00 00 78 05'

        self.RL_on = 'FE 05 00 03 FF 00 68 35'
        self.RL_off = 'FE 05 00 03 00 00 29 C5'

        self.FR_on = 'FE 05 00 04 FF 00 D9 F4'
        self.FR_off = 'FE 05 00 04 00 00 98 04'

        self.FL_on = 'FE 05 00 05 FF 00 88 34'
        self.FL_off = 'FE 05 00 05 00 00 C9 C4'
    def turn_on(self):
        self.serial_interface.write(bytearray.fromhex(self.RR_on))
        time.sleep(0.05)
        self.serial_interface.write(bytearray.fromhex(self.RL_on))
        time.sleep(0.05)
        self.serial_interface.write(bytearray.fromhex(self.FL_on))
        time.sleep(0.05)
        self.serial_interface.write(bytearray.fromhex(self.FR_on))
        time.sleep(0.05)
    def turn_off(self):
        self.serial_interface.write(bytearray.fromhex(self.RR_off))
        time.sleep(0.05)
        self.serial_interface.write(bytearray.fromhex(self.RL_off))
        time.sleep(0.05)
        self.serial_interface.write(bytearray.fromhex(self.FL_off))
        time.sleep(0.05)
        self.serial_interface.write(bytearray.fromhex(self.FR_off))
        time.sleep(0.05)
            # for i in range(10):
        #     self.serial_interface.write(bytearray.fromhex(self.RR_on))
        #     print('Trying RR Light')
        # for i in range(10):
        #     self.serial_interface.write(bytearray.fromhex(self.RL_on))
        # #Front Lights On
        # for i in range(10):
        #     self.serial_interface.write(bytearray.fromhex(self.FR_on))
        # for i in range(10):
        #     self.serial_interface.write(bytearray.fromhex(self.FL_on))
if __name__ == '__main__':
    light_control = Lights()
    light_control.turn_on()
