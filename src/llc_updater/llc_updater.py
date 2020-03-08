#!/usr/bin/env python

import rospy
import serial
import struct
from sensor_msgs.msg import Joy

SERIAL_HANDSHAKE = 0xFB
SERIAL_PACKET_START = 0xFA
STEER_RANGE = (65, 115)
THROTTLE_RANGE = (90, 103)
JS_AXIS_RANGE = (1, -1)
MAP_FACTOR = float(STEER_RANGE[1] - STEER_RANGE[0])/\
    float(JS_AXIS_RANGE[1] - JS_AXIS_RANGE[0])

def map_val(val):
    return int((val - JS_AXIS_RANGE[0]) * MAP_FACTOR + STEER_RANGE[0])

class LLCUpdater(object): 
    def __init__(self):
        rospy.init_node('llc_updater')
        rospy.Subscriber('/joy', Joy, self.joy_cbk)

        self.serial = serial.Serial('/dev/ttyACM0', 9600)
        self.speed = 90
        self.steer = 90
        while ord(self.serial.read()) != SERIAL_HANDSHAKE:
            rospy.loginfo('Waiting for handshake')

        rospy.loginfo('Serial handshake received!')
        self.update()   
    
        rospy.spin()

    def update(self):
        self.serial.write(chr(SERIAL_PACKET_START))
        self.serial.write(chr(self.speed))
        self.serial.write(chr(self.steer))

    def joy_cbk(self, msg):
        #rospy.loginfo('Axes: {0}\n Buttons:{1}'.format(msg.axes, msg.buttons))
        self.steer = map_val(msg.axes[0])
        self.speed = THROTTLE_RANGE[msg.buttons[6]]
        self.update()

if __name__ == '__main__':
    try:
        LLCUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Couldn\'t start LLC Updater node') 
