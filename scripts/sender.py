#!/usr/bin/env python

## Sends messages "@ @ data data ." through serial port
## from the 'sendToID' topic

import sys
import rospy
import serial
from serial import SerialException
import struct
from struct import *
from std_msgs.msg import String
from multi_robots.msg import Control_action

# recived pose in m, send pose in mm
scale = 1000 # 1 m = 1000 mm
N_BYTES = 2 # number of bytes to send for each variable, this case int16
N_BITS = N_BYTES * 8
ser = 0 # serial comunication

def setMaxMin( x, Max, Min ): # set x in the ]min,max[ interval
    if( x > Max ):
	x = Max - 1
    if( x < Min ):
        x = Min + 1
    return x

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + " I heard %s %s", data.v_right, data.v_left)
    #convertion with 'scale'
    #limits 16bits
    velocity_right = setMaxMin( int( data.v_right * scale ) , 2**(N_BITS-1)-1, -2**(N_BITS-1) )
    #works for signed variables
    velocity_left = setMaxMin(  int( data.v_left * scale ), 2**(N_BITS-1)-1, -2**(N_BITS-1) )
    ser.write( '@@' ) #init message
    ser.write( struct.pack('hh', velocity_right, velocity_left) ) # sending int16 int16
    ser.write( '.' ) #end message
    rospy.loginfo("writing %s + %s\n",str(velocity_right),str(velocity_left) )

def shutDown():
    ser.close()

def sender(robotID, port, baudRate):

    rospy.init_node('sender', anonymous=True)
    rospy.on_shutdown( shutDown )
    global ser
    try:
        ser = serial.Serial( port=port, baudrate=baudRate, bytesize=8, parity='N', stopbits=1, timeout=6, xonxoff=0, rtscts=0 )
    except serial.SerialException:
        print("Unable to set communication, verify PORT")
        exit();
    print("Talking in " + port + " at " + str(baudRate) + "\n")
    rospy.Subscriber( "sendTo_" + str( robotID ) , Control_action, callback )
    rospy.spin()

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("usage: sender.py ID Port BaudRate --default--> 01 /dev/ttyUSB0 9600 ")
	sender( 01, '/dev/ttyUSB0', 9600)
    else:
        if len(sys.argv) < 4:
            sender( int(sys.argv[1]), sys.argv[2], 9600 )
        else:
            sender( int(sys.argv[1]), sys.argv[2], int(sys.argv[3]) )
