#!/usr/bin/env python
# this ROS NODE send via the serial port, the goal and position of the robot
# the protocol is one byte for the start char '@'
#               one bit for the identifier char 'G' for goal 'P'  for position
#               two bits of each of: 'x', 'y' and 'theta'
#               one bit for an end char '.'
# for a total of 9 bytes each message.
# it sends 'x' and 'y' in mm and 'theta' in degrees

import sys
import rospy
import serial
import math
from serial import SerialException
import struct
from struct import *
from std_msgs.msg import String
from geometry_msgs.msg import *
from multi_robots.msg import *

# recived pose in m, send pose in mm
scale = 1000 # 1 m = 1000 mm
scaleAngle = 180/math.pi # to degrees
N_BYTES = 2 # number of bytes to send for each variable, this case int16
N_BITS = N_BYTES * 8
ser = 0 # serial comunication

def setMaxMin( x, Max, Min ): # set x in the ]min,max[ interval
    if( x > Max ):
	x = Max - 1
    if( x < Min ):
        x = Min + 1
    return x

def sendGoal( goal ):
    #convertion with 'scale'
    #limits 16bits
    x = setMaxMin( int( goal.x * scale ) , 2**(N_BITS-1)-1, -2**(N_BITS-1) )
    #works for signed variables
    y = setMaxMin(  int( goal.y * scale ), 2**(N_BITS-1)-1, -2**(N_BITS-1) )
    theta = setMaxMin(  int( goal.theta * scaleAngle ), 2**(N_BITS-1)-1, -2**(N_BITS-1) )

    ser.write( '@G' ) #init message
    ser.write( struct.pack('hhh', x, y, theta) ) # sending int16 int16 int16
    ser.write( '.' ) #end message
    rospy.loginfo( "writing goal %s + %s + %s\n",str(x),str(y), str(theta) )


def sendPose(pose):

    #convertion with 'scale'
    #limits 16bits
    x = setMaxMin( int( pose.x * scale ) , 2**(N_BITS-1)-1, -2**(N_BITS-1) )
    #works for signed variables
    y = setMaxMin(  int( pose.y * scale ), 2**(N_BITS-1)-1, -2**(N_BITS-1) )
    theta = setMaxMin(  int( pose.theta * scaleAngle ), 2**(N_BITS-1)-1, -2**(N_BITS-1) )

    ser.write( '@P' ) #init message
    ser.write( struct.pack('hhh', x, y, theta) ) # sending int16 int16 int16
    ser.write( '.' ) #end message
    rospy.loginfo("writing pos %s + %s + %s\n",str(x),str(y), str(theta) )

def shutDown():
    ser.close()

def sender():

    rospy.init_node('Sender' , anonymous=False)
    rospy.on_shutdown( shutDown )
    # init parameters
    port = rospy.get_param('~port', '/dev/ttyUSB0' )
    baudRate = rospy.get_param('~baudRate', 9600 )
    robotID = rospy.get_param('~id', 1 )

    global ser
    try:
        ser = serial.Serial( port=port, baudrate=baudRate, bytesize=8, parity='N', stopbits=1, timeout=6, xonxoff=0, rtscts=0 )
    except serial.SerialException:
        print("Unable to set communication, verify PORT")
        exit();
    print("Talking in " + port + " at " + str(baudRate) + "\n")

    rospy.Subscriber( "goal_" + str( robotID ) , Pose2D, sendGoal )
    rospy.Subscriber( "pose_" + str( robotID ) , Pose2D, sendPose )
    rospy.spin()

if __name__ == '__main__':
    try:
        sender()
    except rospy.ROSInterruptException:
        pass
