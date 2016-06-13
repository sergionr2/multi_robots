#!/usr/bin/env python
##

import rospy
import math
import random
from geometry_msgs.msg import *
from multi_robots.msg import *
from std_msgs.msg import *
from visualization_msgs.msg import *

MAXTIME = 1 #seconds, time to unable if not visible
N = 20 #number of points to calcul velocity
HZ = 15 #frecuence of publish 15hz
URL_OBJS = '/home/multi-robots/catkin_ws/src/multi_robots/GPSconfig/objects.txt'
URL_MAP = '/home/multi-robots/catkin_ws/src/multi_robots/GPSconfig/map.txt'
GEOM ='0.0 , 0.0'
INIT_X = 2
INIT_Y = 0
INIT_THETA = math.pi /4
FRAME = 'World'

robotGeom = {} # int : Point[]
robotLastPoses = {} # int : RobotPose[N]
idList = [] # IDs
#TODO srv set Noise
A = 0 # Noise Amplitud

def readRobotGeom():
    f = open ( URL_OBJS, 'r')
    l = [ map( str, line.split(',') ) for line in f ]
    geoms = dict( ( int(i[0]), getPolygon(i[1].rstrip('\n')) ) for i in l )
    ids = [ int(i[0]) for i in l ]
    lastPoses = dict( ( i, [] ) for i in ids ) #init robotLastPoses
    for row in l:
        if( int(row[0]) < 0 ):
            r = RobotPose()
            r.id = int(row[0])
            r.pose = Pose2D( float( row[2]),float(row[3]),float( row[4].rstrip('\n') ) )
            lastPoses[ r.id ].insert(0,r)
    return ids, geoms, lastPoses

def gps():
    rospy.init_node('gps', anonymous=False)
    rospy.on_shutdown( shutDown )
    topicList = initTopics( idList )
    rospy.Subscriber( "robot_pose" , RobotPose, setRobotPose )
    rate = rospy.Rate( HZ )
    while not rospy.is_shutdown():

        publish( topicList, keys, mapMatrix )
        rate.sleep()

if __name__ == '__main__':
    try:
        gps()
    except rospy.ROSInterruptException:
        pass
