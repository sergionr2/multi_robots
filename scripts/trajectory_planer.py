#!/usr/bin/env python

import rospy
from geometry_msgs.msg import *
from multi_robots.msg import *
from std_msgs.msg import *
from visualization_msgs.msg import *

HZ = 0.5 #frecuence of publish reference in hz
ID = 6
FRAME = 'Local_'+str(ID)

def getInfo( data ):
    print ID

def planer():
    pubGoal = rospy.Publisher('goal_'+str(ID), Pose2D, queue_size=10)
    pubPose = rospy.Publisher('pose_'+str(ID), Pose2D, queue_size=10)
    rospy.init_node('Trajectory_Planer_'+str(ID), anonymous=False)
    rate = rospy.Rate( HZ ) # 10hz
    rospy.Subscriber( "info_"+str(ID) , GPSinfo, getInfo )
    while not rospy.is_shutdown():

        pubGoal.publish()
        rate.sleep()

if __name__ == '__main__':
    try:
        planer()
    except rospy.ROSInterruptException:
        pass
