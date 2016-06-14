#!/usr/bin/env python

import rospy
from geometry_msgs.msg import *
from multi_robots.msg import *
from std_msgs.msg import *
from visualization_msgs.msg import *

HZ = 0.5 #frecuence of publish reference in hz
ID = 6
FRAME = 'Local_'+str(ID)

def translatePoint( point3D, pose2D ):#rotation en Z
    x = point3D.x
    y = point3D.y
    z = point3D.z

    theta = point3D.x
    newPoint = Point( 0, 0, 0 )
    #rotate
    newPoint.x = x*math.cos( pose2D.theta  ) - y*math.sin( pose2D.theta )
    newPoint.y = x*math.sin( pose2D.theta  ) + y*math.cos( pose2D.theta )
    #translate
    newPoint.x += pose2D.x
    newPoint.y += pose2D.y
    return newPoint #point3D

def printData( data ):

    pub = rospy.Publisher( 'local_'+str(ID), MarkerArray, queue_size=10 )
    markers = []
    cont = 0
    m = Marker()
    m.action = 3 # erase all
    markers.append( m )
    cont += 1

    m = Marker()
    m.header.frame_id = FRAME
    m.ns = "obstacles"
    m.id = cont
    m.type = m.SPHERE_LIST
    m.action = 0 # Add/Modify
    m.scale.x = 0.02
    m.scale.y = 0.02
    m.scale.z = 0.02
    m.color.a = 1.0
    m.color.r = 255.0/255
    m.color.g = 0/255
    m.color.b = 0/255
    m.points = data.obstacles
    cont += 1
    markers.append( m )



    pub.publish( markers )

def getInfo( data ):
    printData( data )


def setGoal():
    return Pose2D( 1,1,0 )

def planer():
    pubGoal = rospy.Publisher('goal_'+str(ID), Pose2D, queue_size=10)
    pubPose = rospy.Publisher('pose_'+str(ID), Pose2D, queue_size=10)
    rospy.init_node('Trajectory_Planer_'+str(ID), anonymous=False)
    rate = rospy.Rate( HZ ) # 10hz
    rospy.Subscriber( "info_"+str(ID) , GPSinfo, getInfo )
    while not rospy.is_shutdown():

        pubGoal.publish( setGoal() )
        rate.sleep()

if __name__ == '__main__':
    try:
        planer()
    except rospy.ROSInterruptException:
        pass
