#!/usr/bin/env python
## This ROS NODE suscribes to Alvar_ar_track, recives the position and orientation, and corriges it
## IT publish a RobotPose msg --> id,  Pose2D, and time (of last position recived)
##
import rospy
import math
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import *
from multi_robots.msg import *

#desired origins
ORIGIN_X = -1.3
ORIGIN_Y = -0.9

def correctTheta( q ):
    # Compute the angle theta from quaternion
    theta = -1*math.atan2( 2*(q.w*q.z+q.y*q.x) , 1-2*(q.y**2+q.z**2) )
    ## This is the rotation responsable of asigning x to Y in function reciveAlvarMarkers
    theta += math.pi/2
    if( theta > math.pi ): # verifies that angle is between -pi and pi
        theta -= 2*math.pi
    return theta

def correctX( x ):
    A1 = 0.065087 # Constants obtained from measurements
    A2 = 0.000243
    A3 = 0.019206
    error = A3*(x**3)+A2*(x**2)+A1*(x) # get the error
    return x - error # corriges de error

def correctY( y ):
    A1 = 0.043623 # Constants obtained from measurements
    A2 = 0.001636
    error = A2*(y**2)+A1*(y)# get the error
    return y - error # corriges de error

def reciveAlvarMarkers( alvar ):
    pub = rospy.Publisher('robot_pose', RobotPose, queue_size=10)
    for i in alvar.markers:
        p = i.pose.pose.position # Get position from message AlvarMarkers
        q = i.pose.pose.orientation # Get orientation from message AlvarMarkers
        #X asigned to y to rotate the coordinate system to a desired one
        x = correctY( p.y ) - ORIGIN_Y # Correct position and move to the origin
        y = correctX( p.x ) - ORIGIN_X
        theta =  correctTheta( q ) #Correct orientation

        pose = Pose2D( x, y, theta )

        pub.publish( RobotPose( i.id, pose, rospy.Time.now() ) ) # publish the message

def corrector():

    rospy.init_node('Corrector', anonymous=False) # begin the node
    rospy.Subscriber( 'ar_pose_marker', AlvarMarkers, reciveAlvarMarkers )
    rospy.spin()

if __name__ == '__main__':
    try:
        corrector()
    except rospy.ROSInterruptException:
        pass
