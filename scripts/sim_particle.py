#!/usr/bin/env python
#   this node do the control and simulation of a particle model dinamics
#   Vx = V*Cos( Theta )
#   Vy = V*Sin( Theta )
#
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from multi_robots.msg import RobotPose


#MAX linear speed
V_MAX = 0.200 #  m/s
V_MIN = 0
# Controllers gains
D = 0.400 # if error larger than D use V_MAX
KP_V = float(V_MAX)/D #for the LinearSpeed
KI_V = 0

acumDistance = 0

MAX_POS_ERROR = 0.001 #m

x_goal = 1
y_goal = 1

def setPose(pose):

    x = pose.x
    y = pose.y

    d_x = x_goal - x #distance to goal in X
    d_y = y_goal - y #distance to goal in Y

    distance_error = math.sqrt( d_x**2 + d_y**2 ) #distance to Goal
    print "Distance To Goal: " + str(distance_error) + 'm\n'
    global acumDistance
    acumDistance += distance_error # acumulated errors

    vel = KP_V*distance_error + KI_V*acumDistance
    #set max and min limits of control action
    if( vel > V_MAX ):
        vel = V_MAX
    elif( vel < -1*V_MAX ):
        vel = -1*V_MAX
    elif( abs( vel ) < V_MIN ):
        if( vel > 0 ):
            vel = V_MIN
        else:
            vel = -1*V_MIN
    #if position achieved
    if( abs( distance_error ) < MAX_POS_ERROR):
        vel = 0
    #END OF CONTROL
    #START OF MODEL DINAMICS
    dt = (rospy.Time().now() - last_time ).to_sec()
    global last_time
    last_time = rospy.Time().now()

    ang = math.atan2( d_y, d_x );
    x += vel*math.cos( ang )*dt
    y += vel*math.sin( ang )*dt

    pub = rospy.Publisher('robot_pose', RobotPose, queue_size=10)
    pub.publish( RobotPose( ID, Pose2D( x, y, 0 ), rospy.Time().now() ) )

def setGoal(goal):
    global x_goal, y_goal
    x_goal = goal.x
    y_goal = goal.y

def particle():

    #init params
    rospy.init_node('System' , anonymous=False)
    ID = rospy.get_param('~id', 1 )
    x_0 = rospy.get_param('~x', 0 )
    y_0 = rospy.get_param('~y', 0 )

    last_time = rospy.Time().now()
    init = RobotPose( ID, Pose2D( x_0, y_0, 0 ), rospy.Time().now() )
    pub = rospy.Publisher( 'robot_pose', RobotPose, queue_size=10, latch=True)
    pub.publish( init )
    rospy.Subscriber( "goal_"+str(ID) , Pose2D, setGoal )
    rospy.Subscriber( "pose_"+str(ID) , Pose2D, setPose )
    rospy.spin()

if __name__ == '__main__':
    particle()
