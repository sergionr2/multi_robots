#!/usr/bin/env python
#   this node do the control and simulation of a simple unicycle robot model dinamics
#   Vx = V*Cos( Theta )
#   Vy = V*Sin( Theta )
#   W = d(theta)/dt
#   with w_r and w_l as control actions, the angular velocity of right and left wheels
#   V = (w_r + w_l)*R/2
#   W = (w_r - w_l)*R/L
#
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from multi_robots.msg import RobotPose
#set Start position if lost
LOST_TIME = 2 #check if lost every 2 seconds

R = 0.030 # m  wheel radius
L = 0.100 # m Distance between wheels

# MAX angular speed
W_MAX = 6 #rads/s
W_MIN = 2
#MAX linear speed
V_MAX = 4 # rads/s, *R to m/s
V_MIN = 2

KP = 5 #for the angularSpeed
KI = 0
D = 0.400 # if error larger than D use V_MAX
KP_V = float(V_MAX)/D #for the LinearSpeed
KI_V = 0
acumDistance = 0
acumAngle = 0

MAX_ANGLE_ERROR = 5 #Degrees
MAX_POS_ERROR = 0.020 #m

#initial goal
x_goal = 1
y_goal = 1
#time of last position
last_time = 0

ID = 0

def setPose( pose ):

    x = pose.x
    y = pose.y
    ang = pose.theta

    d_x = x_goal - x #distance to goal in X
    d_y = y_goal - y #distance to goal in Y

    distance_error = math.sqrt( d_x**2 + d_y**2 ) #distance to Goal

    print "Distance To Goal: " + str(distance_error) + 'm\n'
    angle_error = 0
    if distance_error > MAX_POS_ERROR :
        # The projection is NOT d_x*math.cos( ang ) + d_y*math.sin( ang )
        # beacause the robot frame is rotated pi/2 in z compared to the model used
        dotProduct = d_x*-1*math.sin( ang ) + d_y*math.cos( ang ) #proyection of the distance and angle vectors
        angle_error = math.acos( dotProduct/distance_error ) # Angle between the vector from 0 to pi
        # To know the turning direction (if the error is negative)
        dotProduct2 = d_x*-1*math.sin( ang+0.01 ) + d_y*math.cos( ang+0.01 ) # turning 0.01 in positive direction
        nextError = math.acos( dotProduct2/distance_error )
        if nextError > angle_error: # if error is greater
            angle_error *= -1 #the sign is changed
    global acumAngle, acumDistance
    acumDistance += distance_error # acumulated errors
    acumAngle += angle_error

    w_r = angle_error*KP + acumAngle*KI
    #set max and min limits of control action
    if( w_r > W_MAX ):
        w_r = W_MAX
    elif( w_r < -1*W_MAX ):
        w_r = -1*W_MAX
    elif( abs( w_r ) < W_MIN ):
        if( w_r > 0 ):
            w_r = W_MIN
        else:
            w_r = -1*W_MIN

    #if angle achieved
    if( abs(angle_error*180/math.pi) < MAX_ANGLE_ERROR):
        w_r = 0
    w_l = -1*w_r #Put the same but contrary in the left wheel to make turn

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
    w_r += vel # add the linear composant of the speed
    w_l += vel
    #if position achieved
    if( abs( distance_error ) < MAX_POS_ERROR):
        w_r = 0
    #END OF CONTROL
    #START OF MODEL DINAMICS

    w = (w_r - w_l)*R/L
    v = (w_r + w_l)*R/2

    dt = (rospy.Time().now() - last_time ).to_sec()
    global last_time
    last_time = rospy.Time().now()

    ang += w*dt
    #verify that the angle is between -pi and pi
    if( ang > math.pi ):
        ang -= 2*math.pi
    elif( ang < -1*math.pi ):
        ang += 2*math.pi

    # remember the robot frame is rotated pi/2 in z compared to the model used
    x += -1*v*math.sin( ang )*dt
    y += v*math.cos( ang )*dt

    ID = rospy.get_param('~id', 1 )
    pub = rospy.Publisher('robot_pose', RobotPose, queue_size=10)
    pub.publish( RobotPose( ID, Pose2D( x, y, ang ), rospy.Time().now() ) )

def setGoal( goal ):
    global x_goal, y_goal
    x_goal = goal.x
    y_goal = goal.y
    # Do nothing with goal.theta

def unicycle():

    #init params
    rospy.init_node('Sistem' , anonymous=False)
    global ID
    ID = rospy.get_param('~id', 1 )
    x_0 = rospy.get_param('~x', 0 )
    y_0 = rospy.get_param('~y', 0 )
    theta_0 = rospy.get_param('~theta', 0 )
    global last_time
    last_time = rospy.Time().now()
    init = RobotPose( ID, Pose2D( x_0, y_0, theta_0 ), rospy.Time().now() )
    pub = rospy.Publisher( 'robot_pose', RobotPose, queue_size=10, latch=True)
    pub.publish( init )
    rospy.Subscriber( "goal_"+str(ID) , Pose2D, setGoal )
    rospy.Subscriber( "pose_"+str(ID) , Pose2D, setPose )
    rate = rospy.Rate( 1.0/LOST_TIME )
    while not rospy.is_shutdown():
        if  rospy.Time().now() - last_time > rospy.Duration( LOST_TIME ) :
            last_time = rospy.Time().now()
            pub.publish( RobotPose( ID, Pose2D( x_0, y_0, theta_0 ), rospy.Time().now() ) )
        rate.sleep()

if __name__ == '__main__':
    unicycle()
