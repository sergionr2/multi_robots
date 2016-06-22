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

ID = 6

R = 30 # mm  wheel radius
L = 100 # Distance between wheels

# MAX angular speed
W_MAX = 5 #rads/s
W_MIN = 3
#MAX linear speed
V_MAX = 5 # rads/s, *R to mm/s
V_MIN = 2.2

KP = 5 #for the angularSpeed
KI = 0
D = 400 # if error larger than D use V_MAX
KP_V = float(V_MAX)/D #for the LinearSpeed
KI_V = 0
acumDistance = 0
acumAngle = 0

MAX_ANGLE_ERROR = 5 #Degrees
MAX_POS_ERROR = 20 #mm

#pose initial
x_0 = 1 # m
y_0 = 1 # m
theta_0 = 0
last_time = 0 # time of the last publication

x_goal = 1000
y_goal = 1000

def setPose(data):

    x = data.x * 1000 #pass to mm
    y = data.y * 1000
    ang = data.theta

    d_x = x_goal - x #distance to goal in X
    d_y = y_goal - y #distance to goal in Y

    distance_error = math.sqrt( d_x**2 + d_y**2 ) #distance to Goal
    #TODO comenter pour quoi -sin et cos et pas cos et sin
    print "Distance To Goal: " + str(distance_error) + '\n'
    angle_error = 0
    if distance_error > MAX_POS_ERROR :
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

    #TODO meme commentaire
    x += -1*v*math.sin( ang )*dt
    y += v*math.cos( ang )*dt

    pub = rospy.Publisher('robot_pose', RobotPose, queue_size=10)
    pub.publish( RobotPose( ID, Pose2D( x/1000, y/1000, ang ), rospy.Time().now() ) )

def setGoal(data):
    global x_goal, y_goal
    x_goal = data.x * 1000 # the controller works in mm
    y_goal = data.y * 1000

def simulator():

    global ID, last_time, x_0, y_0, theta_0
    rospy.init_node('sim_'+ str(ID) , anonymous=False)
    rospy.Subscriber( "goal_"+str(ID) , Pose2D, setGoal )
    rospy.Subscriber( "pose_"+str(ID) , Pose2D, setPose )
    last_time = rospy.Time().now()
    init = RobotPose( ID, Pose2D( x_0, y_0, theta_0 ), rospy.Time().now() )
    pub = rospy.Publisher( 'robot_pose', RobotPose, queue_size=10, latch=True)

    pub.publish( init )
    print init
    rospy.spin()

if __name__ == '__main__':
    simulator()
