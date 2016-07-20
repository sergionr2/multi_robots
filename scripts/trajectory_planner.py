#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import *
from multi_robots.msg import *
from std_msgs.msg import *
from visualization_msgs.msg import *

#paint Constants

OBSTACLES_NS = "Obstacles"
OBSTACLES_SCALE = Vector3( 0.02, 0.02, 0.02 )
OBSTACLES_COLOR = ColorRGBA( 1, 0, 0, 1 )
GEOMETRIES_NS = "Geometries"
GEOMETRIES_SCALE = Vector3(  0.01, 0.01, 0.01 )
GEOMETRIES_COLOR_ME = ColorRGBA( 15.0/255, 200.0/255, 1, 1 )
GEOMETRIES_COLOR_OTHERS = ColorRGBA( 25.0/255, 1, 0, 1 )
POSITION_NS = "Position"
POSITION_SCALE = Vector3(  0.02, 0.02, 0.1 )
POSITION_COLOR = ColorRGBA( 170.0/255, 85.0/255, 1, 1 )
VELOCITY_NS = "Velocity"
VELOCITY_LENGTH = 3 # n meters for each meter/second
VELOCITY_SCALE = Vector3(  0.02, 0.04, 0.04 ) # x diameter, y head diameter, z head length
VELOCITY_COLOR = ColorRGBA( 1, 170.0/255, 0, 1 )
TEXT_NS = "Text"
TEXT_SCALE = 0.1
TEXT_COLOR = ColorRGBA( 1, 1, 1, 1 )
TRAJECTORY_NS = "Trajectory"
TRAJECTORY_SCALE = Vector3( 0.01, 0.01, 0.01 )
TRAJECTORY_COLOR = ColorRGBA( 1, 65.0/255 , 90.0/255, 0.7 )
NUMBER_OF_LAST_POSES = 300

SAVE_TO_TRAJECTORY_DISTANCE = 0.01
CHANGE_GOAL_DISTANCE = 0.06
HZ = 5 #frecuence of publish
lastPoses = []
behavior = "OA"
robotInfo = [ 0, 0 ] #actual and last information
me = [ 0, 0 ]
#behaviors Variables
trajectoryCounter = 0

def distance( point1, point2 ):  #Computes the distance between two points
    dx = point1.x - point2.x
    dy = point1.y - point2.y
    return math.sqrt( dx**2 + dy**2 )

def translatePoint( point3D_toTranslate, pose2D ): # point
    x = point3D_toTranslate.x
    y = point3D_toTranslate.y
    z = point3D_toTranslate.z

    theta = point3D_toTranslate.x
    newPoint = Point( 0, 0, 0 )
    #rotate
    newPoint.x = x*math.cos( pose2D.theta  ) - y*math.sin( pose2D.theta )
    newPoint.y = x*math.sin( pose2D.theta  ) + y*math.cos( pose2D.theta )
    #translate
    newPoint.x += pose2D.x
    newPoint.y += pose2D.y
    return newPoint #point3D

def paintData( info, uid ):
    FRAME = 'Local_'+str(uid)
    pub = rospy.Publisher( 'local_'+str(uid), MarkerArray, queue_size=10 )

    markers = []
    cont = 0
    m = Marker()
    m.action = 3 # erase all
    markers.append( m )
    cont += 1

    m = Marker()
    m.header.frame_id = FRAME
    m.ns = OBSTACLES_NS
    m.id = cont
    m.type = m.SPHERE_LIST
    m.action = 0 # Add/Modify
    m.color = OBSTACLES_COLOR
    m.scale = OBSTACLES_SCALE
    m.points = info.obstacles
    cont += 1
    markers.append( m )

    m = Marker()
    m.header.frame_id = FRAME
    m.ns = TRAJECTORY_NS
    m.id = cont
    m.type = m.LINE_STRIP
    m.action = 0 # Add/Modify
    m.color = TRAJECTORY_COLOR
    m.scale = TRAJECTORY_SCALE
    m.points = [ Point( p.x, p.y, 0 ) for p in lastPoses ]
    cont += 1
    markers.append( m )

    for r in info.robots:
        m = Marker()
        m.header.frame_id = FRAME
        m.ns = GEOMETRIES_NS
        m.id = cont
        m.type = m.LINE_STRIP # LINE
        if( r.id == uid ):
            m.color = GEOMETRIES_COLOR_ME
        else:
            m.color = GEOMETRIES_COLOR_OTHERS
        m.scale = GEOMETRIES_SCALE
        points = [ translatePoint( p, r.pose ) for p in r.geometry ]
        m.points = points
        markers.append( m )
        cont += 1

        m = Marker()
        m.header.frame_id = FRAME
        m.ns = POSITION_NS
        m.id = cont
        m.type = m.SPHERE
        m.pose.position = Point( r.pose.x, r.pose.y, 0 )
        m.color = POSITION_COLOR
        m.scale = POSITION_SCALE
        markers.append( m )
        cont += 1

        m = Marker()
        m.header.frame_id = FRAME
        m.ns = VELOCITY_NS
        m.id = cont
        m.type = m.ARROW
        m.color = VELOCITY_COLOR
        m.scale = VELOCITY_SCALE
        pos = Point( r.pose.x, r.pose.y, 0 )
        vel = r.velocity
        m.points = [ pos, Point( pos.x + vel.x*VELOCITY_LENGTH, pos.y + vel.y*VELOCITY_LENGTH, pos.z + vel.z*VELOCITY_LENGTH)]
        markers.append( m )
        cont += 1

        m = Marker()
        m.header.frame_id = FRAME
        m.ns = TEXT_NS
        m.id = cont
        m.type = m.TEXT_VIEW_FACING
        m.action = 0 # Add/Modify
        m.pose.position.x = pos.x
        m.pose.position.y = pos.y
        m.pose.position.z = TEXT_SCALE
        m.scale.z = TEXT_SCALE
        m.color = TEXT_COLOR
        if( r.id == uid ):
            m.text = "Behavior: " + behavior
        else:
            m.text = "ID: " + str( r.id )
        cont += 1
        markers.append( m )

    pub.publish( markers )

def getInfo( info, uid ):
    paintData( info, uid )
    canISeeMe = False
    global me, robotInfo
    #get relevant information, me, info, and lastPose
    for r in info.robots:
        if( r.id == uid ):
            canISeeMe = True
            me[0] = r
            info.robots.remove(r)
            robotInfo[0] = info
            #insert last pose
            if len( lastPoses ) > 0: #id is the first time
                if distance( me[0].pose, lastPoses[0] ) > SAVE_TO_TRAJECTORY_DISTANCE:
                    lastPoses.insert( 0, r.pose )
            else:
                lastPoses.insert( 0, r.pose )
            #keep the size of lastPoses bounded
            if len( lastPoses ) > NUMBER_OF_LAST_POSES:
                lastPoses.pop()

    if( canISeeMe ): # if i can see me
        pubPose = rospy.Publisher('pose_'+str(uid), Pose2D, queue_size=10)
        pubPose.publish( me[0].pose )
    else:
        me[0] = 0
        robotInfo[0] = info
        pass #do something if i am blind

# BEHAVIORS
def obstacle_avoidance():
    d = 0.20
    if( me[0] != 0 ):
        for point in robotInfo[0].obstacles:
            if distance( me[0].pose, point ) < d:
                d = distance( me[0].pose, point )
                alfa = math.atan2( point.y - me[0].pose.y, point.x - me[0].pose.x )
        ratio = 0.50
        if d == 0.20:
            return 0
        else:
            return Pose2D( -1*ratio*math.cos(alfa) + me[0].pose.x, -1*ratio*math.sin(alfa) + me[0].pose.y, 0 )
    return 0

def zig_zag_trajectory():
    arrayOfPoints = [
        Pose2D( 0.3, 2, 0),
        Pose2D( 0.3, 0.8, 0),
        Pose2D( 1.4, 2, 0),
        Pose2D( 1.4, 0.8, 0),
    ]
    if isinstance( me[0], Robot ):
        if distance( me[0].pose, arrayOfPoints[ trajectoryCounter % len( arrayOfPoints ) ] ) < CHANGE_GOAL_DISTANCE:
            global trajectoryCounter
            trajectoryCounter += 1
    return arrayOfPoints[ trajectoryCounter % len( arrayOfPoints ) ]

#END OF BEHAVIORS
def setBehavior( behav ):
    global behavior
    behavior = behav

def setGoal( behavior ):

    states = {
        "ZZT": zig_zag_trajectory,
        "OA": obstacle_avoidance,
    }
    function = states.get( behavior, obstacle_avoidance )
    return function()

def planner():
    #init params
    rospy.init_node('Planner', anonymous=False)
    uid = rospy.get_param( '~id', 1 )
    pubGoal = rospy.Publisher('goal_'+str(uid), Pose2D, queue_size=10)
    pubGoalPoint = rospy.Publisher('point_'+str(uid), PointStamped, queue_size=10)
    setBehavior( rospy.get_param('~behavior', "OA") ) #Obstacle avoidance
    rate = rospy.Rate( HZ )
    rospy.Subscriber( "info_"+str(uid) , GPSinfo, getInfo, uid )
    global me, robotInfo
    while not rospy.is_shutdown():
        newGoal = setGoal( str( behavior ) )
        if newGoal != 0:
            goalPoint = PointStamped()
            goalPoint.header.stamp = rospy.Time().now()
            goalPoint.header.frame_id = 'Local_'+str(uid)
            goalPoint.point = Point( newGoal.x, newGoal.y, 0 )
            pubGoalPoint.publish( goalPoint )
            pubGoal.publish( newGoal )
        me[1] = me[0]
        robotInfo[1] = robotInfo[0]
        rate.sleep()

if __name__ == '__main__':
    try:
        planner()
    except rospy.ROSInterruptException:
        pass
