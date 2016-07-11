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
NUMBER_OF_LAST_POSES = 1000 #TODO add distance constaint

HZ = 5 #frecuence of publish
lastPoses = []
behavior = "OA"

#behaviors Variables
trajectoryCounter = 0;

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
            m.text = "Behavior: " + behavior + "\nVel: " + str( round(math.sqrt(vel.x**2+vel.y**2),3) )
        else:
            m.text = "ID: " + str( r.id ) + "\nVel: " + str( round(math.sqrt(vel.x**2+vel.y**2),3) )
        cont += 1
        markers.append( m )

    pub.publish( markers )


def getInfo( info, uid ):

    paintData( info, uid )
    canISeeMe = False
    me = 0
    for r in info.robots:
        if( r.id == uid ):
            canISeeMe = True
            me = r
            info.robots.remove(r)
            lastPoses.insert( 0, r.pose )
            if len( lastPoses ) > NUMBER_OF_LAST_POSES:
                lastPoses.pop()
    if( canISeeMe ): # if i can see me
        pubPose = rospy.Publisher('pose_'+str(uid), Pose2D, queue_size=10)
        pubPose.publish( me.pose )
    else:
        pass #do something if i am blind

# BEHAVIORS
def obstacle_avoidance():
    return Pose2D( 1,0,0 )

def zig_zag_trajectory():
#ADD w8 to goal TODO
    global trajectoryCounter
    trajectoryCounter += 1
    arrayOfPoints = [
        Pose2D( 0.25, 1.25, 0),
        Pose2D( 0.25, 0.25, 0),
        Pose2D( 1.25, 1.25, 0),
        Pose2D( 1.25, 0.25, 0),
    ]
    return arrayOfPoints[ trajectoryCounter % len( arrayOfPoints ) ]
#END OF BEHAVIORS

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
    global behavior
    behavior = rospy.get_param('~behavior', "OA") #Obstacle avoidance
    rate = rospy.Rate( HZ )
    rospy.Subscriber( "info_"+str(uid) , GPSinfo, getInfo, uid )
    while not rospy.is_shutdown():

        pubGoal.publish( setGoal( str( behavior ) ) )
        rate.sleep()

if __name__ == '__main__':
    try:
        planner()
    except rospy.ROSInterruptException:
        pass
