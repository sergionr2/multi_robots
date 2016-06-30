#!/usr/bin/env python

## GPS
##

import rospy
import math
import random
from geometry_msgs.msg import *
from multi_robots.msg import *
from std_msgs.msg import *
from visualization_msgs.msg import *

MAXTIME = 0.5 #seconds, time to unable if not visible
N = 20 #number of points to calcul velocity
HZ = 30 #frecuence of publish 30hz
URL_OBJS = 0
URL_MAP = 0

INIT_X = -1
INIT_Y = -1
INIT_THETA = math.pi /4
FRAME = 'World'
DEFAULT_GEOM = '8'

robotGeom = {} # int : Point[]
robotLastPoses = {} # int : RobotPose[N]
idList = [] # IDs
topicList = {} # int : pub

#TODO parameter
A = 0 # Noise Amplitud m

def setEnableds(): #disabled if pose is older that MAXTIME

    enabled = {}
    for i in idList:
        time = rospy.Time().now()
        if len(robotLastPoses[i]) > 0 :
            posTime = robotLastPoses[i][0].time
            if ( time - posTime < rospy.Duration( MAXTIME )  ):
                enabled[i] = True
            else:
                enabled[i] = False
        else:
            enabled[i] = False

    return enabled

def getRobotLastPose( lastPoses ):
    if len(lastPoses) > 0:
        poseNew  = lastPoses[0].pose
        dt = 1
        if len(lastPoses) == 1:
            poseOld  = lastPoses[0].pose
        else:
            poseOld  = lastPoses[N-1].pose
            dt = ( lastPoses[0].time.to_sec() - lastPoses[N-1].time.to_sec() )#.to_sec()
            if dt == 0:
                dt = 1
        x = poseNew.x
        y = poseNew.y
        dx = x - poseOld.x
        dy = y - poseOld.y
        #add Noise if not inmobile object
        if len(lastPoses) != 1:
            x += A*random.random()
            y += A*random.random()
        return Pose2D( x, y, poseNew.theta ), Vector3( dx/dt, dy/dt, 0 )
    else:
        return Pose2D( INIT_X, INIT_Y, INIT_THETA ), Vector3( 0, 0, 0 )

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

def distance( p1, p2 ):
    dx = p1.x - p2.x
    dy = p1.y - p2.y
    dz = p1.z - p2.z
    return math.sqrt( dx**2 + dy**2 + dz**2 )

def searchIn( pointList, ratio, pose2D ):
    points = []
    x = pose2D.x
    y = pose2D.y
    if ratio > 0:
        for p in pointList:
            if distance( p, Point( x, y, 0 ) ) < ratio:
                points.append( p )
    return points

def printWorld( robots, obstacles, enableds ):
    pub = rospy.Publisher( 'world', MarkerArray, queue_size=10 )
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
    m.points = obstacles
    cont += 1
    markers.append( m )

    for r in robots:
        m = Marker()
        m.header.frame_id = FRAME
        m.ns = "Geom"
        m.id = cont
        m.type = m.LINE_STRIP # LINE
        if( enableds[r.id] ):
            m.color = ColorRGBA( 25.0/255 ,1, 0, 1 )
        else:
            m.color = ColorRGBA( 255.0/255 ,170.0/255, 0, 1 )
        m.scale = Vector3( 0.01, 0.01, 0.01 )
        points = [ translatePoint( p, r.pose ) for p in r.geometry ]
        m.points = points
        markers.append( m )
        cont += 1

        m = Marker()
        m.header.frame_id = FRAME
        m.ns = "Pos"
        m.id = cont
        m.type = m.SPHERE
        m.pose.position = Point( r.pose.x, r.pose.y, 0 )
        m.color = ColorRGBA( 170.0/255 ,85.0/255, 255/255, 1 )
        m.scale = Vector3( 0.02, 0.02, 0.02 )
        markers.append( m )
        cont += 1

        m = Marker()
        m.header.frame_id = FRAME
        m.ns = "vel"
        m.id = cont
        m.type = m.ARROW
        m.color = ColorRGBA( 1 ,170.0/255, 0, 1 )
        s2 = 0.2
        m.scale = Vector3( 0.02, 0.04, 0.04 )
        pos = Point( r.pose.x, r.pose.y, 0 )
        vel = r.velocity
        m.points = [ pos, Point( pos.x + vel.x*s2,  pos.y + vel.y*s2, pos.z + vel.z*s2 )]
        markers.append( m )
        cont += 1

        m = Marker()
        m.header.frame_id = FRAME
        m.ns = "text"
        m.id = cont
        m.type = m.TEXT_VIEW_FACING
        m.action = 0 # Add/Modify
        m.pose.position.x = pos.x
        m.pose.position.y = pos.y
        s = 0.1
        m.pose.position.z = s
        m.scale.z = s
        m.color.a = 1.0
        m.color.r = 255.0/255
        m.color.g = 255.0/255
        m.color.b = 255.0/255
        m.text = "ID: " + str( r.id ) + "\nVel: " + str( round(math.sqrt(vel.x**2+vel.y**2),3) )
        cont += 1
        markers.append( m )
    pub.publish( markers )

def publish( ids, key, mapMatrix ):
    robotActualGeom = {}
    robots = {}
    staticPoints = []

    #CheckForRobots
    enabled = setEnableds() #disabled if pose is older that MAXTIME
    #CreateRobots
    for i in ids: #iterate every id
        #getActualPose and velocity
        actualPose, actualVelocity = getRobotLastPose( robotLastPoses[i] )
        robotActualGeom[i] = [ translatePoint( p, actualPose ) for p in robotGeom[i] ]
        if( i >= 0 ):
            robots[i] =  Robot( i, actualPose, actualVelocity, robotGeom[i] )
        else:
            staticPoints.extend( robotActualGeom[i] )

    #print staticPoints
    printWorld( robots.values(), staticPoints, enabled )

    for i in ids: #iterate every robot
        if i >= 0 and enabled[i]:
            visibleRobots = []
            mapPoints = [] #Posible Obstacles, Every not visible point
            mapPoints.extend( staticPoints )
            for k in ids:
                if enabled[k]:
                    if i in key and k in key:
                        if mapMatrix[ key[i] ][ key[k]+1 ] >= 0: # if i can see K
                            #fill visible robots
                            visibleRobots.append( robots[k] )
                        else:
                            #Fill Posible Points
                            mapPoints.extend( robotActualGeom[k] )
                    else:
                        #Fill Posible Points
                        mapPoints.extend( robotActualGeom[k] )
            obstacles = []
            for k in ids:
                if k >= 0 and enabled[k]:
                    if i in key and k in key:
                        #Search for Obtacles
                        ratio = float( mapMatrix[ key[i] ][ key[k]+1 ] )/1000
                        obstacles.extend( searchIn( mapPoints, ratio, robots[k].pose ) )
            if not( i in key ):
                visibleRobots.append( robots[i] ) # if not in map
            #publish
            topicList[i].publish( GPSinfo( obstacles, visibleRobots ) )

def initTopics( ids ):
    pubs = {}
    for i in ids:
        if i >= 0:
            pubs[i] = rospy.Publisher( 'info_' + str( i ), GPSinfo, queue_size=10 )

    return pubs

def setRobotPose( robotPose ):
    if robotPose.id != 0: # filter ID 0 because of Noise
        if idList.count( robotPose.id ) == 0 : #Add the new robot
            topicList[ robotPose.id ] = rospy.Publisher( 'info_' + str( robotPose.id ), GPSinfo, queue_size=10 )
            if( not robotGeom.has_key( robotPose.id ) ):
                robotGeom[ robotPose.id ] = getPolygon( DEFAULT_GEOM )
            robotLastPoses[ robotPose.id ] = []
            idList.append( robotPose.id )
            print "New Robot Added ID: ", robotPose.id
        while len( robotLastPoses[ robotPose.id ] ) <= N:
            robotLastPoses[ robotPose.id ].insert(0, robotPose )
        robotLastPoses[ robotPose.id ].pop()

def readMapMatrix():
    f = open ( URL_MAP, 'r' )
    matrix = [ map(int, line.split(',') ) for line in f ] #read file
    idToRow = dict( ( matrix[row][0] , row ) for row in range( 0, len(matrix) ) )
    return idToRow, matrix

def getPolygon( url ):
    points = 0
    try:
        f = open ( url, 'r' )
        points = [ map( int, line.split(',') ) for line in f ] #read file
    except Exception as e:
        print "Invalid or inexistent file "+ url +", using default geometry: (0,0)\n"
        points = [ [ 0,0 ] ]
    return [ ( Point( float(point[0])/1000, float(point[1])/1000, 0 ) ) for point in points ] #convert in m


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

def shutDown():
    print("GPS end")

def gps():
    rospy.init_node('Gps', anonymous=False)

    global URL_OBJS, URL_MAP, DEFAULT_GEOM
    #init params
    URL_OBJS = rospy.get_param('objects', '/home/multi-robots/catkin_ws/src/multi_robots/GPSconfig/objects.txt' )
    URL_MAP = rospy.get_param('map', '/home/multi-robots/catkin_ws/src/multi_robots/GPSconfig/map.txt' )
    DEFAULT_GEOM = rospy.get_param('default_geom', '/home/multi-robots/catkin_ws/src/multi_robots/robotGeometry/default.txt' )

    try: # read Objects file
        global idList, robotGeom, robotLastPoses
        idList, robotGeom, robotLastPoses = readRobotGeom()
    except Exception as e:
        print "Invalid or inexistent file 'objects', GPS cannot start \n Verify file URL, check README"
        #raise rospy.ROSInterruptException
    try: # read Map file
        keys, mapMatrix = readMapMatrix()
    except Exception as e:
        print "Invalid or inexistent file 'map', GPS cannot start \n Verify file URL, check README"
        #raise rospy.ROSInterruptException

    rospy.on_shutdown( shutDown )
    global topicList
    topicList = initTopics( idList )
    rospy.Subscriber( "robot_pose" , RobotPose, setRobotPose )
    rate = rospy.Rate( HZ )
    while not rospy.is_shutdown():

        publish( list(idList), keys, mapMatrix )
        rate.sleep()


if __name__ == '__main__':
    try:
        gps()
    except rospy.ROSInterruptException:
        pass
