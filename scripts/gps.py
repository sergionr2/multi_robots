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
        #add Noise
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

def printWorld( geometries, obstacles, positions, velocities  ):
             # [ Point[] ], Point[], Pose2D[], Vector3[]
    #print Robot TODO
    pub = rospy.Publisher( 'world', MarkerArray, queue_size=10 )
    markers = []
    cont = 0
    for i in geometries:
        m = Marker()
        m.header.frame_id = FRAME
        m.ns = "geometries"
        m.id = cont
        m.type = m.LINE_STRIP # LINE
        m.action = 0 # Add/Modify
        m.scale.x = 0.01
        m.scale.y = 0.01
        m.scale.z = 0.01
        m.color.a = 1.0
        m.color.r = 25.0/255
        m.color.g = 255.0/255
        m.color.b = 0.0/255
        m.points = i
        markers.append( m )
        cont += 1
    m = Marker()
    m.header.frame_id = FRAME
    m.ns = "obstacles"
    m.id = cont
    m.type = m.SPHERE_LIST
    m.action = 0 # Add/Modify
    m.scale.x = 0.04
    m.scale.y = 0.04
    m.scale.z = 0.04
    m.color.a = 1.0
    m.color.r = 255.0/255
    m.color.g = 0/255
    m.color.b = 0/255
    m.points = obstacles
    cont += 1
    markers.append( m )

    robotpoints = [( Point( p.x, p.y, 0 )) for p in positions ]

    m = Marker()
    m.header.frame_id = FRAME
    m.ns = "positions"
    m.id = cont
    m.type = m.SPHERE_LIST
    m.action = 0 # Add/Modify
    s = 0.02
    m.scale.x = s
    m.scale.y = s
    m.scale.z = s
    m.color.a = 1.0
    m.color.r = 170.0/255
    m.color.g = 85.0/255
    m.color.b = 255.0/255
    m.points = robotpoints
    cont += 1
    markers.append( m )

    for i in range( 0,len(velocities) ):

        m = Marker()
        m.header.frame_id = FRAME
        m.ns = "velocities"
        m.id = cont
        m.type = m.ARROW
        m.action = 0 # Add/Modify
        s = 0.01
        m.scale.x = s
        m.scale.y = s*2
        m.scale.z = s*2
        m.color.a = 1.0
        m.color.r = 255.0/255
        m.color.g = 255.0/255
        m.color.b = 255.0/255
        s2 = 1 #Scale
        pos = robotpoints[i]
        vel = velocities[i]
        finalVel = Point( pos.x + vel.x*s2,  pos.y + vel.y*s2, pos.z + vel.z*s2 )
        m.points = [pos, finalVel]
        cont += 1
        markers.append( m )

    #for i in markers:
    pub.publish( markers )

def publish( pub, key, mapMatrix ):
    robotActualGeom = {}
    robots = {}
    staticPoints = []
    geometries = [] #TO PAINT
    positions = [] #TO PAINT
    velocities = [] #TO PAINT
    #CheckForRobots
    enabled = setEnableds() #disabled if pose is older that MAXTIME
    #CreateRobots
    for i in idList: #iterate every id
        #getActualPose and velocity
        actualPose, actualVelocity = getRobotLastPose( robotLastPoses[i] )
        robotActualGeom[i] = [ translatePoint( p, actualPose ) for p in robotGeom[i] ]
        if( i >= 0 ):
            robots[i] =  Robot( i, actualPose, actualVelocity, robotGeom[i] )
            geometries.append( robotActualGeom[i] )
            positions.append( actualPose )
            velocities.append( actualVelocity )
        else:
            staticPoints.extend( robotActualGeom[i] )

    printWorld( geometries, staticPoints, positions, velocities )

    #print staticPoints TODO

    for i in idList: #iterate every robot
        if i >= 0 and enabled[i]:
            visibleRobots = []
            mapPoints = [] #Posible Obstacles, Every not visible point
            mapPoints.extend( staticPoints )
            for k in idList:
                if k >= 0:
                    if mapMatrix[ key[i] ][ key[k]+1 ] < 0: # if i cannot see K
                        #Fill Posible Points
                        mapPoints.extend( robotActualGeom[k] )
                    else:
                        #fill visible robots
                        visibleRobots.append( robots[i] )
            obstacles = []
            for k in idList:
                if k >= 0:
                    #Search for Obtacles
                    ratio = float( mapMatrix[ key[i] ][ key[k]+1 ] )/1000
                    obstacles.extend( searchIn( mapPoints, ratio, robots[i].pose ) )
            #publish
            pub[i].publish( GPSinfo( obstacles, visibleRobots ) )

def initTopics( ids ):
    pubs = {}
    for i in ids:
        if i >= 0:
            pubs[i] = rospy.Publisher( 'info_' + str( i ), GPSinfo, queue_size=10 )

    return pubs

def setRobotPose( data ):
        if idList.count( data.id ) == 0:
            print "[WARN]Unknow ID: "+str(data.id)+", ignoring item, check configuration"
        else:
            while len( robotLastPoses[ data.id ] ) <= N:
                robotLastPoses[ data.id ].insert(0, data)
            robotLastPoses[ data.id ].pop()

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
        print "Invalid or inexistent file "+ url +", using default geometry: "+ GEOM +'\n'
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
    try: # read Objects file
        global idList, robotGeom, robotLastPoses
        idList, robotGeom, robotLastPoses = readRobotGeom()
    except Exception as e:
        print "Invalid or inexistent file 'objects', GPS cannot start \n Verify file URL, check README"
        raise rospy.ROSInterruptException
    try: # read Map file
        keys, mapMatrix = readMapMatrix()
    except Exception as e:
        print "Invalid or inexistent file 'map', GPS cannot start \n Verify file URL, check README"
        raise rospy.ROSInterruptException

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
