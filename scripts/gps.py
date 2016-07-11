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

#color Constants



MAXTIME = 0.5 #seconds, time to unable if not visible
N = 30 #number of points to compute velocity
HZ = 30 #publish frecuence 30hz

INIT_X = -1
INIT_Y = -1
INIT_THETA = math.pi /4
FRAME = 'World'

robotGeom = {} # int : Point[]
robotLastPoses = {} # int : RobotPose[N]
idList = [] # IDs
topicList = {} # int : pub

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

def getRobotLastPose( lastPoses ): #computes Pose and velocity
    if len(lastPoses) > 0:
        poseNew  = lastPoses[0].pose
        dt = 1
        if len(lastPoses) == 1: # inmobile object
            poseOld  = lastPoses[0].pose
        else:
            poseOld  = lastPoses[N-1].pose
            dt = ( lastPoses[0].time.to_sec() - lastPoses[N-1].time.to_sec() )
            if dt == 0: #to prevent division by 0
                dt = 1
        x = poseNew.x
        y = poseNew.y
        dx = x - poseOld.x
        dy = y - poseOld.y
        A = rospy.get_param('~noise_amplitud', 0 ) #noise amplitud
        #add Noise if not inmobile object
        if len(lastPoses) != 1:
            x += A*random.random()
            y += A*random.random()

        return Pose2D( x, y, poseNew.theta ), Vector3( dx/dt, dy/dt, 0 )
    else:
        return Pose2D( INIT_X, INIT_Y, INIT_THETA ), Vector3( 0, 0, 0 ) #return initial values

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

def distance( point1, point2 ):  #Computes the distance between two points
    dx = point1.x - point2.x
    dy = point1.y - point2.y
    dz = point1.z - point2.z
    return math.sqrt( dx**2 + dy**2 + dz**2 )

def searchIn( pointList, ratio, pose2D ):
    #search in a Point list, the points near to pose2D in a ratio
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
            mapPoints = [] #Posible Obstacles (Every not visible point)
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
                visibleRobots.append( robots[i] ) # if not in map publish pose
            #publish
            topicList[i].publish( GPSinfo( obstacles, visibleRobots ) )

def initTopics( ids ):
    pubs = {}
    for i in ids:
        if i >= 0: # negative ids are inmobile objects
            pubs[i] = rospy.Publisher( 'info_' + str( i ), GPSinfo, queue_size=10 )
    return pubs

def setRobotPose( robotPose ): #Getting new poses

    if robotPose.id != 0: # filter ID 0 because of Noise
        if idList.count( robotPose.id ) == 0 : #Add the new robot to topic list
            topicList[ robotPose.id ] = rospy.Publisher( 'info_' + str( robotPose.id ), GPSinfo, queue_size=10 )
            #add defaul geometry if needed
            if( not robotGeom.has_key( robotPose.id ) ):
                default_geometry_url = rospy.get_param('~default_geom')
                robotGeom[ robotPose.id ] = getPolygon( default_geometry_url )
            #Create new lastPoses list
            robotLastPoses[ robotPose.id ] = []
            #Finally add to the id list
            idList.append( robotPose.id )
            print "New Robot Added ID: ", robotPose.id
        #add to the lastPoses the new position
        while len( robotLastPoses[ robotPose.id ] ) <= N:
            robotLastPoses[ robotPose.id ].insert(0, robotPose )
        robotLastPoses[ robotPose.id ].pop()

def readMapMatrix(): #reading visibility map
    map_url = rospy.get_param('~map')
    f = open ( map_url, 'r' )
    matrix = [ map(int, line.split(',') ) for line in f ] #read file
    idToRow = dict( ( matrix[row][0] , row ) for row in range( 0, len(matrix) ) )
    return idToRow, matrix

def getPolygon( url ): #this function reads a geometryFile
    points = 0
    try:
        f = open ( url, 'r' )
        points = [ map( int, line.split(',') ) for line in f ] #read file
    except Exception as e:
        print "Invalid or inexistent file "+ url +", using default geometry: (0,0)\n"
        points = [ [ 0,0 ] ]
    return [ ( Point( float(point[0])/1000, float(point[1])/1000, 0 ) ) for point in points ] #/1000 to convert to meters

def readRobotGeom():
    #Reading the 'robot' parameter
    geomDictionary = rospy.get_param('~robot' )
    ids = []
    for i in geomDictionary.keys():
        ids.append( int(i) )
    geoms = {}
    lastPoses = {}
    for i in ids:
        geoms[ i ] = getPolygon( geomDictionary[ str(i) ][ 'geom' ] )
        lastPoses[ i ] = []
        if i < 0:
            robot = RobotPose()
            robot.id = i
            robot.pose.x = geomDictionary[ str(i) ]['x']
            robot.pose.y = geomDictionary[ str(i) ]['y']
            robot.pose.theta = geomDictionary[ str(i) ]['theta']
            # robot time is 0 par default
            lastPoses[ robot.id ].append( robot )
    return ids, geoms, lastPoses

def shutDown():
    print("GPS end")

def gps():
    rospy.init_node('Gps', anonymous=False)

    #init params, read parameters and get the visibility map
    # read Objects file
    global idList, robotGeom, robotLastPoses
    idList, robotGeom, robotLastPoses = readRobotGeom()
    # read Map file
    try:
        keys, mapMatrix = readMapMatrix()
    except Exception as e:
        print "Invalid or inexistent file 'map', GPS cannot start \n Verify file URL, check README"
        raise rospy.ROSInterruptException
    rospy.on_shutdown( shutDown )
    global topicList
    topicList = initTopics( idList )
    rospy.Subscriber( "robot_pose" , RobotPose, setRobotPose )
    rate = rospy.Rate( HZ )
    while not rospy.is_shutdown():
        # a copy of ids is made to protect from 'setRobotPose()' adding more ids
        publish( list(idList), keys, mapMatrix )
        rate.sleep()

if __name__ == '__main__':
    try:
        gps()
    except rospy.ROSInterruptException:
        pass
