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
VELOCITY_LENGTH = 2 # n meters for each meter/second
VELOCITY_SCALE = Vector3(  0.02, 0.04, 0.04 ) # x diameter, y head diameter, z head length
VELOCITY_COLOR = ColorRGBA( 1, 170.0/255, 0, 1 )
TEXT_NS = "Text"
TEXT_SCALE = 0.1
TEXT_COLOR = ColorRGBA( 1, 1, 1, 1 )
TRAJECTORY_NS = "Trajectory"
TRAJECTORY_SCALE = Vector3( 0.008, 0.008, 0.008 )
TRAJECTORY_COLOR = ColorRGBA( 1, 65.0/255 , 90.0/255, 0.7 )
NUMBER_OF_LAST_POSES = 300
VISION_AREA_NS = "Vision_Area"
VISION_AREA_SCALE = Vector3( 0.01, 0.01, 0.01 )
VISION_AREA_COLOR = ColorRGBA( 1, 170.0/255 , 1, 0.7 )

PLANED_TRAJECTORY_NS = "Planed_Trajectory"
PLANED_TRAJECTORY_WIDTH =  0.020
PLANED_TRAJECTORY_COLOR = ColorRGBA( 85.0/255, 85.0/255, 1, 0.8 )
PLANED_TRAJECTORY_LINE_LEN = 0.200

PLANED_FIRST_SCALE = Vector3( 0.05, 0.05, 0.05 )
PLANED_FIRST_COLOR = ColorRGBA( 85.0/255, 1, 0, 1 )

FT = [ #Point( 0.01*x, 0.01*x, 0) for x in range(0,12) ] # to follow trajectory
    Pose2D( 0.3, 0.8, 0),
    Pose2D( 1.4, 2, 0),
    Pose2D( 1.4, 0.8, 0),
    Pose2D( 0.3, 2, 0),
]

def bound( n ): # values must be in [0-1] interval
    if n <= 1 and n >= 0:
        return n
    if n > 1:
        return 1
    if n < 0:
        return 0

def get_FT_Color():
    color = []
    r = 1
    g = 0
    b = 0
    a = 1
    dx = 6.0/len(FT)
    da = -1.0/len(FT)
    deltaColor = {
        0: [ 0, dx, 0 ], #r,g,b
        1: [ -dx, 0, 0 ], #r,g,b
        2: [ 0, 0, dx ], #r,g,b
        3: [ 0, -dx, 0 ], #r,g,b
        4: [ dx, 0, 0 ], #r,g,b
        5: [ 0, 0, -dx ], #r,g,b
    }
    for i in range( 0,len(FT) ):
        color.append( ColorRGBA( bound(r), bound(g), bound(b), bound(a) ) )
        if len(FT) >= 12:
            key = 6*i/len(FT)
            r += deltaColor[ key ][0]
            g += deltaColor[ key ][1]
            b += deltaColor[ key ][2]
        else:
            r = FOLLOW_TRAJECTORY_COLOR_LOW_NUMBER.r
            g = FOLLOW_TRAJECTORY_COLOR_LOW_NUMBER.g
            b = FOLLOW_TRAJECTORY_COLOR_LOW_NUMBER.b
        a = a + da
    return color

FOLLOW_TRAJECTORY_NS = "Follow_Trajectory"
FOLLOW_TRAJECTORY_SCALE = Vector3( 0.05, 0.05, 0.05 )
FOLLOW_TRAJECTORY_COLOR_LOW_NUMBER = ColorRGBA( 1, 1, 1, 1 ) #if less than 12 positions
FOLLOW_TRAJECTORY_COLOR = get_FT_Color()

lastPoses = [] # for painting the Trajectory
angle_of_vision = 0 # for painting the Vision_Area
ratio_of_vision = 0 # for painting the Vision_Area

SAVE_TO_TRAJECTORY_DISTANCE = 0.01
CHANGE_GOAL_DISTANCE = 0.06
HZ = 5 #frecuence of publish
behavior = "OA"
robotInfo = [ 0, 0 ] #actual and last information
me = [ 0, 0 ] # [0] actual position, [1] last position
#behaviors Variables
NOT_VISIBLE_DISTANCE = 3 # in m
#Region limits to change behavior, Distance in m
OA_AVOID_RATIO = 0.300
OA_2_RA = 0.120
RA_2_OA = 0.130
RA_2_S = 0.040
S_2_RA = 0.060

ZZT = [
    Pose2D( 0.3, 0.8, 0),
    Pose2D( 1.4, 2, 0),
    Pose2D( 1.4, 0.8, 0),
    Pose2D( 0.3, 2, 0),
]

PT = [] # planed trajectory

def getArc( angle, ratio  ):
    #rospy.loginfo( str(ratio) )
    ratio*=2 #FIXME, is not an real representative arc of visual points taked into acount
    angle = int( angle/math.pi*180 )
    #rospy.loginfo( 'angle ' +str(angle) )
    step =  10 # degrees
    arc = []
    for theta in range( -1*angle , angle + step, step ):
        arc.append( Point( ratio*math.cos( theta*math.pi/180 ), ratio*math.sin( theta*math.pi/180 ), 0 ) )
    arc.insert( 0, Point( 0, 0, 0) )
    arc.append( Point( 0, 0, 0) )
    pose = me[0].pose
    pose.theta += math.pi/2
    if pose.theta > math.pi:
        pose.theta -= math.pi*2
    return [ translatePoint( p, pose ) for p in arc ]

def distance( point1, point2 ):  #Computes the distance between two points
    dx = point1.x - point2.x
    dy = point1.y - point2.y
    return math.sqrt( dx**2 + dy**2 )

def rotatePoint( pointToRotate, angle, frame ):

    x = pointToRotate.x
    y = pointToRotate.y

    x0 = frame.x
    y0 = frame.y
    newPoint = Pose2D( 0, 0, 0 )
    #rotate
    newPoint.x = (x-x0)*math.cos( angle  ) - (y-y0)*math.sin( angle ) + x0
    newPoint.y = (x-x0)*math.sin( angle  ) + (y-y0)*math.cos( angle ) + y0

    return newPoint #pose2D

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
    FRAME = 'Local'#+str(uid)
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

    m = Marker()
    m.header.frame_id = FRAME
    m.ns = VISION_AREA_NS
    m.id = cont
    m.type = m.LINE_STRIP
    m.action = 0 # Add/Modify
    m.color = VISION_AREA_COLOR
    m.scale = VISION_AREA_SCALE
    m.points = getArc( angle_of_vision , ratio_of_vision )
    cont += 1
    markers.append( m )

    if len(PT) > 0:
        planed = list( PT )
        first = planed.pop(0)

        m = Marker()
        m.header.frame_id = FRAME
        m.ns = PLANED_TRAJECTORY_NS
        m.id = cont
        m.type = m.SPHERE
        m.action = 0 # Add/Modify
        m.color = PLANED_FIRST_COLOR
        m.scale = PLANED_FIRST_SCALE
        m.pose.position = Point( first.x, first.y, 0 )
        cont += 1
        markers.append( m )

        m = Marker()
        m.header.frame_id = FRAME
        m.ns = PLANED_TRAJECTORY_NS
        m.id = cont
        m.type = m.LINE_LIST
        m.action = 0 # Add/Modify
        m.color = PLANED_TRAJECTORY_COLOR
        m.scale.x = PLANED_TRAJECTORY_WIDTH
        lines = []
        for p in planed:
            lines.append( Point( p.x, p.y, 0 ) )
            point = Point( p.x, p.y, PLANED_TRAJECTORY_LINE_LEN )
            lines.append( point )
        m.points = lines
        cont += 1
        markers.append( m )

    if len(FT) > 0:
        m = Marker()
        m.header.frame_id = FRAME
        m.ns = FOLLOW_TRAJECTORY_NS
        m.id = cont
        m.type = m.CUBE_LIST
        m.action = 0 # Add/Modify
        m.colors = FOLLOW_TRAJECTORY_COLOR
        m.scale = FOLLOW_TRAJECTORY_SCALE
        lines = []
        for p in FT:
            lines.append( Point( p.x, p.y, 0 ) )
        m.points = lines
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
    if( me[0] != 0 ): # if actual position known
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

def closestPoint():  #returns the distance from the actual pose and angle of the closest point, in front of
    if len( robotInfo[0].obstacles) > 0: #check there are obstacles
        minimalDistance = robotInfo[0].obstacles[0]
    else:
        return NOT_VISIBLE_DISTANCE, 0, 0

    geometryPoints = [ translatePoint( p, me[0].pose ) for p in me[0].geometry ]

    for obstaclePoint in robotInfo[0].obstacles: #for each obstacle
        for point in geometryPoints: #for each point in robot geometry
            obstacleDistance = distance( point, obstaclePoint ) #get distance
            if obstacleDistance < minimalDistance:
                obst = obstaclePoint
                minimalDistance = obstacleDistance
                angle = math.atan2( obstaclePoint.y - me[0].pose.y, obstaclePoint.x - me[0].pose.x ) #get the direction of the point
    return minimalDistance, angle, obst

def getObstaclesIn( ratio ): # return obstacles in a ratio from me, TODO add also visible robot geometries
    l = []
    for obstacle in robotInfo[0].obstacles: #for each obstacle
        if distance( obstacle, me[0].pose ) < ratio: # TODO make it dependent of robot geometry
            l.append( obstacle )
    return l

def checkTurningObstacles( goal ):

    goalAngle = math.atan2( goal.y - me[0].pose.y, goal.x - me[0].pose.x ) #get the angle to the goal
    #print  "Goal Angle ", goalAngle
    step = 10
    diff = math.pi/2 + me[0].pose.theta - goalAngle # + pi/2 because of the frame of robot. it moves forward in y axis
    while diff > math.pi:
        diff -= 2*math.pi
    while diff < -1*math.pi:
        diff += 2*math.pi
    if diff > 0 : # if i have to turn clock wise
        step *= -1
    #print "diff", diff*180/math.pi

    obstacles = getObstaclesIn( 1.5*RA_2_OA ) # TODO make ratio dependent of robot geometry
    dtheta = 0
    while abs( dtheta ) < abs( diff ):
        newPose = Pose2D( me[0].pose.x, me[0].pose.y, me[0].pose.theta )
        newPose.theta += dtheta
        geometryPoints = [ translatePoint( p, newPose ) for p in me[0].geometry ] # translate geometries
        for p in geometryPoints:
            for o in obstacles:
                if distance( p, o ) <= RA_2_S:
                    maxDTheta = -1*( goalAngle - 60*math.pi/180 )
                    print "enter"

                    newGoal = rotatePoint( goal, maxDTheta, me[0].pose )
                    return newGoal
        dtheta += step * math.pi/180
    return goal

# BEHAVIORS
def run_away():
    if( me[0] != 0 ): # if actual position known
        if distance( me[0].pose, PT[0] ) < 2*CHANGE_GOAL_DISTANCE:
            global PT
            PT.pop(0)
            return 0
        elif distance( me[0].pose, PT[1] )*2/3 < distance( me[0].pose, PT[0] ):
            global PT
            PT.pop(0)

        minimalDistance, alfa, obstacle = closestPoint()
        if minimalDistance >= RA_2_OA: #if no obstacles
            setBehavior( "OA" ) #change behavior
            return 0
        elif minimalDistance <= RA_2_S and abs(alfa - me[0].pose.theta) < math.pi/2 : #if TOO close
            setBehavior( "S" ) #change behavior
            return stop()
        else:
            global ratio_of_vision
            ratio_of_vision = RA_2_OA
            ratio = RA_2_OA # goal from here, in meters
            angle = math.atan2( PT[0].y - me[0].pose.y, PT[0].x - me[0].pose.x ) #get the angle to the goal

            diff = abs( me[0].pose.theta + math.pi/2 - alfa )
            if diff > math.pi: # verifies that angle is between 0 and pi
                diff -= 2*math.pi
                diff = abs(diff)

            global angle_of_vision
            angle_of_vision = math.pi*3/8
            if diff < angle_of_vision : # true if osbtacle is in front of the robot
                #get first posibility
                angle1 = alfa + math.pi/3
                if angle1 > math.pi: # verifies that angle is between -pi and pi
                    angle1 -= 2*math.pi
                point1 = Pose2D( ratio*math.cos(angle1) + me[0].pose.x, ratio*math.sin(angle1) + me[0].pose.y, 0 )
                #get second posibility
                angle2 = alfa - math.pi/3
                if angle2 < -1*math.pi: # verifies that angle is between -pi and pi
                    angle2 += 2*math.pi
                point2 = Pose2D( ratio*math.cos(angle2) + me[0].pose.x, ratio*math.sin(angle2) + me[0].pose.y, 0 )
                #return the closest to goal
                pointToReturn = 0
                if distance( point1, PT[0] ) < distance( point2, PT[0] ):
                    pointToReturn = checkTurningObstacles( point1 ) #takes the goal and returns a goal with an angle closest to the goal without obstacles
                else:
                    pointToReturn = checkTurningObstacles( point2 )
                return pointToReturn
            else:
                ratio *= 2
                return Pose2D( ratio*math.cos(angle) + me[0].pose.x, ratio*math.sin(angle) + me[0].pose.y, 0 )
    else: # if actual position UNknown
        return 0

def zig_zag_trajectory():

    if( me[0] != 0 ):
        if distance( me[0].pose, ZZT[0] ) < 2*CHANGE_GOAL_DISTANCE:
            global ZZT
            ZZT.insert( 0, ZZT.pop() ) # FIXME
    return ZZT[0]

def stop():
    if( me[0] != 0 ): # if actual position known
        global ratio_of_vision
        ratio_of_vision = S_2_RA
        global angle_of_vision
        angle_of_vision = math.pi

        minimalDistance, alfa, obstacle  = closestPoint()
        if len( FT ) == 0:
            return Pose2D( me[0].pose.x, me[0].pose.y, 0 ) # return a goal at actual position
        elif minimalDistance >= S_2_RA: #if no obstacles
            setBehavior( "RA" ) #change behavior
            return run_away()
        else: # return a goal at actual position
            return Pose2D( me[0].pose.x, me[0].pose.y, 0 )
    else: # if actual position UNknown
        return 0

def obstacle_avoidance():
    if( me[0] != 0 ): # if actual position known
        #verify if goal is acomplished
        if distance( me[0].pose, PT[0] ) < 2*CHANGE_GOAL_DISTANCE:
            global PT
            PT.pop(0)
        elif distance( me[0].pose, PT[1] ) < distance( me[0].pose, PT[0] ):
            global PT
            PT.pop(0)
            #print "yes"

        minimalDistance, alfa, obstacle = closestPoint()
        if minimalDistance >= OA_AVOID_RATIO:
            if len(PT) > 1:
                return Pose2D ( PT[ 1 ].x, PT[ 1 ].y, 0 )
            else:
                return Pose2D ( PT[ 0 ].x, PT[ 0 ].y, 0 )

        if minimalDistance <= OA_2_RA: #if TOO close
            setBehavior( "RA" ) #change behavior
            return 0
        else: # return a goal perpendicular to the obstacle; if the obstacle is in the goal path
            global ratio_of_vision
            ratio_of_vision = OA_AVOID_RATIO # goal from here, in meters
            ratio = ratio_of_vision
            if len(PT) > 1: # To go faster if there are other targets in path
                angle = math.atan2( PT[1].y - me[0].pose.y, PT[1].x - me[0].pose.x ) #get the angle to the goal
            else:
                angle = math.atan2( PT[0].y - me[0].pose.y, PT[0].x - me[0].pose.x ) #get the angle to the goal

            diff = abs( me[0].pose.theta + math.pi/2 - alfa )
            if diff > math.pi: # verifies that angle is between 0 and pi
                diff -= 2*math.pi
                diff = abs(diff)

            global angle_of_vision
            angle_of_vision = 50*math.pi/180
            if diff < angle_of_vision: # true if osbtacle is in front of the robot
                #get first posibility
                #ratio /=  #to slow the speed
                angle1 = alfa + angle_of_vision
                if angle1 > math.pi: # verifies that angle is between -pi and pi
                    angle1 -= 2*math.pi
                point1 = Pose2D( ratio*math.cos(angle1) + me[0].pose.x, ratio*math.sin(angle1) + me[0].pose.y, 0 )
                #get second posibility
                angle2 = alfa - angle_of_vision
                if angle2 < -1*math.pi: # verifies that angle is between -pi and pi
                    angle2 += 2*math.pi
                point2 = Pose2D( ratio*math.cos(angle2) + me[0].pose.x, ratio*math.sin(angle2) + me[0].pose.y, 0 )
                #return the closest to goal
                index = 0
                if len(PT) > 1: # To go faster if there are other targets in path
                    index = 1
                if distance( point1, PT[index] ) < distance( point2, PT[index] ):
                    return point1
                else:
                    return point2
            else:
                return Pose2D( ratio*math.cos(angle) + me[0].pose.x, ratio*math.sin(angle) + me[0].pose.y, 0 )
    else:
        return 0
def rendez_vous():
    if( me[0] != 0 ): # if actual position known
        rendez_vous_goal = Pose2D( 0, 0, 0 )
        for i in robotInfo[0].robots:
            rendez_vous_goal.x += i.pose.x
            rendez_vous_goal.y += i.pose.y
        rendez_vous_goal.x += me[0].pose.x
        rendez_vous_goal.y += me[0].pose.y

        rendez_vous_goal.x /= len( robotInfo[0].robots )+1
        rendez_vous_goal.y /= len( robotInfo[0].robots )+1
        return rendez_vous_goal
    else: # if actual position UNknown
        return 0
#END OF BEHAVIORS

def getCenterOfCell( row, column, SIZE, N_ROWS, N_COLUMNS  ):
    #get cell centre
    pX = me[0].pose.x + (column - N_COLUMNS/2)*SIZE
    pY = me[0].pose.y + (row - N_ROWS/2)*SIZE
    return Point( pX, pY, 0 )


def isOcupied( row, column, SIZE, N_ROWS, N_COLUMNS ):
    #already checked for me[0]
    #get cell centre
    p = getCenterOfCell( row, column, SIZE, N_ROWS, N_COLUMNS  )
    #get corners
    right = p.x + SIZE/2
    up = p.y + SIZE/2
    left = p.x - SIZE/2
    down = p.y - SIZE/2
    #verify all posible points and return
    for point in robotInfo[0].obstacles: # check in obstacles
        if left < point.x and point.x < right:
            if down < point.y and point.y < up:
                return True
    for robot in robotInfo[0].robots: #check in robots
        if left < robot.pose.x and robot.pose.x < right:
            if down < robot.pose.y and robot.pose.y < up:
                return True
    return False

def isPathBlocked():
    CHECK_DISTANCE = 0.100
    for p in PT:
        for point in robotInfo[0].obstacles: # check in obstacles
            if distance( point, p ) < CHECK_DISTANCE :
                return True
        for robot in robotInfo[0].robots: #check in robots
            if distance( robot.pose , p  ) < CHECK_DISTANCE and robot != me[0] :
                return True
    return False

def isGoalAchieved( row, column, SIZE, N_ROWS, N_COLUMNS ):
    #already checked for me[0]
    #get cell centre
    p = getCenterOfCell( row, column, SIZE, N_ROWS, N_COLUMNS  )
    #get corners
    right = p.x + SIZE/2
    up = p.y + SIZE/2
    left = p.x - SIZE/2
    down = p.y - SIZE/2
    #verify all posible points and return
    if left < FT[0].x and FT[0].x < right:
        if down < FT[0].y and FT[0].y < up:
            return True
    return False

def inGridBounds( i, j, N_ROWS, N_COLUMNS ):
    if 0 <= i and i <  N_ROWS:
        if 0 <= j and j < N_COLUMNS:
            return True
    return False

directions = [
    [ 1, 0 ], # up
    [ -1, 0 ], # down
    [ 0, 1 ], # righ
    [ 0, -1 ], # left
    [ 1, 1 ], # up right
    [ -1, 1 ], # down right
    [ -1, -1 ], # down left
    [ 1, -1 ], # up left
]

def getPath( grid, goalIndex, SIZE, N_ROWS, N_COLUMNS  ):
    i, j = goalIndex
    point = getCenterOfCell( i, j, SIZE, N_ROWS, N_COLUMNS )
    value = grid[i][j]
    #print value
    if value == 0:
        #path = [ getCenterOfCell(i, j, SIZE, N_ROWS, N_COLUMNS) ]
        path = []
        return path
    else:
        for index in directions:
            di, dj = index
            if inGridBounds( i+di, j+dj, N_ROWS, N_COLUMNS ):
                if grid[i+di][j+dj] == value - 1:
                    path = getPath( grid, [ i+di, j+dj ], SIZE, N_ROWS, N_COLUMNS  )
                    path.append(point)
                    return path

def doPlanning():
    GRID_SIZE = 11
    ROWS = GRID_SIZE
    COLUMNS = GRID_SIZE
    CELL_LEN = 0.120 + 2*RA_2_S # meters #TODO select dinamic

    if( me[0] != 0 ): # if actual position known
    #verify if goal is acomplished
        if distance( me[0].pose, FT[0] ) < CHANGE_GOAL_DISTANCE:
            global FT
            FT.insert( 0, FT.pop() ) # FIXME tmp
            #FT.pop(0)
            doPlanning()
        else:
            grid = [[-2 for x in range(COLUMNS)] for y in range(ROWS)]
            # put -1 if there are obstacles in the grid
            for i in range( ROWS ):
                for j in range( COLUMNS ):
                    if isOcupied( i, j, CELL_LEN, ROWS, COLUMNS ):
                        grid[i][j] = -1
            # Solve the grid
            # assure that algorithm starts even if theres an obstacle in the initial cell
            grid[ROWS/2][COLUMNS/2] = -2
            stackOld = [] # to pop indices
            stackOld.append( [ROWS/2, COLUMNS/2] )
            stackNew = [] # to push indices
            counter = 0

            goalIndex = []
            while len(stackOld) != 0:
                while len(stackOld) != 0:
                    i, j = stackOld.pop()
                    if isGoalAchieved( i, j, CELL_LEN, ROWS, COLUMNS ):
                        grid[i][j] = counter
                        stackOld = []
                        stackNew = []
                        goalIndex = [ i, j ]
                    if( grid[i][j] == -2 ):
                        grid[i][j] = counter
                        for index in directions:
                            di, dj = index
                            if inGridBounds( i+di, j+dj, ROWS, COLUMNS ):
                                stackNew.append( [i+di, j+dj] )
                counter += 1
                stackOld = stackNew
                stackNew = []
            # select a trajectory
            #print grid
            path = []
            if len( goalIndex ) == 2: # goal reached
                #get shortest path
                path = getPath( grid, goalIndex, CELL_LEN, ROWS, COLUMNS )
            else:
                #find closest point diferent from -1 and get shortest path
                p = getCenterOfCell( 0, 0, CELL_LEN, ROWS, COLUMNS )
                minDistancePoint = p
                minDistance = distance( FT[0], minDistancePoint ) + 1
                pointIndex = 0

                for i in range(ROWS):
                    for j in range(COLUMNS):
                        if grid[i][j] != -1:
                            center = getCenterOfCell( i, j, CELL_LEN, ROWS, COLUMNS )
                            if distance( center, FT[0] ) < minDistance:
                                minDistancePoint = center
                                minDistance = distance( center, FT[0] )
                                pointIndex = [ i, j ]

                path = getPath( grid, pointIndex, CELL_LEN, ROWS, COLUMNS  )
            #asign the computed path
            if len(path) <= 1:
                path.append( FT[0] )
                path.append( FT[0] )
            global PT
            PT = path

def setBehavior( behav ):
    global behavior
    behavior = behav

def setGoal( behavior ):
    states = {
        "RV": rendez_vous,
        "ZZT": zig_zag_trajectory,
        "OA": obstacle_avoidance,
        "RA": run_away, #FIXME change to slow_OA
        "S": stop,
    }
    if len( FT ) == 0: # if there are no goals, do nothing
        return stop()
    elif behavior == "OA" or behavior == "RA":
        if len(PT) < 2 or isPathBlocked():
            doPlanning()
    function = states.get( behavior, obstacle_avoidance ) #if behavior not in states, return OA
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
        #print behavior
        newGoal = setGoal( str( behavior ) )
        if newGoal != 0 and newGoal != None :
            #print newGoal
            goalPoint = PointStamped()
            goalPoint.header.stamp = rospy.Time().now()
            goalPoint.header.frame_id = 'Local'
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
