#!/usr/bin/env python
## this script writes a robotGeometry file of a rectangle

import rospy

NAME = "./border.txt"
# ALL IN mm
WIDTH = 1700
HEIGHT = 2400
ORIGIN_X = 0  # to place the robot coordinate system in
ORIGIN_Y = 0
DELTA = 50

# with ORIGIN_X = 0, and ORIGIN_Y = 0
# you will get a rectangle from (0,0) to (WIDTH, HEIGHT) with the robot coordinate system at (0,0)


def makeIt():

    array = []
    x = 0
    y = 0
    while( x < WIDTH ):
        array.append( [x,y] )
        x += DELTA
    while( y < HEIGHT ):
        array.append( [x,y] )
        y += DELTA
    while( x > 0 ):
        array.append( [x,y] )
        x -= DELTA
    while( y > 0 ):
        array.append( [x,y] )
        y -= DELTA
    for p in array:
        p[0] -= ORIGIN_X
        p[1] -= ORIGIN_Y
    print array
    f = open( NAME, 'w')
    for p in array:
        f.write( str( p[0] ) +','+ str( p[1] ) + '\n' )

if __name__ == '__main__':
        makeIt()
