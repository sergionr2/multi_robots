#!/usr/bin/env python
##

import rospy

NAME = "./border.txt"
# ALL IN mm
W = 1700
H = 2400
X = 0
Y = 0
DELTA = 50

def makeIt():

    array = []
    x = 0
    y = 0
    while( x < W ):
        array.append( [x,y] )
        x += DELTA
    while( y < H ):
        array.append( [x,y] )
        y += DELTA
    while( x > 0 ):
        array.append( [x,y] )
        x -= DELTA
    while( y > 0 ):
        array.append( [x,y] )
        y -= DELTA
    for p in array:
        p[0] -= X
        p[1] -= Y
    print array
    f = open( NAME, 'w')
    for p in array:
        f.write( str( p[0] ) +','+ str( p[1] ) + '\n' )

if __name__ == '__main__':
        makeIt()
