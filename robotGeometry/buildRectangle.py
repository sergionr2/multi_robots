#!/usr/bin/env python
##

import rospy

NAME = "./Border.txt"
# ALL IN mm
W = 1000
H = 2000
X = 500
Y = 1000
DELTA = 500

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
