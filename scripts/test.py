#!/usr/bin/env python
## THIS IS NOT PART OF THE MAIN APLICATION.
# IT IS JUST FOR FUNCTION AND SINTAX TESTS

import rospy
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import *
from multi_robots.msg import *
url = '/home/multi-robots/catkin_ws/src/multi_robots/GPSconfig/objects.txt'
focus = rospy.get_param("/usb_cam/focus")
def shutDown():
    print("end")

def initTopics( ids ):
    pubs = {}
    for i in ids:
        if i >= 0:
            pubs[i] = rospy.Publisher( 'info_' + str( i ), GPSinfo, queue_size=10 )

    return pubs

def lol():
    return 0,1

def publisher():
    print "test started"
    f = open ( url , 'r')
    l = [ map(str, line.split(',') ) for line in f ]
    d = dict( ( int(i[0]), i[1].rstrip('\n') ) for i in l )
    rospy.init_node('TEST', anonymous=False)
    g,h = lol()
    print g,h
    try:
        i = 2
        print i+1
    except Exception as e:
        print 'laca'
        raise rospy.ROSInterruptException

    #print d
    lista = [0,-2,-5,25,1,2,3,4,5]
    pubs = initTopics( lista )
    print lista.count(2)
    print lista.count(-2)
    print 0 == lista.count(8)

    rospy.spin()

if __name__ == '__main__':
    try:
        print focus
        publisher()
    except rospy.ROSInterruptException:
        pass
