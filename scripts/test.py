#!/usr/bin/env python
## THIS IS NOT PART OF THE MAIN APLICATION.
# IT IS JUST FOR FUNCTION AND SINTAX TESTS

import rospy
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import *
from multi_robots.msg import *

def publisher():
    # test dictionaty parameters
    #init params
    rospy.init_node('Test' , anonymous=False)
    dic = rospy.get_param('robot', 1 )
    print dic

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
