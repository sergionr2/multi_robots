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

def publisher():
    # test dictionaty parameters
    #init params
    rospy.init_node('Test' , anonymous=False)
    dic = rospy.get_param('geom', 1 )
    print dic
    pub = rospy.Publisher( 'robot_pose', RobotPose, queue_size=10, latch=True)
    pub.publish( init )

if __name__ == '__main__':
    try:
        print focus
        publisher()
    except rospy.ROSInterruptException:
        pass
