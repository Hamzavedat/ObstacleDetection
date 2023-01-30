#! /usr/bin/env python

import rospy
import math
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan


def clbk_laser(msg):
    # 640 / 3 = 213

    x = min(msg.ranges[0:212])
    y = min(msg.ranges[213:425])
    z = min(msg.ranges[426:639])
    
    regions = Float32MultiArray()
    regions.data = [x,y,z]

    pub.publish(regions)


if __name__ == '__main__':

    rospy.init_node('obstacle_detection')
    

    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    pub = rospy.Publisher('/obstacle/info', Float32MultiArray, queue_size=10)

    rospy.spin()



