#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point

rospy.init_node('target_position')

pub = rospy.Publisher('/target/info', Point, queue_size=10)

position = Point()
while not rospy.is_shutdown():
	target_position = Point()
	target_position.x = input('X:')
	target_position.y = input('Y:')
	target_position.z = 0
	if target_position != position:
		pub.publish(target_position)
		target_position = position
