#! /usr/bin/env python

import rospy
from std_msgs.msg import String, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
import threading
import math
from tf import transformations

obstacle = 'no'

initial_position = Point()
initial_position.x = 0
initial_position.y = 0
initial_position.z = 0
position_ = Point()
target_position = Point()
target_position.x = 0
target_position.y = 0
target_position.z = 0
yaw_ = 0
state_robot = 0
state_obs = 0
yaw_precision_ = math.pi/90 # +/- 2 degree allowed
dist_precision_ = 0.3
regions = [10.0,10.0,10.0]

def clbk_obs(msg):

	global obstacle, state_obs, regions
	regions = msg.data
	if(regions[0] < 1.5 and regions[1] < 1.5 and regions[2] < 1.5):
		state_obs = 1
	elif(regions[0] < 1.5 and regions[1] < 1.5 ):
		state_obs = 1
	elif( regions[1] < 1.5 and regions[2] < 1.5):
		state_obs = 1                                                                                                                                                                         
	elif(regions[0] < 1.5 and regions[2] < 1.5):
		state_obs = 0
	elif(regions[0] < 1.5):
		state_obs = 2
	elif(regions[1] < 1.5 ):
		state_obs = 1
	elif(regions[2] < 1.5):
		state_obs = 0
	else:
		state_obs = 0


def clbk_odom(msg):
	global position_, yaw_

	# position
	position_ = msg.pose.pose.position

	# yaw
	quaternion = (
		msg.pose.pose.orientation.x,
		msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z,
		msg.pose.pose.orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	yaw_ = euler[2]

#def clbk_trgt(msg):
	#global target_position, initial_position, position_
	#target_position = msg 
	#initial_position = position_

def distance_to_line(p0):
	# p0 is the current position
	# p1 and p2 points define the line
	global initial_position , target_position
	p1 = initial_position
	p2 = target_position
	# here goes the equation
	up_eq = math.fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x))
	lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
	distance = up_eq / lo_eq

	return distance

def normalize_angle(angle):
	if(math.fabs(angle) > math.pi):
		angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
	return angle

def err():
	global yaw_, position_, target_position 
	desired_yaw = math.atan2(target_position.y - position_.y, target_position.x - position_.x)
	err_yaw = normalize_angle(desired_yaw - yaw_)
	err_pos = math.sqrt(pow(target_position.y - position_.y, 2) + pow(target_position.x - position_.x, 2))
	
	return err_yaw, err_pos  

def command():
	while True:
		global state_robot, yaw_precision_, dist_precision_
		msg = Twist()
		if state_robot == 0:
			err_yaw,err_pos = err()
			if err_pos > dist_precision_:
				#if math.fabs(err_yaw) > 5*yaw_precision_:
					#msg.angular.z = 0.6 if err_yaw > 0 else -0.6
				#elif math.fabs(err_yaw) > 3*yaw_precision_:
					#msg.angular.z = 0.3 if err_yaw > 0 else -0.3
				#elif math.fabs(err_yaw) > 2*yaw_precision_:
					#msg.angular.z = 0.1 if err_yaw > 0 else -0.1
					#msg.linear.x = 0.3
				#elif math.fabs(err_yaw) > yaw_precision_:
					#msg.angular.z = 0.05 if err_yaw > 0 else -0.05
					#msg.linear.x = 0.4
				if math.fabs(err_yaw) <= 2*yaw_precision_:
					msg.linear.x = 0.6
				else:
					i = math.fabs(err_yaw) / yaw_precision_
					msg.angular.z = i / 35 if err_yaw > 0 else -i / 35
					msg.angular.x = 1/i 
			else:
				print 'Position error: [%s]' % err_pos
				msg.linear.x = 0
				msg.angular.z = 0
			pub.publish(msg)
		
		elif state_robot == 1:
		    
			if state_obs == 0:
				msg.linear.x = 0.3
				msg.angular.z = -0.3
			elif state_obs == 1:
				msg.angular.z = 0.3
			elif state_obs == 2:
				msg.linear.x = 0.5
			pub.publish(msg)


rospy.init_node('navigation')
rospy.Subscriber('/obstacle/info', Float32MultiArray, clbk_obs)
rospy.Subscriber('/odom', Odometry, clbk_odom)
#rospy.Subscriber('/target/info', Point, clbk_trgt)
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

target_position.x = input('X:')
target_position.y = input('Y:')

initial_position = position_

thr_command = threading.Thread(target=command)
thr_command.start()


rate = rospy.Rate(30)
count_state_time_ = 0
count_loop_ = 0

while not rospy.is_shutdown():    
	distance_position_to_line = distance_to_line(position_)
	err_yaw,err_pos = err()
	if state_robot == 0:
		print(regions[1])
		if regions[1] < 1.5 and math.fabs(err_yaw) < 3*yaw_precision_:
			state_robot = 1
			count_state_time_ = 0

	elif state_robot == 1:
		print('1')
		if distance_position_to_line < 0.1 and count_state_time_ > 5:
			state_robot = 0
			count_state_time_ = 0

	count_loop_ = count_loop_ + 1
	if count_loop_ == 30:
		count_state_time_ = count_state_time_ + 1
		count_loop_ = 0
	rate.sleep()


