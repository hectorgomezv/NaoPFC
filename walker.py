#!/usr/bin/env python
# -*- encoding: UTF-8 -*-
import init_robot
import rospy
import sys
import math
from naoqi import ALProxy
from naoqi import ALBroker
from geometry_msgs.msg import Point

def calculate_theta(ball):
	theta = math.copysign(math.pi/16, ball.y)
	return theta 

def move_robot(motionProxy, ball):
	theta = calculate_theta(ball)
	motionProxy.walkTo(0.1, 0.0, theta)

def main():
	motionProxy = init_robot.start_motion()
	rospy.init_node('walker', anonymous=True)
	if rospy.get_param('lower'):
		sys.exit()
	ball = rospy.wait_for_message('pos', Point)
	while ball.x > 0.45:
		move_robot(motionProxy, ball)
		ball = rospy.wait_for_message('pos', Point)
	# Una vez hemos concluido el acercamiento con el walker...
	rospy.set_param('lower', True)

if __name__ == '__main__':
	main()
