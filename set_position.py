#!/usr/bin/env python
# -*- encoding: UTF-8 -*-
import init_robot
import rospy
import sys
import math
import time
import camera
from naoqi import ALProxy
from naoqi import ALBroker
from geometry_msgs.msg import Point

TARGET_X = 0.12
TARGET_Y = -0.05
ERROR_X = 0.01
ERROR_Y = 0.005
STEP_X = 0.03
STEP_Y = 0.015

def inrange(x, min, max):
	""" Función que devuelve True si x pertenece al intervalo [min, max] """
	return (min is None or min <= x) and (max is None or max >= x)

def calculate_x_step(ball):
	""" Función que calcula el desplazamiento en el sentido del eje x. """
	if inrange(ball.x, TARGET_X-ERROR_X, TARGET_X+ERROR_X):
		return 0.0
	elif ball.x > TARGET_X:
		return STEP_X
	elif ball.x < TARGET_X:
		return -STEP_X	
	else:
		return -1

def calculate_y_step(ball):
	""" Función que calcula el desplazamiento en el sentido del eje y. """
	if inrange(ball.y, TARGET_Y-ERROR_Y, TARGET_Y+ERROR_Y):
		return 0.0
	elif ball.y > TARGET_Y:
		return STEP_Y
	elif ball.y < TARGET_Y:
		return -STEP_Y	
	else:
		return -1

def set_position(motionProxy, ball):
	# rospy.loginfo(ball)
	move_x, move_y = calculate_x_step(ball), calculate_y_step(ball)
	if move_x == -1 or move_y == -1:
		rospy.loginfo('set_position ERROR -1\n')
		sys.exit()
	motionProxy.walkTo(move_x, move_y, 0.0)	

def main():
	motionProxy = init_robot.start_motion()
	rospy.init_node('set_position', anonymous=True)
	while not rospy.get_param('lower'):
		time.sleep(0.5)
	# Cuando otro módulo hace lower True...
	videoDeviceProxy = camera.init_camera()
	camera.low_camera(videoDeviceProxy)
	ball = rospy.wait_for_message('pos', Point)
	while not inrange(ball.x, TARGET_X-ERROR_X, TARGET_X+ERROR_X) or \
		not inrange(ball.y, TARGET_Y-ERROR_Y, TARGET_Y+ERROR_Y):
		set_position(motionProxy, ball)
		ball = rospy.wait_for_message('pos', Point)
	rospy.set_param('kick_now', True)

if __name__ == '__main__':
	main()
