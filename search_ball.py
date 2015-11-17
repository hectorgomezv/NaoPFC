#!/usr/bin/env python
# -*- encoding: UTF-8 -*-
import time
import math
import almath
import motion

NAO_IP = '127.0.0.1'


def ball_detected(redBallTrackerProxy):
	""" Función que devuelve true si se está detectando la bola en estos momentos. """
	return redBallTrackerProxy.isNewData()

def move_head_right(motionProxy, redBallTrackerProxy):
	""" Método que mueve la cabeza hacia la derecha n grados."""
	redBallTrackerProxy.stopTracker()
	names = 'HeadYaw'
	angleLists = [10*almath.TO_RAD, 20*almath.TO_RAD]
	timeLists = [1.0, 2.0]
	isAbsolute = True
	motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
	redBallTrackerProxy.startTracker()

def move_head_left(motionProxy, redBallTrackerProxy):
	""" Método que mueve la cabeza hacia la izquierda n grados."""
	redBallTrackerProxy.stopTracker()
	names = 'HeadYaw'
	angleLists = [-10*almath.TO_RAD, -20*almath.TO_RAD]
	timeLists = [1.0, 2.0]
	isAbsolute = True
	motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
	redBallTrackerProxy.startTracker()

def down_head(motionProxy):
	""" Método que mueve la cabeza hacia abajo. """
	names = 'HeadPitch'
	angleLists = [8*almath.TO_RAD, 16*almath.TO_RAD]
	timeLists = [1.0, 2.0]
	isAbsolute = True
	motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)

def up_head(motionProxy, redBallTrackerProxy):
	""" Método que mueve la cabeza hacia abajo. """
	redBallTrackerProxy.stopTracker()
	names = 'HeadPitch'
	angleLists = [-8*almath.TO_RAD, -16*almath.TO_RAD]
	timeLists = [1.0, 2.0]
	isAbsolute = True
	motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
	redBallTrackerProxy.startTracker()

def search(videoDevice, motionProxy, redBallTrackerProxy):
	""" Método que mueve la cabeza y cambia la cámara en tanto no se detecta la bola."""
	if not ball_detected(redBallTrackerProxy):
		move_head_right(motionProxy, redBallTrackerProxy)
	if not ball_detected(redBallTrackerProxy):
		move_head_left(motionProxy, redBallTrackerProxy)
		move_head_left(motionProxy, redBallTrackerProxy)
	if not ball_detected(redBallTrackerProxy):
		move_head_right(motionProxy, redBallTrackerProxy)
	return ball_detected(redBallTrackerProxy)		

def main():
	pass
			
if __name__ == '__main__':
	main()	