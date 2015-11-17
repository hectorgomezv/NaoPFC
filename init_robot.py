#!/usr/bin/env python
# -*- encoding: UTF-8 -*-
import rospy
import sys
from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import motion
import almath

#NAO_IP = '169.254.222.39'
NAO_IP = '127.0.0.1'

def getIP():
	return NAO_IP

def stiffness_on(motionProxy):
	"""Activa los motores de movimiento del Nao para todos los controles. 
	Esta función debe ser ejecutada antes de requerir al robot cualquier 
	tipo de movimiento. 
	"""
	p_names = "Body"
	p_stiffness_list = 1.0
	p_time_lists = 1.0
	motionProxy.stiffnessInterpolation(p_names, p_stiffness_list, p_time_lists)

def init_pos(postureProxy):
	"""Coloca al robot en la estable posición inicial. """
	postureProxy.goToPosture("StandInit", 0.4)	

def start_motion():
	"""Devuelve una instancia (proxy) al módulo ALMotion de Naoqi,
	que controla el movimiento del robot.
	""" 
	try:
		motionProxy = ALProxy("ALMotion", NAO_IP, 9559)
	except Exception, e:
		print "Couldn't create proxy to ALMotion.\nError was:", e
	return motionProxy	

def start_posture():
	"""Devuelve una instancia (proxy) al módulo ALPostura de Naoqi,
	que controla la postura del robot.
	"""
	try:
		postureProxy = ALProxy("ALRobotPosture", NAO_IP, 9559)
	except Exception, e:
		print "Couldn't create proxy to ALRobotPosture.\nError was:", e		
	return postureProxy	
			
def start_redBallTracker():
	"""Devuelve una instancia (proxy) al módulo ALRedBallTracker 
	de Naoqi, que permite hace seguimiento de una pelota a través de
	la cámara del robot.
	"""
	try:
		redBallTrackerProxy = ALProxy("ALRedBallTracker", NAO_IP, 9559)
	except Exception, e:
		print "Couldn't create proxy to ALRedBallTracker.\nError was:", e		
	return redBallTrackerProxy

def start_memory():
	"""Devuelve una instancia (proxy) al módulo ALMemory de Naoqi,
	que gestiona la memoria interna del robot.
	"""
	try:
		memProxy = ALProxy("ALMemory", NAO_IP, 9559)
	except Exception, e:
		print "Couldn't create proxy to ALMemory.\nError was:", e		
	return memProxy	

def start_mark_detection():	
	"""Devuelve una instancia (proxy) al módulo ALLandMarkDetection 
	de Naoqi, que permite la detección de NaoMarks a través de
	la cámara del robot.
	"""
	try:
		markProxy = ALProxy("ALLandMarkDetection", NAO_IP, 9559)
	except Exception, e:
		print "Couldn't create proxy to ALLandMarkDetection.\nError was:", e		
	return markProxy	

def main():
	pass

if __name__ == '__main__':
	main()