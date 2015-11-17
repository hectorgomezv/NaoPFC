# -*- encoding: UTF-8 -*-
#!/usr/bin/env python
import sys
import time
import motion
import almath
import rospy
from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

from optparse import OptionParser

NAO_IP = "127.0.0.1"

# Global variable to store the ballSpy module instance
ballSpy = None
memory = None

def stiffness_on(motionProxy):
	p_names = "Body"
	p_stiffness_list = 1.0
	p_time_lists = 1.0
	motionProxy.stiffnessInterpolation(p_names, p_stiffness_list, p_time_lists)
	#print motionProxy.getSummary()


class ballSpyModule(ALModule):
	""" Módulo para detección y reacción.
	"""

	def __init__(self, name):
		ALModule.__init__(self, name)
		
		try:
			redBallDetectionProxy = ALProxy("ALRedBallDetection", NAO_IP, 9559)
		except Exception, e:
			print "Couldn't create proxy to ALRedBallDetection.\nError was:", e

		try:
			global memProxy
			memProxy = ALProxy("ALMemory", NAO_IP, 9559)
		except Exception, e:
			print "Couldn't create proxy to ALMemory.\nError was:", e

		memProxy.subscribeToEvent("redBallDetected", "ballSpy", "onBallDetected")
		
	def onBallDetected(self, *_args):
		""" Función callback para una detección de balón.
		"""
		# Para evitar que se siga lanzando el callback mientras se ejecuta.
		memProxy.unsubscribeToEvent("redBallDetected", "ballSpy")

		# Código que se ejecuta cuando se lanza el callback y hace cosas.
		data = memProxy.getData("redBallDetected")
		TimeStamp = data[0]
		BallInfo = data[1]
		CameraPose_InTorsoFrame = data[2]
		CameraPose_InRobotFrame = data[3]
		Camera_Id = data[4]
		print "TimeStamp: " + str(TimeStamp) + str(type(TimeStamp))
		print "BallInfo: centerX[" + str(data[1][0]) + type(data[1][0]) + "] centerY[" + str(data[1][1]) + type(data[1][0]) + "] sizeX[" + str(data[1][2]) + "] sizeY[" + str(data[1][3]) + "]"
		# print "CameraPose_InTorsoFrame: " + str(CameraPose_InTorsoFrame)
		# print "CameraPose_InRobotFrame: " + str(CameraPose_InRobotFrame)
		print "Camera_Id: " + str(Camera_Id)

		# Volvemos a susribirnos al evento una vez se ha ejecutado el callback.
		memProxy.subscribeToEvent("redBallDetected", "ballSpy", "onBallDetected")



def main():
	""" Punto de entrada al Módulo.
	"""

	parser = OptionParser()
	parser.add_option("--pip",
		help = "Parent broker IP, robot IP address",
		dest = "pip")
	parser.add_option("--pport",
		help = "Parent broker IP, NaoQi port",
		dest = "pport",
		type = "int")
	parser.set_defaults(
		pip = NAO_IP,
		pport = 9559)

	(opts, args_) = parser.parse_args()
	pip = opts.pip
	pport = opts.pport

	# Nuevo broker para la construcción de nuevos módulos.
	# Debe ejecutarse en tanto el programa exista.
	myBroker = ALBroker("myBroker", "0.0.0.0", 0, pip, pport)

	global ballSpy
	ballSpy = ballSpyModule("ballSpy")

	try:
		while True:
			time.sleep(1)
	except KeyboardInterrupt:
		print 
		print "KeyboardInterrupt, shutting down..."
		myBroker.shutdown()
		sys.exit(0)		

if __name__ == "__main__":
	main()