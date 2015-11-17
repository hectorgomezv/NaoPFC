# -*- encoding: UTF-8 -*-
from naoqi import ALProxy
from naoqi import ALModule
from naoqi import ALBroker
import time
import sys
import motion
import almath
from optparse import OptionParser

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
	
	try: 
		motionProxy = ALProxy("ALMotion", NAO_IP, 9559)
	except Exception, e:
		print "Couldn't create proxy to ALMotion.\nError was:", e

	try:
		postureProxy = ALProxy("ALRobotPosture", NAO_IP, 9559)
	except Exception, e:
		print "Couldn't create proxy to ALRobotPosture.\nError was:", e


	stiffness_on(motionProxy)
	postureProxy.goToPosture("StandInit", 1.0)

	# Bajamos cabeza para tener un plano de los pies del Nao
	motionProxy.setAngles("HeadPitch", 0.3, 0.1)
	# if redBallDetectionProxy.redBallDetected: 
	#	print "Bolaaaa\n"

if __name__ = "__main__":
	main()