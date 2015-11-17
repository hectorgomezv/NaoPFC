# -*- encoding: UTF-8 -*-
import sys
import motion
import almath
from naoqi import ALProxy

def stiffness_on(motionProxy):
	p_names = "Body"
	p_stiffness_list = 1.0
	p_time_lists = 1.0
	motionProxy.stiffnessInterpolation(p_names, p_stiffness_list, p_time_lists)
	#print motionProxy.getSummary()


def main(robotIP):
	
	try: 
		motionProxy = ALProxy("ALMotion", robotIP, 9559)
	except Exception, e:
		print "Couldn't create proxy to ALMotion.\nError was:", e

	try:
		postureProxy = ALProxy("ALRobotPosture", robotIP, 9559)
	except Exception, e:
		print "Couldn't create proxy to ALRobotPosture.\nError was:", e

	stiffness_on(motionProxy)
	postureProxy.goToPosture("StandInit", 0.5)

	# Parámetros comunes
	time_coef = 0.5;
	times = [time_coef, time_coef*2, time_coef*3, time_coef*4]
	space = motion.FRAME_ROBOT
	isAbsolute = False

	# Paso adelante con pierna izquierda
	legName  = ["LLeg"]
	footSteps = [[0.08, 0.09, 0.0]]
	timeList = [0.6]
	clearExisting = False
	motionProxy.setFootSteps(legName, footSteps, timeList, clearExisting)

	# Inclinación de torso para equilibrarse
	effector = "Torso"
	initPos = initPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	pos1 = [0.005, 0.0, 0.0, 0.0, 0.0, 0.0]
	pos2 = [0.01, 0.03, -0.01, 0.0, 0.0, 0.0]
	pos3 = [0.015, 0.04, -0.015, 0.0, 0.0, 0.0]
	pos4 = [0.02, 0.05, -0.02, 0.0, 0.0, 0.0]
	path = [pos1, pos2, pos3, pos4]
	axisMask = almath.AXIS_MASK_ALL
	motionProxy.positionInterpolation(effector, space, path, axisMask, times, isAbsolute)

	# Adelante brazo izquierdo para equilibrarse
	effector = "LArm"
	targetPos = [0.2, 0.025, 0.2, 0.0, 0.0, 0.0]
	axisMask = almath.AXIS_MASK_VEL
	motionProxy.positionInterpolation(effector, space, targetPos, axisMask, 1.0, isAbsolute)	

	# Atrás brazo derecho para equilibrarse
	effector = "RArm"
	targetPos = [-0.05, -0.025, 0.01, 0.0, 0.0, 0.0]
	axisMask = almath.AXIS_MASK_VEL
	motionProxy.positionInterpolation(effector, space, targetPos, axisMask, 0.5, isAbsolute)

	# Golpeo con pierna derecha
	legName = ["RLeg"]
	footSteps = [[0.08, 0.09, 0.0]]
	clearExisting = False
	timeList = [0.6]
	motionProxy.setFootSteps(legName, footSteps, timeList, clearExisting)	

	# Volvemos a posición de partida.
	postureProxy.goToPosture("StandInit", 0.5)

if __name__ == "__main__":
	robotIP = "127.0.0.1"
	if len(sys.argv) <= 1:
		print "Usage python foot_move.py robotIP (optional default: 127.0.0.1)"
	else:
		robotIP = sys.argv[1]
	main(robotIP)		