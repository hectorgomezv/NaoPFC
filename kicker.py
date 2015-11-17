#!/usr/bin/env python
# -*- encoding: UTF-8 -*-
import init_robot
import rospy
import sys
import os
import time
import math
import motion
import almath
import pyqtgraph as pg
import numpy as np
from naoqi import ALProxy
from naoqi import ALBroker
from geometry_msgs.msg import Point
import csv
import basic_kick

TRAJECTORY_FRAMES = 15

def calculate_target_x(frames):
	""" Función que obtiene sucesivos mensajes de posición de la bola
		con respecto al eje X y calcula la media de dichos valores. 
	"""
	rospy.loginfo('Calculating kick target on X axis...')
	balls = []
	for i in range(frames):
		ball = rospy.wait_for_message('pos', Point)
		balls.append(ball.x)
	target_x = np.mean(balls)
	rospy.loginfo('TARGET_X = '+ str(target_x))
	return target_x

def calculate_target_y(frames):
	""" Función que obtiene sucesivos mensajes de posición de la bola
		con respecto al eje Y y calcula la media de dichos valores. 
	"""
	rospy.loginfo('Calculating kick target on Y axis...')
	balls = []
	for i in range(frames):
		ball = rospy.wait_for_message('pos', Point)
		balls.append(ball.y)
	target_y = np.mean(balls)
	rospy.loginfo('TARGET_Y = '+ str(target_y))
	return target_y

def prepare_kick(motionProxy):
	""" Función que establece una posición desde la cual el Nao está equilibrado
		y listo para ejercer el golpeo con pierna derecha.
	"""
	# Parámetros comunes a todos los movimientos.
	space = motion.FRAME_ROBOT
	axisMask = almath.AXIS_MASK_ALL 
	isAbsolute = False

	# Bajamos el torso.
	effector = 'Torso'
	path = [0.0, -0.06, -0.03, 0.0, 0.0, 0.0]
	times = 1.2                   
	motionProxy.positionInterpolation(effector, space, path, \
		axisMask, times, isAbsolute)

	# Colocamos pierna izquierda.
	effector = 'LLeg'
	path = [0.03,  0.00,  0.00, 0.0, 0.0, 0.0]
	times = 1.2                   
	motionProxy.positionInterpolation(effector, space, path, \
		axisMask, times, isAbsolute)

	# Elevamos torso de nuevo.
	effector = 'Torso'
	path = [0.05, 0.06, 0.03, 0.0, 0.0, 0.0]
	times = 1.2
	motionProxy.positionInterpolation(effector, space, path, \
		axisMask, times, isAbsolute)

	time.sleep(2)

	# Movemos torso e inclinamos para mantener estabilidad en golpeo.
	effector = 'Torso'
	path = [0.0, 0.08, -0.01, 0.0, 0.0, 0.0]
	times = 1.5
	motionProxy.positionInterpolation(effector, space, path, \
		axisMask, times, isAbsolute)    	 	

def kick(motionProxy, target_x, target_y, deviation):
	""" Función que realiza el movimiento de golpeo. Se pasa como 
		parámetro la posición de la bola con respecto al torso del Nao.
	"""
	rospy.loginfo('Deviation: ' + str(deviation))
	rospy.loginfo('Kicking ball at final position (' + 
		str(target_x) +', ' + str(target_y+deviation) + ').')
	# Golpeamos con pierna derecha.
	space = motion.FRAME_ROBOT
	axisMask = almath.AXIS_MASK_ALL 
	isAbsolute = False
	effector = "RLeg"
	alpha = 999999
	while alpha > 30 or alpha < -30:
		alpha = float(raw_input('Enter angle deviation; range -30 to 30 (degrees): '))
	deviation = angle2y(alpha, target_x, target_y)
	path = [[0.00, -(target_y/2)+deviation, 0.06, 0.0, 0.0, 0.0],
			[target_x,  (target_y/2),  0.03, 0.0, 0.0, 0.0]]
	times = [0.3, 0.6]
	motionProxy.positionInterpolation(effector, space, path, \
		axisMask, times, isAbsolute) 

def get_trajectory():
	""" Función que devuelve la trayectoria de la bola en una tupla de dos listas.
		La primera lista contiene las posiciones de la bola con respecto al eje X, 
		y la segunda lista ídem con respecto al eje Y.
		Los datos se leen directamente del topic pos durante INTERVAL segundos.
	"""
	trajectory_x, trajectory_y = [], []
	while len(trajectory_x) < TRAJECTORY_FRAMES:
		ball = rospy.wait_for_message('pos', Point)
		trajectory_x.append(ball.x)
		trajectory_y.append(ball.y)
	return [trajectory_x, trajectory_y]

def print_trajectory(trajectory):
	""" Función que imprime un gráfico de la modificación de la posición de la bola
		durante el INTERVAL. Documentación en:
		http://www.pyqtgraph.org/documentation/plotting.html
	"""	
	x = np.asarray(trajectory[0])
	y = np.asarray(trajectory[1])
	plotWidget = pg.plot(title="Posición de la bola desde FRAME_ROBOT")
	for i in range(3):
		plotWidget.plot(x, y, pen=(i,3))
	raw_input('Press Enter to continue...')

def angle2y(alpha, target_x, target_y):
	""" Función que toma como parámetros un ángulo de desviación en el tiro,
		la distancia a la pelota, y la coordenada Y de la pelota con respecto a
		FRAME_ROBOT. Calcula la coordenada en el eje Y asociada mediante un 
		cálculo trigonométrico. 
	"""
	alpha = math.radians(alpha)
	deviation = target_x * math.tan(alpha) - abs(target_y)
	return deviation 

def y2angle(deviation, target_x, target_y):
	""" Función inversa a angle2y. 
		Toma como parámetros la coordenada de destino del golpeo, y las coordenadas
		de la pelota con respecto a FRAME_ROBOT. Calcula el ángulo asociado
		mediante un cálculo trigonométrico.
	"""
	alpha = math.atan((abs(target_y)+deviation)/target_x)
	return math.degrees(alpha)

def trajectory_to_csv(trajectory, alpha):
	""" Función que escribe la trayectoria del balón pasada como parámetro en el
		archivo trajectory.csv
		FORMATO: ángulo, frame1, frame2, frame3... framei
	"""
	rospy.loginfo('CSV saved to: ' + str(os.getcwd()))
	f = open('trajectory.csv', 'a')
	# for i in trajectory[0]:
	# 	f.write('%.3f' % i)
	# 	f.write(', ')
	# f.write('\n')
	f.write(str(alpha) + ', ')
	for i in trajectory[1]:
		f.write('%.3f' % i)
		f.write(', ')
	f.write('\n')
	f.close()	


def main():
	motionProxy = init_robot.start_motion()
	postureProxy = init_robot.start_posture()
	rospy.init_node('kicker', anonymous=True)
	if not rospy.has_param('frames'):
		rospy.set_param('frames', 10) #valor default
	frames = rospy.get_param('frames')

	target_x = calculate_target_x(frames)
	target_y = calculate_target_y(frames)

	prepare_kick(motionProxy)
	basic_kick.basic_kick(motionProxy)
	#kick(motionProxy, target_x, target_y, deviation)
	trajectory = get_trajectory()
	rospy.loginfo('Ball trajectory: ' + str(trajectory))
	trajectory_to_csv(trajectory, alpha)
	# print_trajectory(trajectory)
	init_robot.init_pos(postureProxy)

if __name__ == "__main__":
	main()