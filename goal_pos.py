#!/usr/bin/env python
# -*- encoding: UTF-8 -*-
import init_robot
import camera
import rospy
import sys
import time
from naoqi import ALProxy
from geometry_msgs.msg import Point

def launcher(motionProxy, postureProxy, markProxy):
	init_robot.stiffness_on(motionProxy)
	init_robot.init_pos(postureProxy)
	videoDeviceProxy = camera.init_camera()	
	camera.up_camera(videoDeviceProxy)
	period = 500 # ms
	markProxy.subscribe('goal_pos', period, 0.0)
	
def publish_goal(memProxy, pub):
	# https://community.aldebaran.com/doc/1-14/dev/python/examples/vision/landmark.html
	goal = Point()
	val = memProxy.getData('LandmarkDetected')
	if (val and isinstance(val, list) and len(val) == 2):
		timeStamp = val[0]
		markInfoArray = val[1]
		try:
			# Browse the markInfoArray to get info on each detected mark.
			for markInfo in markInfoArray:
			# First Field = Shape info.
			markShapeInfo = markInfo[0]
			# Second Field = Extra info (ie, mark ID).
			markExtraInfo = markInfo[1]
			# rospy.loginfo( Mark information.
			rospy.loginfo('mark  ID: %d' % (markExtraInfo[0]))
			rospy.loginfo('alpha %.3f - beta %.3f' % (markShapeInfo[1], markShapeInfo[2]))
			rospy.loginfo('width %.3f - height %.3f' % (markShapeInfo[3], markShapeInfo[4]))
		except Exception, e:
			rospy.loginfo('Naomarks detected, but it seems getData is invalid. ALValue =')
			rospy.loginfo(val)
			rospy.loginfo('Error msg %s' % (str(e)))
	else:
		rospy.loginfo( 'Error with getData. ALValue = %s' % (str(val)))

	# goal.x, goal.y, goal.z = val[0], val[1], val[2]
	# rospy.loginfo(goal)
	# pub.publish(goal)

def main():
	rospy.init_node('goal_pos')
	motionProxy = init_robot.start_motion()
	postureProxy = init_robot.start_posture()
	markProxy = init_robot.start_mark_detection()
	memProxy = init_robot.start_memory()
	launcher(motionProxy, postureProxy, markProxy)

	pub = rospy.Publisher('goal', Point, queue_size=1)
	rate = rospy.Rate(5) #5Hz

	while not rospy.is_shutdown():
		publish_goal(memProxy, pub)
		rate.sleep()

	redgoalTrackerProxy.stopTracker()
	postureProxy.goToPosture('StandInit', 1.0)	

if __name__ == '__main__':
	main()