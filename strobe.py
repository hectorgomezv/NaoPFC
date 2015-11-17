#!/usr/bin/env python
# -*- encoding: UTF-8 -*-
import rospy
import time
import std_msgs

def main():
	while not rospy.is_shutdown():
		rospy.set_param('strobe', False)
		time.sleep(5.0)
		rospy.set_param('strobe', True)
		time.sleep(5.0)

if __name__ == '__main__':
	main()