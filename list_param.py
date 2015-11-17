#!/usr/bin/env python
# -*- encoding: UTF-8 -*-
import rospy
import sys
import time
import math

def main():
	rospy.set_param('list', [1.0, 1.0, 1.0])
	while not rospy.is_shutdown():
		l = rospy.get_param('list')
		for i in range(0, len(l)):
			l[i] = l[i] + 1.0
		rospy.set_param('list', l)
		lp = rospy.get_param('list')
		print(lp)
		time.sleep(1.0)

if __name__ == '__main__':
	main()