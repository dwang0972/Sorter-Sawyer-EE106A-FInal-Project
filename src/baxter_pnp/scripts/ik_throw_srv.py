#!/usr/bin/env python

import rospy
import sys


import scipy.optimize as opt

class IkThrowService:
	def __init__(self):
		rospy.init_node("ik_throw_service", log_level=rospy.DEBUG)
		rospy.Service("ik_throw", IkThrow, self.callback)

		self.joints = {
			'right_j0': 
		}


	def callback(self):
		pass

	def equations(self, variables):
		theta, t = variables
		y_eq = 

def main():


if __name__ == "__main__":
	sys.exit(main())