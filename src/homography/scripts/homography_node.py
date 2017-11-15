#! /usr/bin/python

import numpy as np
import sys

import rospy

from ar_track_alvar_msgs.msg import AlvarMarkers
from homography.srv import *

import tf.transformations as tftr

def main():
	rospy.init_node("homography")

	rospy.wait_for_service('last_marker')

	last_image_service = rospy.ServiceProxy('last_marker', ArLastMarker)
	
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		markers = last_image_service().marker.markers
		if len(markers) > 0:
			orientation = markers[0].pose.pose.orientation
			q = tftr.quaternion_about_axis(orientation.w, (orientation.x, orientation.y, orientation.z))
			print("Position: {0} \tOrientation: {1}".format(markers[0].pose.pose.position, q))
		rate.sleep()


if __name__ == "__main__":
	sys.exit(main())