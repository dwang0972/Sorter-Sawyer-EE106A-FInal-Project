#! /usr/bin/python

import cv2
import numpy as np
import sys

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def detect_circles(img):
	cv2.medianBlur(img, 5)
	img = cv2.cvtColor(img,cv2.COLOR_BGRA2GRAY)
	circles = cv2.HoughCircles(img, cv2.cv.CV_HOUGH_GRADIENT,1,500,
                            param1=50,param2=30,minRadius=10,maxRadius=0)
	circles = np.uint16(np.around(circles))

	for i in circles[0, :]:
		cv2.circle(img, (i[0], i[1]), i[2], (0,255,0), 2)
		cv2.circle(img, (i[0], i[1]), 2, (0,0,255), 3)
	cv2.imshow('circles', img)
	cv2.waitKey()
	return circles


def callback(data):
	print("Received image")
	try:
		cv_image = bridge.imgmsg_to_cv2(data, "bgra8")
	except CvBridgeError as e:
		print(e)

	#print("Image size: ({0}, {1})".format(cv_image.shape[0], cv_image.shape[1]))
	circles = detect_circles(cv_image)
	#print(circles)


def main():
	rospy.init_node("circle")
	rospy.Subscriber("/io/internal_camera/head_camera/image_raw", Image, callback)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()


if __name__ == "__main__":
	sys.exit(main())