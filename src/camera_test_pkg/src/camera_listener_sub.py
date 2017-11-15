#! /usr/bin/python

import cv2
import numpy as np
import sys
import argparse
import intera_interface

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

global img, once

def detect_circles(img):
    cv2.medianBlur(img, 5)
    cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
    circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,20,
                            param1=50,param2=30,minRadius=0,maxRadius=0)
    circles = np.uint16(np.around(circles))


def callback(data):
    global image, once

    if once:
        cv2.imshow("sample", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        sys.exit()

    once = True

    rospy.logdebug("Received image")

    try:
        image = bridge.imgmsg_to_cv2(data, "bgr8")
        print(type(image))
    except CvBridgeError as e:
        print(e)

    

    print("Image size: ({0}, {1})".format(image.shape[0], image.shape[1]))
    # subscriber.unregister(subscriber)


def main():
    # initialize subscriber to take one image #
    global once
    once = False
    rospy.init_node("circle_detection")
    img = rospy.Subscriber("/io/internal_camera/head_camera/image_rect_color", Image, callback)
    # cv2.imshow("sample", bridge.imgmsg_to_cv2(take_image, "bgr8"))

    try:
        rospy.spin()
        once = True
        # cv2.imshow("sample", img)
    except KeyboardInterrupt:
        print("Shutting down")

# `   cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
#     cv2.imshow("sample", cv_image)
    # cv2.destroyAllWindows()

    

if __name__ == "__main__":
    sys.exit(main())
