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
    cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
    circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,20,
                            param1=50,param2=30,minRadius=0,maxRadius=0)
    circles = np.uint16(np.around(circles))


def callback(data):
    rospy.logdebug("Received image")
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    print("Image size: ({0}, {1})".format(cv_image.shape[0], cv_image.shape[1]))
    return cv_image


def main():
    rospy.init_node("circle_detection")
    data = rospy.Subscriber("io/internal_cameras/head_camera/image_rect_color", Image, callback)
    print(data)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    sys.exit(main())
