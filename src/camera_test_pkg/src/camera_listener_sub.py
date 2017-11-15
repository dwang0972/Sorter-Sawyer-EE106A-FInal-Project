#! /usr/bin/python

import cv2
import cv2.cv as cv
import numpy as np
import sys
import argparse
import intera_interface

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

global img, once

def detect_circles(img_path):
    img = cv2.imread(img_path, 0)
    cv2.medianBlur(img, 5)
    cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
    circles = cv2.HoughCircles(img,cv.CV_HOUGH_GRADIENT,1,10,
                            param1=50,param2=30,minRadius=20,maxRadius=40)
    try: 
        circles = np.uint16(np.around(circles))
    except AttributeError as e:
        print(e)

    for i in circles[0,:]:
        # draw the outer circle
        cv2.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center of the circle
        cv2.circle(img,(i[0],i[1]),2,(0,0,255),3)
        print("Center: " + str(i[0]) +"," +str(i[1]))
    cv2.imshow("sample with circles", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()



def callback(data):
    global image, once

    if once:
        cv2.imwrite('sample.jpg', image)
        detect_circles('sample.jpg')
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
    img = None
    img = rospy.Subscriber("/io/internal_camera/head_camera/image_rect_color", Image, callback, img)
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
