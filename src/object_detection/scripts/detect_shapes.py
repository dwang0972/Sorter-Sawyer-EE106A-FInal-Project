#!/usr/bin/env python

from pyimagesearch.shapedetector import ShapeDetector
import argparse
#import imutils
import cv2
import numpy as np
import rospy
import sys
from matplotlib import pyplot as plt

from ar_marker.srv import *
from object_detection.srv import *

import tf
import tf.transformations

from sensor_msgs.msg import (
    Image,
    CameraInfo
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
    Vector3,
    Vector3Stamped
)

from std_msgs.msg import (
    Header,
    Empty,
)

from image_geometry import PinholeCameraModel

from cv_bridge import CvBridge, CvBridgeError

class ShapeDetection:

    def __init__(self, shape, color):
        self.req_shape = shape
        self.req_color = color
        self.filtered_image = None
        self.results = None
        self.coords = None

        self.init_results()
        print(str(self.results))

        self.bridge = CvBridge()
        self.listener = tf.TransformListener()
        
        rospy.init_node("shape_detection", log_level=rospy.DEBUG)

        self.marker_type = int( rospy.get_param("marker_frame") )
        print("Using marker frame id: {0}".format(self.marker_type))

        self.frame = rospy.get_param("~image_frame")
        self.image_topic = rospy.get_param("~image_topic")
        rospy.Subscriber(self.image_topic, Image, self.callback)

        self.pinhole_camera = PinholeCameraModel()
        self.cam_info_topic = rospy.get_param("~cam_info_topic")
        rospy.Subscriber(self.cam_info_topic, CameraInfo, self.update_cam_info)

        # Marker service
        rospy.wait_for_service('marker_pose', 5)
        self.marker_pose_srv = rospy.ServiceProxy('marker_pose', ArMarkerPose)

        rospy.Service("shape_detection", ObjectPoses, self.get_shape_positions)

    def run(self):
        rospy.spin()

    def update_cam_info(self, message):
        self.pinhole_camera.fromCameraInfo(message)


    def color_filter(self, img, lower, upper):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_range = np.array(lower, dtype=np.uint8)
        upper_range = np.array(upper, dtype=np.uint8)

        mask = cv2.inRange(hsv, lower_range, upper_range)
        return cv2.bitwise_and(hsv, hsv, mask=mask)

    def init_results(self):
        self.results = {'rectangle': [],
                'circle': [],
                'square': []}

    def categorize(self, shape, contour):
        self.init_results()
        if shape not in self.results.keys():
            return
        self.results[shape].append(contour)
        return self.results

# construct the argument parse and parse the arguments
# ap = argparse.ArgumentParser()
# ap.add_argument("-i", "--image", required=True,
#   help="path to the input image")
#
# ap.add_argument("-s", "--shape", required=True,
#   help="type of shape [rectangle, circle, square]")
#
# ap.add_argument("-c", "--color", required=True,
#   help="color of object [red, green, blue, yellow]")
#
# args = vars(ap.parse_args())

# load the image and resize it to a smaller factor so that
# the shapes can be approximated better

    def filter_image(self):
        min_limit = [0,0,0]
        max_limit = [255,255,255]

        if color == "blue":
            min_limit = [103, 50, 180]
            # min_limit = [100, 0, 0]
            max_limit = [130, 255, 255]
            # max_limit = [255, 255, 255]
        elif color == "yellow":
            min_limit = [20, 0, 0]
            max_limit = [40, 200, 255]

        self.filtered_image = color_filter(image, min_limit, max_limit)
# cv2.imwrite("test.jpg", image)

    def callback(self, request):
        #resized = imutils.resize(request.image, width=300)
        #ratio = self.image.shape[0] / float(resized.shape[0])

        # convert the resized image to grayscale, blur it slightly,
        # and threshold it

        image = self.bridge.imgmsg_to_cv2(request, "bgra8")

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]

        # find contours in the thresholded image and initialize the
        # shape detector
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] #f imutils.is_cv2() else cnts[1]
        sd = ShapeDetector()

        # loop over the contours
        for c in cnts:
            # compute the center of the contour, then detect the name of the
            # shape using only the contour
            M = cv2.moments(c)
            if not M["m00"]:
                continue
            cX = int((M["m10"] / M["m00"]))# * ratio)
            cY = int((M["m01"] / M["m00"]))# * ratio)
            shape = sd.detect(c)

            # multiply the contour (x, y)-coordinates by the resize ratio,
            # then draw the contours and the name of the shape on the image
            c = c.astype("float")
            #c *= ratio
            c = c.astype("int")
            self.results = self.categorize(shape, c)

            # cv2.drawContours(image, [c], -1, (255, 255, 255), 20)
            # cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
            #   10, (255, 255, 255), 2)


        if len(self.results[shape]) == 0:
            print("No requested shape found")
            sys.exit()

        cx = 0
        cy = 0

        if self.req_shape == "rectangle" or self.req_shape == "square":
            if len(results[self.req_shape]) != 0:
                contour = results[self.req_shape][0] # set contour
                for x in contour:
                    cx += x[0][0]
                    cy += x[0][1]
                cx = int(cx / len(contour))
                cy = int(cy/len(contour))
                cv2.circle(image, (cx, cy), 10, (255, 255, 255), -1)


        cv2.imshow('shapes', image)
        cv2.waitKey(1)

        if self.req_shape == "circle":
            pass

        self.coords = (cx, cy)

    def get_shape_positions(self, request):
        response = ObjectPosesResponse()

        if self.coords is None:
            return response

        if len(self.coords) == 0:
            rospy.logdebug("No object detected")
            return response

        rospy.wait_for_service('marker_pose', 5.0)
        marker_pose_request = ArMarkerPoseRequest(frame=request.frame, marker_type=self.marker_type)
        marker_pose_response = self.marker_pose_srv(marker_pose_request)
        marker_pose = marker_pose_response.pose
        if marker_pose is None or marker_pose.pose.position.x == 0:
            rospy.logdebug("Couldn't find marker frame")
            return response

        o = self.coords
        ray = self.pinhole_camera.projectPixelTo3dRay((o[0], o[1]))
        point = self.ray_to_3dpoint(ray, marker_pose, request.frame)
        pose = PoseStamped(header=Header(frame_id=self.frame, stamp=rospy.Time(0)), pose=Pose(position=point))

        self.listener.waitForTransform(request.frame, self.frame, rospy.Time(0), rospy.Duration(3.0))
        transformed_pose = self.listener.transformPose(request.frame, pose)

        #rospy.logdebug("Point in base frame: {0}".format(transformed_pose.pose.position))
        response.poses.append(transformed_pose)

        return response

    def ray_to_3dpoint(self, ray, marker_pose, base_frame):
        point = Point()

        #rospy.logdebug("Ray: {0}".format(ray))

        self.listener.waitForTransform(base_frame, self.frame, rospy.Time(0), rospy.Duration(3.0))
        p, q = self.listener.lookupTransform(base_frame, self.frame, rospy.Time(0))

        #rospy.logdebug("Translation: {0}".format(p))

        R = tf.transformations.quaternion_matrix(q)

        #rospy.logdebug("Rotation: {0}".format(R))

        #rospy.logdebug("AR tag: {0}".format(marker_pose.pose.position))

        rray = Vector3Stamped(header=Header(frame_id=self.frame, stamp=rospy.Time(0)), vector=Vector3(x=ray[0], y=ray[1], z=ray[2]))
        tray = self.listener.transformVector3(base_frame, rray)

        #rospy.logdebug("True transformed ray: {0}".format(tray.vector))

        #rospy.logdebug("Transformed ray: {0}".format(np.dot(R[:3, :3], np.array(ray))))

        t = (marker_pose.pose.position.z + 0.07 - p[2]) / np.dot(R[:3, :3], np.array(ray))[2]

        #rospy.logdebug("Parameter t: {0}".format(t))

        point.x = ray[0] * t
        point.y = ray[1] * t
        point.z = ray[2] * t

        #rospy.logdebug("Point in camera frame: {0}".format(point))

        return point


if __name__ == "__main__":
    sd = ShapeDetection(None, None)
    sd.run()