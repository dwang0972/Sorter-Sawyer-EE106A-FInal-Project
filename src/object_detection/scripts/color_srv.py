#! /usr/bin/python

import copy
import cv2
import math
import numpy as np
import sys

import rospy
from cv_bridge import CvBridge, CvBridgeError

from ar_marker.srv import *
from object_detection.srv import *

import tf
import tf.transformations

from image_geometry import PinholeCameraModel

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

from sensor_msgs.msg import (
    Image,
    CameraInfo
)

img_display = None

class ColorDetectionService:
    def __init__(self):
        self.bridge = CvBridge()
        self.listener = tf.TransformListener()
        self.detected_objects = []

        rospy.init_node("color_detection", log_level=rospy.DEBUG)

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

        rospy.Service("color_detection", ObjectPoses, self.get_color_positions)


    def update_cam_info(self, message):
        self.pinhole_camera.fromCameraInfo(message)


    def color_filter(self, img, lower, upper):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_range = np.array(lower, dtype=np.uint8)
        upper_range = np.array(upper, dtype=np.uint8)

        mask = cv2.inRange(hsv, lower_range, upper_range)
        return cv2.bitwise_and(hsv, hsv, mask=mask), mask

    def find_contours(self, img, mask):
        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        
        contours = []

        # only proceed if at least one contour was found
        if len(cnts) == 0:
            return contours
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
        for c in cnts:
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            # only proceed if the radius meets a minimum size
            if radius < 4 or radius > 15:
                continue

            #rospy.logdebug("Min enclosing circle: {0}, {1}, {2}".format(x, y, radius))

            M = cv2.moments(c)

            if M["m00"] != 0:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                #rospy.logdebug("Moment: {0}, {1}".format(center[0], center[1]))

                too_close = False
                for cnt in contours:
                    if math.sqrt(math.pow(cnt[0]-center[0], 2) + math.pow(cnt[1]-center[1], 2)) < 20:
                        too_close = True
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        
                if not too_close:
                    contours.append((center[0], center[1], int(radius)))

        return contours

    def detect_color(self, img, lower, upper):
        copy_img = copy.copy(img)
        new_img, new_mask = self.color_filter(copy_img, lower, upper)
        return self.find_contours(new_img, new_mask)


    def detect_colors(self, img):
        #cv2.medianBlur(img, 5)

        detected_objects = {}

        yellowObjects = self.detect_color(img, [20, 70, 90], [60, 110, 140])
        detected_objects["yellow"] = yellowObjects

        #redObjects = self.detect_color(img, [0, 100, 80], [30, 150, 120])
        #detected_objects["red"] = redObjects

        blueObjects = self.detect_color(img, [100, 110, 40], [110, 150, 60])
        detected_objects["blue"] = blueObjects

        #greenObjects = self.detect_color(img, [70, 100, 45], [90, 115, 60])
        #detected_objects["green"] = greenObjects
        
        for color in detected_objects:
            for obj in detected_objects[color]:
                obj = np.uint16(np.around(obj))
                if color == "yellow":
                    bgr = (0, 255, 255)
                elif color == "red":
                    bgr = (0, 0, 255)
                elif color == "blue":
                    bgr = (255, 0, 0)
                elif color == "green":
                    bgr = (0, 255, 0)
                cv2.circle(img, (obj[0], obj[1]), obj[2], bgr, 2)
                cv2.circle(img, (obj[0], obj[1]), 2, bgr, 3)

        global img_display
        img_display = img
        cv2.imshow('colors', img)
        key = cv2.waitKey(1) & 0xFF

        #rospy.logdebug("Detected objects: {0}".format(detected_objects))

        return detected_objects

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgra8")
        except CvBridgeError as e:
            rospy.logerr(e)

        self.detected_objects = self.detect_colors(cv_image)
        #print(self.detected_objects)

    def get_color_positions(self, request):
        response = ObjectPosesResponse()

        if self.detected_objects is None:
            return response

        if len(self.detected_objects) == 0:
            rospy.logdebug("No object detected")
            return response

        rospy.wait_for_service('marker_pose', 5.0)
        marker_pose_request = ArMarkerPoseRequest(frame=request.frame, marker_type=self.marker_type)
        marker_pose_response = self.marker_pose_srv(marker_pose_request)
        marker_pose = marker_pose_response.pose
        if marker_pose is None or marker_pose.pose.position.x == 0:
            rospy.logdebug("Couldn't find marker frame")
            return response

        colors = self.detected_objects.keys()
        objects = self.detected_objects.values()

        for c, objs in zip(colors, objects):
            for o in objs:
                ray = self.pinhole_camera.projectPixelTo3dRay((o[0], o[1]))
                point = self.ray_to_3dpoint(ray, marker_pose, request.frame)
                pose = PoseStamped(header=Header(frame_id=self.frame, stamp=rospy.Time(0)), pose=Pose(position=point))

                self.listener.waitForTransform(request.frame, self.frame, rospy.Time(0), rospy.Duration(3.0))
                transformed_pose = self.listener.transformPose(request.frame, pose)

                #rospy.logdebug("Point in base frame: {0}".format(transformed_pose.pose.position))
                response.colors.append(c)
                response.poses.append(transformed_pose)        

        response

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


    def run(self):
        rospy.spin()

def main():
    color_srv = ColorDetectionService()
    color_srv.run()


if __name__ == "__main__":
    sys.exit(main())