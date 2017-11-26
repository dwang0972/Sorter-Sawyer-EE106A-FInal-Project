#! /usr/bin/python

import cv2
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


class CircleDetectionService:
    def __init__(self):
        self.bridge = CvBridge()
        self.listener = tf.TransformListener()
        self.circles = []

        rospy.init_node("circle_detection", log_level=rospy.DEBUG)

        self.marker_type = int( rospy.get_param("marker_type", "9") )
        
        self.frame = rospy.get_param("~image_frame")
        self.image_topic = rospy.get_param("~image_topic")
        rospy.Subscriber(self.image_topic, Image, self.callback)

        self.pinhole_camera = PinholeCameraModel()
        self.cam_info_topic = rospy.get_param("~cam_info_topic")
        rospy.Subscriber(self.cam_info_topic, CameraInfo, self.update_cam_info)

        # Marker service
        rospy.wait_for_service('marker_pose', 5)
        self.marker_pose_srv = rospy.ServiceProxy('marker_pose', ArMarkerPose)

        rospy.Service("circle_detection", ObjectPoses, self.getCirclePositions)


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
        
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            #rospy.logdebug("Min enclosing circle: {0}, {1}, {2}".format(x, y, radius))

            M = cv2.moments(c)

            if M["m00"] != 0:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                #rospy.logdebug("Moment: {0}, {1}".format(center[0], center[1]))
            
                # only proceed if the radius meets a minimum size
                if radius > 5:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    return np.array([[[center[0], center[1], int(radius)]]])

    def find_circles(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
        return cv2.HoughCircles(gray, cv2.cv.CV_HOUGH_GRADIENT,1,100,
                                param1=200,param2=30,minRadius=0,maxRadius=0)


    def detect_circles(self, img):
        #cv2.medianBlur(img, 5)

        img, mask = self.color_filter(img, [20, 80, 100], [80, 200, 255])
        circles = self.find_contours(img, mask)

        #circles = self.find_circles(img)
        
        '''
        rospy.logdebug("Circles: {0}".format(circles))
        if circles is not None:
            circles_to_draw = np.uint16(np.around(circles))

            for i in circles_to_draw[0, :]:
                cv2.circle(img, (i[0], i[1]), i[2], (0,255,0), 2)
                cv2.circle(img, (i[0], i[1]), 2, (0,0,255), 3)

        cv2.imshow('circles', img)
        key = cv2.waitKey(1) & 0xFF
        '''

        return circles


    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgra8")
        except CvBridgeError as e:
            rospy.logerr(e)

        #print("Image size: ({0}, {1})".format(cv_image.shape[0], cv_image.shape[1]))
        self.circles = self.detect_circles(cv_image)
        #print(circles)

    def getCirclePositions(self, request):
        response = ObjectPosesResponse()

        if self.circles is None:
            return response

        if len(self.circles) == 0:
            return response

        rospy.wait_for_service('marker_pose', 5.0)
        marker_pose_request = ArMarkerPoseRequest(frame=request.frame, marker_type=self.marker_type)
        marker_pose_response = self.marker_pose_srv(marker_pose_request)
        marker_pose = marker_pose_response.pose
        if marker_pose is None or marker_pose.pose.position.x == 0:
            return response

        for c in self.circles[0, :]:
            ray = self.pinhole_camera.projectPixelTo3dRay((c[0], c[1]-c[2]))
            point = self.ray_to_3dpoint(ray, marker_pose, request.frame)
            pose = PoseStamped(header=Header(frame_id=self.frame, stamp=rospy.Time(0)), pose=Pose(position=point))

            self.listener.waitForTransform(request.frame, self.frame, rospy.Time(0), rospy.Duration(3.0))
            transformed_pose = self.listener.transformPose(request.frame, pose)
            #transformed_pose.pose.position.x += 0.05
            #transformed_pose.pose.position.y += 0.05

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

        t = (marker_pose.pose.position.z - p[2]) / np.dot(R[:3, :3], np.array(ray))[2]

        #rospy.logdebug("Parameter t: {0}".format(t))

        point.x = ray[0] * t
        point.y = ray[1] * t
        point.z = ray[2] * t

        #rospy.logdebug("Point in camera frame: {0}".format(point))

        return point


    def run(self):
        rospy.spin()


def main():
    circle_srv = CircleDetectionService()
    circle_srv.run()


if __name__ == "__main__":
    sys.exit(main())