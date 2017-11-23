#! /usr/bin/python

import cv2
import numpy as np
import sys

import rospy
from cv_bridge import CvBridge, CvBridgeError

from ar_marker.srv import *
from object_detection.srv import *

import tf

from image_geometry import PinholeCameraModel

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
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
        return cv2.bitwise_and(hsv, hsv, mask=mask)


    def detect_circles(self, img):
        #cv2.medianBlur(img, 5)

        #img = self.color_filter(img, [0, 20, 100], [50, 200, 255])

        gray = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
        circles = cv2.HoughCircles(gray, cv2.cv.CV_HOUGH_GRADIENT,1,100,
                                param1=200,param2=30,minRadius=0,maxRadius=0)

        #rospy.logdebug("Circles: {0}".format(circles))

        if circles is not None:
            circles_to_draw = np.uint16(np.around(circles))

            for i in circles_to_draw[0, :]:
                cv2.circle(img, (i[0], i[1]), i[2], (0,255,0), 2)
                cv2.circle(img, (i[0], i[1]), 2, (0,0,255), 3)

        cv2.imshow('circles', img)
        cv2.waitKey(100)

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
        marker_pose_request = ArMarkerPoseRequest(frame=self.frame, marker_type=self.marker_type)
        marker_pose_response = self.marker_pose_srv(marker_pose_request)
        marker_pose = marker_pose_response.pose
        if marker_pose is None:
            return response

        for c in self.circles[0, :]:
            ray = self.pinhole_camera.projectPixelTo3dRay((c[0], c[1]))
            point = self.ray_to_3dpoint(ray, marker_pose)
            pose = PoseStamped(header=Header(frame_id=self.frame, stamp=rospy.Time(0)), pose=Pose(position=point))

            self.listener.waitForTransform(request.frame, self.frame, rospy.Time(0), rospy.Duration(3.0))
            transformed_pose = self.listener.transformPose(request.frame, pose)
            
            response.poses.append(transformed_pose)        

        return response

    def ray_to_3dpoint(self, ray, marker_pose):
        point = Point()

        t = marker_pose.pose.position.z / ray[2]

        point.x = ray[0] * t
        point.y = ray[1] * t
        point.z = marker_pose.pose.position.z

        return point


    def run(self):
        rospy.spin()


def main():
    circle_srv = CircleDetectionService()
    circle_srv.run()


if __name__ == "__main__":
    sys.exit(main())