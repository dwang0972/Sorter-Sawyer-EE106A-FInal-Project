#! /usr/bin/python

import copy
import numpy as np
import struct
import sys

import rospy

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

from baxter_pnp.srv import *
from object_detection.srv import *


class Homography:
    def __init__(self):
        rospy.init_node("homography", log_level=rospy.DEBUG)

        # Circle Service
        rospy.wait_for_service('circle_detection', 5)
        self.object_pose_srv = rospy.ServiceProxy('circle_detection', ObjectPoses)

        # Pick and place Service
        rospy.wait_for_service('pick_and_place', 5)
        self.pnp_srv = rospy.ServiceProxy('pick_and_place', PickAndPlace)


    def run(self):
        rate = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            rospy.wait_for_service('circle_detection', 5.0)
            object_pose_request = ObjectPosesRequest(frame='/base')
            object_pose_response = self.object_pose_srv(object_pose_request)
            poses = object_pose_response.poses

            rospy.logdebug("Circle detection response: {0}".format(object_pose_response))
            if len(poses) > 0 and poses[0].pose.position.x != 0:
                pose = poses[0]

                rospy.logdebug("Object detected: \n{0}".format(pose))

                orientation = Quaternion(
                    x=0.8,
                    y=-0.5,
                    z=0,
                    w=0
                )

                pose.pose.orientation = orientation

                pick = copy.deepcopy(pose)
                place = copy.deepcopy(pose)
                place.pose.position.y += 0.5

                pnp_request = PickAndPlaceRequest()
                pnp_request.pick = pick
                pnp_request.place = place

                self.pnp_srv(pnp_request)

            else:
                rospy.logdebug("No object found")
                
            rate.sleep()


def main():
    homography = Homography()
    homography.run()


if __name__ == "__main__":
    sys.exit(main())