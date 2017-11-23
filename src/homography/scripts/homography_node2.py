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

from sensor_msgs.msg import (
    JointState
)

from ar_marker.srv import *
from baxter_pnp.srv import *
from object_detection.srv import *




orientation = Quaternion(
                    x=0.8,
                    y=-0.5,
                    z=0,
                    w=0
                )





class Homography:
    def __init__(self):
        rospy.init_node("homography", log_level=rospy.DEBUG)
        self.rate = rospy.Rate(0.5)

        # AR Tag Service
        rospy.wait_for_service('marker_pose', 5)
        self.marker_pose_srv = rospy.ServiceProxy('marker_pose', ArMarkerPose)

        # Circle Service
        rospy.wait_for_service('circle_detection', 5)
        self.object_pose_srv = rospy.ServiceProxy('circle_detection', ObjectPoses)

        # Pick and place Service
        rospy.wait_for_service('pick_and_place', 20)
        self.pnp_srv = rospy.ServiceProxy('pick_and_place', PickAndPlace)

        # Movement Service
        rospy.wait_for_service('move_to_position', 5)
        self.position_srv = rospy.ServiceProxy('move_to_position', PositionMovement)

        # Joint Service
        rospy.wait_for_service('move_to_joint', 5)
        self.joint_srv = rospy.ServiceProxy('move_to_joint', JointMovement)

    def move_to_start(self):
        starting_joint_angles = {'right_j0': -2.7097744140625,
                             'right_j1': 0.120755859375,
                             'right_j2': -1.40407421875,
                             'right_j3': -1.0929814453125,
                             'right_j4': 1.3393115234375,
                             'right_j5': 1.6684326171875,
                             'right_j6': 4.5927880859375}

        starting_pose = Pose(
            position=Point(
                x=-0.0693072626854,
                y=-0.787608264825,
                z=0.320786757036
            ),
            orientation=Quaternion(
                x=-0.964505139643,
                y=0.26332012058,
                z=0.0195351258605,
                w=0.00327544766135
            )
        )

        starting_joint_angles_names = starting_joint_angles.keys()
        starting_joint_angles_positions = starting_joint_angles.values()
        joint_state = JointState(header=Header(frame_id='/base', stamp=rospy.Time(0)), name=starting_joint_angles_names, position=starting_joint_angles_positions)

        #request = JointMovementRequest(joint_state=joint_state)

        request = PositionMovementRequest(pose=starting_pose)

        rospy.wait_for_service('move_to_position', 5)
        return self.position_srv(request)


    def execute_move_to_start(self):
        rospy.logdebug("Moving to starting point...")
        while not rospy.is_shutdown():
            response = self.move_to_start()
            if response.status:
                rospy.loginfo("Robot in starting point")
                break
            self.rate.sleep()

    def execute_move_above_ar_tag(self):
        rospy.logdebug("Looking for AR tag...")
        while not rospy.is_shutdown():
            rospy.wait_for_service('marker_pose', 5.0)
            request = ArMarkerPoseRequest(frame='/base', marker_type=13)
            marker_pose_response = self.marker_pose_srv(request)
            marker_pose = marker_pose_response.pose
            if marker_pose is None:
                rospy.logdebug("No AR tag found")
                continue

            marker_pose.pose.position.z += 0.4
            marker_pose.pose.orientation = orientation

            rospy.wait_for_service('move_to_position', 5.0)
            move_request = PositionMovementRequest(pose=marker_pose.pose)
            move_response = self.position_srv(move_request)
            if move_response.status:
                rospy.loginfo("Robot above AR tag")
                break

            self.rate.sleep()

    def execute_pick_and_place(self):
        while not rospy.is_shutdown():
            rospy.wait_for_service('circle_detection', 5.0)
            object_pose_request = ObjectPosesRequest(frame='/base')
            object_pose_response = self.object_pose_srv(object_pose_request)
            poses = object_pose_response.poses

            rospy.logdebug("Circle detection response: {0}".format(object_pose_response))
            if len(poses) > 0 and poses[0].pose.position.x != 0:
                pose = poses[0]

                rospy.logdebug("Object detected: \n{0}".format(pose))

                pose.pose.orientation = orientation

                pick = copy.deepcopy(pose)
                place = copy.deepcopy(pose)
                place.pose.position.y += 0.5

                pnp_request = PickAndPlaceRequest()
                pnp_request.pick = pick
                pnp_request.place = place
                rospy.wait_for_service('pick_and_place', 5.0)
                self.pnp_srv(pnp_request)

            else:
                rospy.logdebug("No object found")
                
            self.rate.sleep()


    def run(self):
        self.execute_move_to_start()
        self.execute_move_above_ar_tag()
        self.execute_pick_and_place()

        

        


def main():
    homography = Homography()
    homography.run()


if __name__ == "__main__":
    sys.exit(main())