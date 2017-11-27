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

        self.markers = {}
        self.load_markers()

        # AR Tag Service
        rospy.wait_for_service('marker_pose', 5)
        self.marker_pose_srv = rospy.ServiceProxy('marker_pose', ArMarkerPose)

        # Circle Service
        rospy.wait_for_service('circle_detection', 5)
        self.object_pose_srv = rospy.ServiceProxy('circle_detection', ObjectPoses)

        # Pick and place Service
        rospy.wait_for_service('pick_and_place', 5)
        self.pnp_srv = rospy.ServiceProxy('pick_and_place', PickAndPlace)

        # Movement Service
        rospy.wait_for_service('move_to_position', 5)
        self.position_srv = rospy.ServiceProxy('move_to_position', PositionMovement)

        # Joint Service
        rospy.wait_for_service('move_to_joint', 5)
        self.joint_srv = rospy.ServiceProxy('move_to_joint', JointMovement)

        # Head Service
        rospy.wait_for_service('move_head', 5)
        self.head_srv = rospy.ServiceProxy("move_head", HeadMovement)

        # Throw service
        rospy.wait_for_service('throw', 5)
        self.throw_srv = rospy.ServiceProxy('throw', Throw)


    def load_markers(self):
        for color in ["red", "yellow", "green", "blue"]
            marker = rospy.get_param("~marker_" + color, None)
            if marker is not None:
                self.markers[color] = marker


    def move_to_start(self):
        starting_joint_angles = {'right_j0': 1.5087646484375,
                             'right_j1': -0.11942578125,
                             'right_j2': -3.0390751953125,
                             'right_j3': 0.047236328125,
                             'right_j4': -2.9739267578125,
                             'right_j5': 0.0444990234375,
                            'right_j6': -0.005763671875}

        starting_pose = Pose(
            position=Point(
                x=-0.000735619,
                y=0.855682,
                z=0.205539
            ),
            orientation=Quaternion(
                x=0.997216,
                y=-0.0447665,
                z=-0.0411083,
                w=0.0432003
            )
        )

        starting_joint_angles_names = starting_joint_angles.keys()
        starting_joint_angles_positions = starting_joint_angles.values()
        joint_state = JointState(header=Header(frame_id='/base', stamp=rospy.Time(0)), name=starting_joint_angles_names, position=starting_joint_angles_positions)

        request = JointMovementRequest(joint_state=joint_state)
        rospy.wait_for_service('move_to_joint', 5)
        return self.joint_srv(request)

        #request = PositionMovementRequest(pose=starting_pose)
        #rospy.wait_for_service("move_to_position", 5)
        #return self.position_srv(request)


    def execute_move_to_start(self):
        rospy.logdebug("Moving to starting point...")

        while not rospy.is_shutdown():
            response = self.move_to_start()
            if response.status:
                rospy.loginfo("Robot in starting point")
                rospy.sleep(1)
                break
            self.rate.sleep()

        request = HeadMovementRequest(angle=0)
        rospy.wait_for_service('move_head')
        self.head_srv(request)

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

            rospy.logdebug("Found AR tag at: \n{0}".format(marker_pose.pose))
            marker_pose.pose.position.z += 0.4
            marker_pose.pose.orientation = orientation

            rospy.wait_for_service('move_to_position', 5.0)
            move_request = PositionMovementRequest(pose=marker_pose.pose)
            move_response = self.position_srv(move_request)
            if move_response.status:
                rospy.loginfo("Robot above AR tag")
                rospy.sleep(1)
                break

            self.rate.sleep()

    def execute_pick_and_place(self):
        while not rospy.is_shutdown():
            rospy.wait_for_service('circle_detection', 5.0)
            object_pose_request = ObjectPosesRequest(frame='/base')
            object_pose_response = self.object_pose_srv(object_pose_request)
            pick_poses = object_pose_response.poses
            pick_colors = object_pose_response.colors

            rospy.logdebug("Circle detection response: {0}".format(object_pose_response))
            if len(pick_poses) > 0 and pick_poses[0].pose.position.x != 0:

                pick_pose = pick_poses[0]
                pick_color = pick_colors[0]

                pick_pose.pose.orientation = orientation
                rospy.logdebug("Pick detected: \n{0}".format(pick_pose))

                rospy.wait_for_service('marker_pose', 5.0)
                marker_pose_request = ArMarkerPoseRequest(frame='/base', marker_type=self.markers[pick_color])
                marker_pose_response = self.marker_pose_srv(marker_pose_request)
                place_pose = marker_pose_response.pose

                if place_pose is not None and place_pose.pose.position.x != 0:
                    place_pose.pose.orientation = orientation
                    rospy.logdebug("Place detected: \n{0}".format(place_pose))


                    pnp_request = PickAndPlaceRequest()
                    pnp_request.pick = pick_pose
                    pnp_request.place = place_pose
                    rospy.wait_for_service('pick_and_place', 5.0)
                    self.pnp_srv(pnp_request)
                    rospy.sleep(1)
                    break
                
                else:
                    rospy.logdebug("No place detected")

            else:
                rospy.logdebug("No object detected")
                
            self.rate.sleep()


    def run(self):
        while not rospy.is_shutdown():
            self.execute_move_to_start()
            #self.execute_move_above_ar_tag()
            self.execute_pick_and_place()

        

        


def main():
    homography = Homography()
    homography.run()


if __name__ == "__main__":
    sys.exit(main())