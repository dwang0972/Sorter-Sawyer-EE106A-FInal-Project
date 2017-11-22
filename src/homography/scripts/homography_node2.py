#! /usr/bin/python

import copy
import numpy as np
import struct
import sys

import rospy

from ar_track_alvar_msgs.msg import AlvarMarkers
from homography.srv import *
from baxter_pnp.srv import *

import tf
import tf.transformations as tftr

import intera_interface

from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import moveit_msgs
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
import moveit_commander

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

from sensor_msgs.msg import JointState


class Homography:
    def __init__(self, limb_name):
        rospy.init_node("homography", log_level=rospy.DEBUG)

        self.limb_name = limb_name
        self.limb = intera_interface.Limb(limb_name)

        # IK Service
        self.ik_srv_name = "ExternalTools/" + limb_name + "/PositionKinematicsNode/IKService"
        rospy.wait_for_service(self.ik_srv_name, 5)
        self.ik_srv = rospy.ServiceProxy(self.ik_srv_name, SolvePositionIK)

        # Marker Service
        rospy.wait_for_service('marker_pose', 5)
        self.marker_pose_srv = rospy.ServiceProxy('marker_pose', ArMarkerPose)

        # Pick and place Service
        rospy.wait_for_service('pick_and_place', 5)
        self.pnp_srv = rospy.ServiceProxy('pick_and_place', PickAndPlace)


    def setup(self):
        # Enable the robot
        _rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        _init_state = _rs.state().enabled
        rospy.logdebug("Enabling robot... ")
        _rs.enable()

        if self.limb_name == "right":
            starting_joint_angles2 = {'right_j0': -2.7097744140625,
                                 'right_j1': 0.120755859375,
                                 'right_j2': -1.40407421875,
                                 'right_j3': -1.0929814453125,
                                 'right_j4': 1.3393115234375,
                                 'right_j5': 1.6684326171875,
                                 'right_j6': 4.5927880859375}

        else:
            rospy.logerr("Unknown limb: {0}".format(self.limb_name))
            return

        rospy.logdebug("Moving to start position")
        self.move(starting_joint_angles2)

        rospy.logdebug("Initialization done")



    def run(self):
        rate = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            rospy.wait_for_service('marker_pose', 5.0)
            marker_pose_request = ArMarkerPoseRequest(frame='/base')
            marker_pose_response = self.marker_pose_srv(marker_pose_request)
            pose = marker_pose_response.pose

            if pose is not None and pose.pose.position.x != 0:

                orientation = Quaternion(
                    x=0.8,
                    y=-0.5,
                    z=0,
                    w=0
                )

                #pose.pose.position.z += 0.4
                pose.pose.orientation = orientation

                rospy.logdebug("Marker: \n{0}".format(pose))

                pick = copy.deepcopy(pose)
                place = copy.deepcopy(pose)
                place.pose.position.y += 0.5

                pnp_request = PickAndPlaceRequest()
                pnp_request.pick = pick
                pnp_request.place = place

                self.pnp_srv(pnp_request)
                
            rate.sleep()



    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        ikreq.tip_names.append('right_hand')


        # NOT WORKING


        ikreq.seed_mode = ikreq.SEED_USER
        seed = JointState()
        seed.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3',
                         'right_j4', 'right_j5', 'right_j6']
        seed.position = [-2.7, 0.12, -1.4, -1, 1.3, 1.6, 4.6]
        ikreq.seed_angles.append(seed)

        # Once the primary IK task is solved, the solver will then try to bias the
        # the joint angles toward the goal joint configuration. The null space is 
        # the extra degrees of freedom the joints can move without affecting the
        # primary IK task.
        ikreq.use_nullspace_goal.append(True)
        # The nullspace goal can either be the full set or subset of joint angles
        goal = JointState()
        goal.name = ['right_j0', 'right_j1']
        goal.position = [-2.7, 0.12]
        ikreq.nullspace_goal.append(goal)
        ikreq.nullspace_gain.append(0.4)

        try:
            rospy.wait_for_service(self.ik_srv_name, 5.0)
            resp = self.ik_srv(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        

        rospy.logdebug("IKService response: \n{0}".format(resp))

        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        limb_joints = {}
        if (resp.result_type[0] != resp.IK_FAILED):
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints


    def move(self, joint_angles):
        self.limb.move_to_joint_positions(joint_angles)

def main_old():
    rospy.init_node("homography", log_level=rospy.DEBUG)

    limb_name = 'right'
    limb = intera_interface.Limb(limb_name)

    # IK Service
    ik_service_name = "ExternalTools/" + limb_name + "/PositionKinematicsNode/IKService"
    rospy.wait_for_service(ik_service_name)

    # AR Tags
    rospy.wait_for_service('marker_pose')
    marker_pose_service = rospy.ServiceProxy('marker_pose', ArMarkerPose)
    iksvc = rospy.ServiceProxy(ik_service_name, SolvePositionIK)



    # Enable the robot
    _rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
    _init_state = _rs.state().enabled
    rospy.logdebug("Enabling robot... ")
    _rs.enable()

    rate = rospy.Rate(0.1)

    if limb_name == "right":
        starting_joint_angles2 = {'right_j0': -2.7097744140625,
                                 'right_j1': 0.120755859375,
                                 'right_j2': -1.40407421875,
                                 'right_j3': -1.0929814453125,
                                 'right_j4': 1.3393115234375,
                                 'right_j5': 1.6684326171875,
                                 'right_j6': 4.5927880859375}

    else:
        rospy.logerr("Unknown limb: {0}".format(limb_name))

    rospy.logdebug("Moving to start position")
    move(limb, starting_joint_angles2)

    rospy.logdebug("Initialization done")

    while not rospy.is_shutdown():
        rospy.wait_for_service('marker_pose', 5.0)
        marker_pose_request = ArMarkerPoseRequest(frame='/base')
        marker_pose_response = marker_pose_service(marker_pose_request)
        pose = marker_pose_response.pose
        if pose is not None:

            orientation = Quaternion(
                x=0.8,
                y=-0.5,
                z=0,
                w=0
            )

            pose.pose.position.z += 0.4
            pose.pose.orientation = orientation

            rospy.logdebug(pose)

            
            joint_angles = ik_request(iksvc, pose.pose)
            if joint_angles != False:
                move(limb, joint_angles)
            
        rate.sleep()


def main():
    homography = Homography('right')
    homography.setup()
    homography.run()


if __name__ == "__main__":
    sys.exit(main())