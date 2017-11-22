#!/usr/bin/env python

import rospy
import sys

import argparse
import struct
import sys
import copy

from baxter_pnp.srv import *

import intera_interface

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

from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from sensor_msgs.msg import JointState

class PnPService:
    def __init__(self):
        rospy.init_node("pick_and_place_node", log_level=rospy.DEBUG)

        self._hover_distance = rospy.get_param("~hover_distance", 0.4)
        self._limb_name = rospy.get_param("~limb", 'right')
        self._limb = intera_interface.Limb(self._limb_name)

        self._gripper = intera_interface.Gripper(self._limb_name)
        if self._gripper is None:
            rospy.logerr("Gripper error")
        else:
            rospy.logdebug("Gripper OK")

        self._iksvc_name = "ExternalTools/" + self._limb_name + "/PositionKinematicsNode/IKService"
        rospy.wait_for_service(self._iksvc_name, 5.0)
        self._iksvc = rospy.ServiceProxy(self._iksvc_name, SolvePositionIK)

        # Enable the robot
        _rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        _init_state = _rs.state().enabled
        rospy.logdebug("Enabling robot... ")
        _rs.enable()
        self.move_to_start()

        rospy.Service("pick_and_place", PickAndPlace, self.execute)
        rospy.logdebug("PNP Ready")

    def execute(self, request):
        response = PickAndPlaceResponse()

        rospy.logdebug("\nPicking...")
        self.pick(request.pick.pose)

        rospy.logdebug("\nPlacing...")
        self.place(request.place.pose)

        return response

    def pick(self, pose):
        # open the gripper
        status = self.gripper_open()
        if not status:
            rospy.logerr("Gripper open error, moving back to starting point")
            self.move_to_start()
            return
        
        # servo above pose
        status = self._approach(pose)
        if not status:
            rospy.logerr("Approach error, moving back to starting point")
            self.move_to_start()
            return
        
        # servo to pose
        status = self._servo_to_pose(pose)
        if not status:
            rospy.logerr("Servo to pose error, moving back to starting point")
            self.move_to_start()
            return
        
        # close gripper
        status = self.gripper_close()
        if not status:
            rospy.logerr("Gripper close error, moving back to starting point")
            self.move_to_start()
            return
        
        # retract to clear object
        status = self._retract()
        if not status:
            rospy.logerr("Retract error, moving back to starting point")
            self.move_to_start()
            return

    def place(self, pose):
        # servo above pose
        status = self._approach(pose)
        if not status:
            rospy.logerr("Approach error, moving back to starting point")
            self.move_to_start()
            return
        
        # servo to pose
        status = self._servo_to_pose(pose)
        if not status:
            rospy.logerr("Servo to pose error, moving back to starting point")
            self.move_to_start()
            return

        # open the gripper
        status = self.gripper_open()
        if not status:
            rospy.logerr("Gripper open error, moving back to starting point")
            self.move_to_start()
            return
        
        # retract to clear object
        status = self._retract()
        if not status:
            rospy.logerr("Retract error, moving back to starting point")
            self.move_to_start()
            return

    def run(self):
        rospy.spin()



    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        ikreq.tip_names.append('right_hand')


        # NOT WORKING

        '''
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
        '''

        try:
            rospy.wait_for_service(self._iksvc_name, 5)
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        rospy.logdebug("IKService response: \n{0}".format(resp))

        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        limb_joints = {}
        if (resp.result_type[0] != resp.IK_FAILED):
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            rospy.logdebug("Joints:\n{0}".format(resp.joints[0].name))
            rospy.logdebug("IK Joint Solution:\n{0}".format(limb_joints))
            rospy.logdebug("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
            return True
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
            return False

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)
        return True

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)
        return True

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        rospy.logdebug("Approach: \n{0}".format(approach))
        
        joint_angles = self.ik_request(approach)

        return self._guarded_move_to_joint_position(joint_angles)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x 
        ik_pose.position.y = current_pose['position'].y 
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x 
        ik_pose.orientation.y = current_pose['orientation'].y 
        ik_pose.orientation.z = current_pose['orientation'].z 
        ik_pose.orientation.w = current_pose['orientation'].w
        rospy.logdebug("Retrqct: \n{0}".format(ik_pose))

        joint_angles = self.ik_request(ik_pose)

        # servo up from current pose
        return self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose):
        # servo down to release
        rospy.logdebug("Move to pose: \n{0}".format(pose))
        
        joint_angles = self.ik_request(pose)

        return self._guarded_move_to_joint_position(joint_angles)


    def move_to_start(self):
        if self._limb_name == "right":
            starting_joint_angles2 = {'right_j0': -2.7097744140625,
                                 'right_j1': 0.120755859375,
                                 'right_j2': -1.40407421875,
                                 'right_j3': -1.0929814453125,
                                 'right_j4': 1.3393115234375,
                                 'right_j5': 1.6684326171875,
                                 'right_j6': 4.5927880859375}

        else:
            rospy.logerr("Unknown limb: {0}".format(self._limb_name))
            return

        return self._guarded_move_to_joint_position(starting_joint_angles2)





def main():
    pnp = PnPService()
    pnp.run()

if __name__ == "__main__":
    sys.exit(main())