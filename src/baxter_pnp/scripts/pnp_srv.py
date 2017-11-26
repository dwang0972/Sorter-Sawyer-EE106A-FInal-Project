#!/usr/bin/env python

import rospy
import sys

import argparse
import struct
import sys
import copy
from copy import deepcopy

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

from threading import Thread
import time

class PnPService:
    def __init__(self):
        rospy.init_node("pick_and_place_node", log_level=rospy.DEBUG)

        self._hover_distance = rospy.get_param("~hover_distance", 0.2)
        self._limb_name = rospy.get_param("~limb", 'right')
        self._limb = intera_interface.Limb(self._limb_name)
        self._limb.set_joint_position_speed(0.3)

        self._gripper = intera_interface.Gripper(self._limb_name)
        if self._gripper is None:
            rospy.logerr("Gripper error")
        else:
            rospy.logdebug("Gripper OK")

        self._head = intera_interface.Head()

        self._iksvc_name = "ExternalTools/" + self._limb_name + "/PositionKinematicsNode/IKService"
        rospy.wait_for_service(self._iksvc_name, 5.0)
        self._iksvc = rospy.ServiceProxy(self._iksvc_name, SolvePositionIK)

        # Enable the robot
        _rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        _init_state = _rs.state().enabled
        rospy.logdebug("Enabling robot... ")
        _rs.enable()


        rospy.Service("pick_and_place", PickAndPlace, self.execute)
        rospy.Service("move_to_position", PositionMovement, self.move_to_position)
        rospy.Service("move_to_joint", JointMovement, self.move_to_joint)
        rospy.Service("move_head", HeadMovement, self.move_head)
        rospy.Service("throw", Throw, self.throw)
        rospy.logdebug("PNP Ready")

    def execute(self, request):
        response = PickAndPlaceResponse()

        rospy.logdebug("\nPicking...")
        self.pick(request.pick.pose)

        rospy.logdebug("\nPlacing...")
        #self.place(request.place.pose)
        self.throw(None)

        return response

    def pick(self, pose):
        # open the gripper
        status = self.gripper_open()
        if not status:
            rospy.logerr("Gripper open error, moving back to starting point")
            return
        
        # servo above pose
        status = self._approach(deepcopy(pose))
        if not status:
            rospy.logerr("Approach error, moving back to starting point")
            return
        
        # servo to pose
        status = self._servo_to_pose(deepcopy(pose))
        if not status:
            rospy.logerr("Servo to pose error, moving back to starting point")
            return
        
        # close gripper
        status = self.gripper_close()
        if not status:
            rospy.logerr("Gripper close error, moving back to starting point")
            return
        
        # retract to clear object
        status = self._retract()
        if not status:
            rospy.logerr("Retract error, moving back to starting point")
            return

    def place(self, pose):
        # servo above pose
        status = self._approach(deepcopy(pose))
        if not status:
            rospy.logerr("Approach error, moving back to starting point")
            return
        
        # servo to pose
        place_pose = deepcopy(pose)
        place_pose.position.z += 0.05
        status = self._servo_to_pose(place_pose)
        if not status:
            rospy.logerr("Servo to pose error, moving back to starting point")
            return

        # open the gripper
        status = self.gripper_open()
        if not status:
            rospy.logerr("Gripper open error, moving back to starting point")
            return
        
        # retract to clear object
        status = self._retract()
        if not status:
            rospy.logerr("Retract error, moving back to starting point")
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
            rospy.sleep(1.0)
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
        rospy.logdebug("Retract: \n{0}".format(ik_pose))

        joint_angles = self.ik_request(ik_pose)

        # servo up from current pose
        return self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose):
        # servo down to release
        rospy.logdebug("Move to pose: \n{0}".format(pose))

        pose.position.z += 0.08
        
        joint_angles = self.ik_request(pose)

        return self._guarded_move_to_joint_position(joint_angles)


    def move_to_position(self, request):
        response = PositionMovementResponse()

        joint_angles = self.ik_request(request.pose)
        if not self._guarded_move_to_joint_position(joint_angles):
            rospy.logerr("Can't move to position")
            response.status = False
        else:
            response.status = True
        return response

    def move_to_joint(self, request):
        response = JointMovementResponse()

        joint_angles = dict(zip(request.joint_state.name, request.joint_state.position))
        if not self._guarded_move_to_joint_position(joint_angles):
            rospy.logerr("Can't move to position")
            response.status = False
        else:
            response.status = True
        return response


    def move_head(self, request):
        response = HeadMovementResponse()
        angle = request.angle
        rate = rospy.Rate(100)
        while (not rospy.is_shutdown() and
           not (abs(self._head.pan() - angle) <= intera_interface.HEAD_PAN_ANGLE_TOLERANCE)):
            self._head.set_pan(angle, speed=0.3, timeout=0)
            print(self._head.pan())
            rate.sleep()

        return response

    def throw(self, request):
        self._limb.set_joint_position_speed(1.0)

        joint = 'right_j5'
        while not rospy.is_shutdown() and self._limb.joint_angle(joint) >= -2.9:
            self._limb.set_joint_velocities({joint: -1})

        timer = time.time()
        done = False
        while not rospy.is_shutdown():
            #self._limb.set_joint_velocities({'right_j3': +20})
            self._limb.set_joint_velocities({joint: +20})
            if time.time() - timer >= 0.5 and not done:
                Thread(target = self._gripper.open).start()
                done = True
                timer = time.time()
            if time.time() - timer >= 0.5 and done:
                self._limb.set_joint_velocities({joint: +3})
                break

        self._limb.set_joint_position_speed(0.3)

        return ThrowResponse()




def main():
    pnp = PnPService()
    pnp.run()

if __name__ == "__main__":
    sys.exit(main())