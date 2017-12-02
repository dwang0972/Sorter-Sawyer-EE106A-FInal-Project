#!/usr/bin/env python

import rospy
import sys

import argparse
import math
import struct
import sys
import tf

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

from ik_throw_solver import IkThrowSolver

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

        self.listener = tf.TransformListener()

        # Enable the robot
        _rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        _init_state = _rs.state().enabled
        rospy.logdebug("Enabling robot... ")
        _rs.enable()


        rospy.Service("pick_and_place", PickAndPlace, self.pnp)
        rospy.Service("pick_and_throw", PickAndPlace, self.pnt)
        rospy.Service("move_to_position", PositionMovement, self.move_to_position)
        rospy.Service("move_to_joint", JointMovement, self.move_to_joint)
        rospy.Service("move_head", HeadMovement, self.move_head)
        #rospy.Service("throw", Throw, self.throw)
        rospy.logdebug("PNP Ready")

    def pnp(self, request):
        response = PickAndPlaceResponse()

        rospy.logdebug("\nPicking...")
        status = self.pick(request.pick.pose)
        if not status:
            return response

        rospy.logdebug("\nPlacing...")
        status = self.place(request.place.pose)
        if not status:
            return response

        return response

    def pnt(self, request):
        response = PickAndPlaceResponse()

        rospy.logdebug("\nPicking...")
        self.pick(request.pick.pose)

        rospy.logdebug("\nThrowing...")
        self.throw(request.place.pose)
        #self.throw(None)

        return response

    def pick(self, pose):
        # open the gripper
        status = self.gripper_open()
        if not status:
            rospy.logerr("Gripper open error, moving back to starting point")
            return status

        pick_pose = deepcopy(pose)
        pick_pose.position.z += 0.04
        
        # servo above pose
        status = self._approach(pick_pose)
        if not status:
            rospy.logerr("Approach error, moving back to starting point")
            return status
        
        # servo to pose
        status = self._servo_to_pose(pick_pose)
        if not status:
            rospy.logerr("Servo to pose error, moving back to starting point")
            return status
        
        # close gripper
        status = self.gripper_close()
        if not status:
            rospy.logerr("Gripper close error, moving back to starting point")
            return status
        
        # retract to clear object
        status = self._retract()
        if not status:
            rospy.logerr("Retract error, moving back to starting point")
            return status

        return True

    def place(self, pose):
        # servo above pose
        place_pose = deepcopy(pose)
        place_pose.position.y -= 0.1
        place_pose.position.z -= 0.1

        status = self._approach(place_pose)
        if not status:
            rospy.logwarn("Approach error, trying to throw")
            self.throw(pose)
            return
        
        # servo to pose
        status = self._servo_to_pose(place_pose)
        if not status:
            rospy.logwarn("Servo to pose error, trying to throw")
            self.throw(pose)
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
        approach = deepcopy(pose)
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
            rate.sleep()

        return response

    def throw(self, pose):
        self.prepare_throw(pose)
        rospy.loginfo("Prepared to throw")
        time.sleep(1)
        self.execute_throw(pose)

    def prepare_throw(self, pose):
        point = pose.position

        rospy.logdebug("Target: {0}".format(point))

        d = (192.5+168.5+136.3)/1000.0

        alpha = math.atan2(point.x, -point.y)
        beta = math.acos(d/math.sqrt(math.pow(point.x, 2) + math.pow(point.y, 2)))

        theta0 = math.pi - alpha - beta
        theta1 = math.atan2(point.z, math.sqrt(math.pow(point.x, 2) + math.pow(point.y, 2) + math.pow(point.z, 2)))

        rospy.logdebug("Alpha: {0}, Beta: {1}".format(alpha, beta))
        rospy.logdebug("Theta0: {0}, Theta1: {1}".format(theta0, theta1))

        joints = {
            'right_j0': -theta0,
            'right_j1': -theta1,
            'right_j2': -2.9,
            'right_j3': 0,
            'right_j4': -2.9,
            'right_j5': 0,
            'right_j6': 1.57
        }

        return self._guarded_move_to_joint_position(joints)

    def execute_throw(self, pose):
        self.listener.waitForTransform("/base", "/right_l3", rospy.Time(0), rospy.Duration(3.0))
        p, q = self.listener.lookupTransform("/base", "/right_l3", rospy.Time(0))

        rospy.logdebug("Solving...")
        solver = IkThrowSolver(
            math.sqrt(math.pow(p[0],2) + math.pow(p[1], 2)),
            p[2],
            math.sqrt(math.pow(pose.position.x,2) + math.pow(pose.position.y, 2)), 
            pose.position.z + 0.1
        )

        theta3_f, theta5_f, t = solver.solve().x


        joint3 = 'right_j3'
        joint5 = 'right_j5'

        w3 = 1.957
        w5 = 3.485

        while not rospy.is_shutdown() and self._limb.joint_angle(joint3) <= 2.4:
            self._limb.set_joint_velocities({joint3: 1})

        while not rospy.is_shutdown() and self._limb.joint_angle(joint5) >= -2.4:
            self._limb.set_joint_velocities({joint5: -1})

        time.sleep(1)

        theta3_0 = self._limb.joint_angle(joint3)
        theta5_0 = self._limb.joint_angle(joint5)

        rospy.logdebug("Theta3 0: {0} Theta5 0: {1}".format(theta3_0, theta5_0))
        rospy.logdebug("Theta3 F: {0} Theta5 F: {1}".format(theta3_f, theta5_f))

        t_exec3, t_exec5 = solver.get_execution_times(theta3_0, theta5_0, theta3_f, theta5_f)
        t_start3, t_start5 = solver.get_start_times(theta3_0, theta5_0, theta3_f, theta5_f)

        t_total = solver.get_total_execution_time(theta3_0, theta5_0, theta3_f, theta5_f)

        t_gripper = t_total

        
        rospy.logdebug("Start joint 3: {0}".format(t_start3))
        rospy.logdebug("Start joint 5: {0}".format(t_start5))
        rospy.logdebug("Total time: {0}".format(t_total))


        gripper_thread = Thread(target = self._gripper.open)

        started = False


        start_timer = time.time()
        while not rospy.is_shutdown():
            new_timer = time.time()

            dic = {}

            if new_timer - start_timer >= t_start3:
                dic[joint3] = -99
            
            if new_timer - start_timer >= t_start5:
                dic[joint5] = 99
                
            #print(dic)
            self._limb.set_joint_velocities(dic)
            print(self._limb.joint_velocities())

            if new_timer - start_timer >= t_gripper and not started:
                gripper_thread.start()
                started = True

            if new_timer - start_timer >= t_total + 0.1:
                break

            time.sleep(0.01)

        gripper_thread.join()
        time.sleep(5)




def main():
    pnp = PnPService()
    pnp.run()

if __name__ == "__main__":
    sys.exit(main())