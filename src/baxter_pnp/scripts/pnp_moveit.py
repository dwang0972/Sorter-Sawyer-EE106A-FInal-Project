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

import moveit_commander
from moveit_msgs.msg import (
    DisplayTrajectory
)

from sensor_msgs.msg import JointState

class PnPService:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("pick_and_place_moveit", log_level=rospy.DEBUG)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("right_arm")


        self.group.clear_pose_targets()
        self.group.allow_replanning(True)
        rospy.logdebug(self.group.get_current_joint_values())

        self._hover_distance = rospy.get_param("~hover_distance", 0.4)
        self._limb_name = rospy.get_param("~limb", 'right')
        self._limb = intera_interface.Limb(self._limb_name)

        self._gripper = intera_interface.Gripper(self._limb_name)
        if self._gripper is None:
            rospy.logerr("Gripper error")
        else:
            rospy.logdebug("Gripper OK")

        # Enable the robot
        _rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        _init_state = _rs.state().enabled
        rospy.logdebug("Enabling robot... ")
        _rs.enable()

        rospy.Service("pick_and_place", PickAndPlace, self.execute)
        rospy.Service("move_to_position", PositionMovement, self.move_to_position)
        rospy.Service("move_to_joint", JointMovement, self.move_to_joint)
        rospy.Service("move_head", HeadMovement, self.move_head)
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
        self.group.set_pose_target(pose)

    def _guarded_move_to_joint_position(self):
        self.group.go()

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
        
        self.ik_request(approach)

        return self._guarded_move_to_joint_position()

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

        self.ik_request(ik_pose)

        # servo up from current pose
        return self._guarded_move_to_joint_position()

    def _servo_to_pose(self, pose):
        # servo down to release
        rospy.logdebug("Move to pose: \n{0}".format(pose))
        
        self.ik_request(pose)

        return self._guarded_move_to_joint_position()


    def move_to_position(self, request):
        response = PositionMovementResponse()

        print(self.group.get_current_pose())
        print(request.pose)

        self.group.set_pose_target(request.pose)
        self.group.plan()
        self.group.clear_pose_target()
        #self.group.go()

        '''
        plan1 = self.group.plan()

        rospy.sleep(2)

        display_trajectory = DisplayTrajectory()

        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan1)
        self.display_trajectory_publisher.publish(display_trajectory)

        rospy.sleep(2)
        '''

        '''
        if not self._guarded_move_to_joint_position(joint_angles):
            rospy.logerr("Can't move to position")
            response.status = False
        else:
            response.status = True
        '''

        #self.group.clear_pose_targets()
        return response

    def move_to_joint(self, request):
        response = JointMovementResponse()

        joint_angles = dict(zip(request.joint_state.name, request.joint_state.position))
        rospy.logdebug(self.group.get_current_joint_values())
        rospy.logdebug(request.joint_state.position)
        
        self.group.set_joint_value_target(request.joint_state.position)
        self.group.plan()
        #self.group.go()
        
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

    def add_table(self, request):
        response = TablePositionResponse()
        table_z = response.table_z
        collision_object = moveit_msgs.msg.CollisionObject()
        collision_object
        return response





def main():
    pnp = PnPService()
    pnp.run()

if __name__ == "__main__":
    sys.exit(main())