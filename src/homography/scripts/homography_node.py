#! /usr/bin/python

import numpy as np
import struct
import sys

import rospy

from ar_track_alvar_msgs.msg import AlvarMarkers
from homography.srv import *

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

listener = tf.TransformListener()

def ik_request(srv, pose):
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
        ns = "ExternalTools/right/PositionKinematicsNode/IKService"
        rospy.wait_for_service(ns, 5.0)
        resp = srv(ikreq)
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

# NOT WORKING
def moveit_request(srv, pose):
    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"
    request.ik_request.ik_link_name = "right_gripper"
    request.ik_request.attempts = 20

    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    request.ik_request.pose_stamped = PoseStamped(header=hdr, pose=pose)

    try:
        #Send the request to the service
        response = srv(request)
        
        #Print the response HERE
        print(response)
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def move(limb, joint_angles):
    limb.move_to_joint_positions(joint_angles)

# NOT WORKING
def moveit_go(group, pose_stamped):
    # Setting position and orientation target
    group.set_pose_target(pose_stamped)

    # TRY THIS
    # Setting just the position without specifying the orientation
    ###group.set_position_target([0.5, 0.5, 0.0])

    # Plan IK and execute
    group.go()

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("homography", log_level=rospy.DEBUG)

    limb_name = 'right'
    limb = intera_interface.Limb(limb_name)

    # IK Service
    ik_service_name = "ExternalTools/" + limb_name + "/PositionKinematicsNode/IKService"
    rospy.wait_for_service(ik_service_name)



    # MoveIt
    rospy.wait_for_service('compute_ik')
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    robot = moveit_commander.RobotCommander()
    rospy.logdebug("Robot groups: {0}".format(robot.get_group_names()))
    group = moveit_commander.MoveGroupCommander("right_arm")
    group.clear_pose_targets()

    display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=20)



    # AR Tags
    rospy.wait_for_service('last_marker')
    last_image_service = rospy.ServiceProxy('last_marker', ArLastMarker)
    iksvc = rospy.ServiceProxy(ik_service_name, SolvePositionIK)



    # Enable the robot
    _rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
    _init_state = _rs.state().enabled
    rospy.logdebug("Enabling robot... ")
    _rs.enable()

    rate = rospy.Rate(0.1)

    overhead_orientation = Quaternion(
                             x=0,
                             y=0,
                             z=0,
                             w=-1)

    if limb_name == "right":
        starting_joint_angles = {'right_w0': 0.6699952259595108,
                                 'right_w1': 1.030009435085784,
                                 'right_w2': -0.4999997247485215,
                                 'right_e0': -1.189968899785275,
                                 'right_e1': 1.9400238130755056,
                                 'right_s0': -0.08000397926829805,
                                 'right_s1': -0.9999781166910306}

        starting_joint_angles2 = {'right_j0': -2.7097744140625,
                                 'right_j1': 0.120755859375,
                                 'right_j2': -1.40407421875,
                                 'right_j3': -1.0929814453125,
                                 'right_j4': 1.3393115234375,
                                 'right_j5': 1.6684326171875,
                                 'right_j6': 4.5927880859375}

        starting_joint_angles3 = [0.6699952259595108,1.030009435085784,-0.4999997247485215,-1.189968899785275,1.9400238130755056,-0.08000397926829805,-0.9999781166910306]

        starting_pose = Pose()
        starting_pose.position.x = 0.5
        starting_pose.position.y = 0.2
        starting_pose.position.z = 0.1
        starting_pose.orientation.w = 1

        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"
        request.ik_request.ik_link_name = "right_gripper"
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        #Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = 0.481
        request.ik_request.pose_stamped.pose.position.y = 0.5
        request.ik_request.pose_stamped.pose.position.z = 0.097
        request.ik_request.pose_stamped.pose.orientation.x = 0.984
        request.ik_request.pose_stamped.pose.orientation.y = 0.141
        request.ik_request.pose_stamped.pose.orientation.z = 0.084
        request.ik_request.pose_stamped.pose.orientation.w = -0.062
    else:
        rospy.logerr("Unknown limb: {0}".format(limb_name))

    rospy.logdebug("Moving to start position")
    move(limb, starting_joint_angles2)

    #response = compute_ik(request)

    #moveit_go(group, starting_pose)

    #group.set_pose_target(starting_pose)
    #group.plan()

    #group_variable_values = group.get_current_joint_values()
    #rospy.logdebug("Joint states: {0}".format(group_variable_values))

    #group.set_joint_value_target(starting_joint_angles3)
    #group.go()

    rospy.logdebug("Initialization done")

    while not rospy.is_shutdown():
        markers = last_image_service().marker.markers
        if len(markers) > 0:
            rospy.logdebug("Found markers: {0}".format(len(markers)))
            orientation = markers[0].pose.pose.orientation
            #q = tftr.quaternion_about_axis(orientation.w, (orientation.x, orientation.y, orientation.z))
            #rospy.logdebug("Position: \n{0}".format(markers[0].pose.pose.position))
            #rospy.logdebug("Marker: {0}".format(markers[0]))

            '''
            (trans,rot) = listener.lookupTransform('/base', '/ar_marker_13', rospy.Time(0))

            rospy.logdebug("Trans: {0}".format(trans))

            

            pose = Pose(
                position=Point(x=trans[0], y=trans[1], z=trans[2]+0.4),
                orientation=orientation
            )
            '''

            orientation = Quaternion(
                x=0.8,
                y=-0.5,
                z=0,
                w=0
            )

            pose = markers[0].pose
            pose.header = markers[0].header
            pose = listener.transformPose("/base", pose)

            pose.pose.position.z += 0.4
            pose.pose.orientation = orientation

            rospy.logdebug(pose)

            
            joint_angles = ik_request(iksvc, pose.pose)
            if joint_angles != False:
                move(limb, joint_angles)
            
            '''
            response = moveit_request(compute_ik, pose)
            if response:
                moveit_go(group, response.ik_request.pose_stamped)
            '''

            '''
            position = np.array([   [markers[0].pose.pose.position.x],
                                    [markers[0].pose.pose.position.y],
                                    [markers[0].pose.pose.position.z],
                                    [1]])

            g = listener.fromTranslationRotation(trans, rot)
            rospy.logdebug("g: {0}".format(g))

            rot_mat = tftr.quaternion_matrix(rot)
            rot_mat[0][3] = trans[0]
            rot_mat[1][3] = trans[1]
            rot_mat[2][3] = trans[2]
            rospy.logdebug('r: {0}'.format(rot_mat))

            position2 = np.dot(rot_mat, position)

            pose2 = Pose()
            pose2.position.x = position2[0][0]
            pose2.position.y = position2[1][0]
            pose2.position.z = position2[2][0] + 0.2
            pose2.orientation = overhead_orientation

            rospy.logdebug(pose2)

            try:
                pose = markers[0].pose
                pose.header = markers[0].header

                rospy.logdebug(pose)

                #if listener.waitForTransform(pose.header.frame_id, "/base", pose.header.stamp, rospy.Duration(1)):
                pose2 = listener.transformPose("/base", pose)

                joint_angles = ik_request(iksvc, pose2)
                rospy.logdebug("Joint angles: {0}".format(joint_angles))

                if joint_angles != False:
                    move(limb, joint_angles)
                #else:
                #    rospy.logdebug("Wait for transform timeout")
            except tf.LookupException as e:
                rospy.logdebug(e)
            '''


        rate.sleep()


if __name__ == "__main__":
    sys.exit(main())