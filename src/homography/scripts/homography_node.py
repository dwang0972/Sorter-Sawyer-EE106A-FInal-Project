#! /usr/bin/python

import numpy as np
import struct
import sys

import rospy

from ar_track_alvar_msgs.msg import AlvarMarkers
from homography.srv import *

import tf
import tf.transformations as tftr

import baxter_interface

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

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

listener = tf.TransformListener()

def ik_request(srv, pose):
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    ikreq = SolvePositionIKRequest()
    ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
    try:
        resp = srv(ikreq)
        print(resp)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return False
    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    limb_joints = {}
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
    else:
        rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
        return False
    return limb_joints


def move(limb, joint_angles):
    limb.move_to_joint_positions(joint_angles)

def main():
    rospy.init_node("homography")

    _limb = 'right'
    limb = baxter_interface.Limb(_limb)

    ik_service_name = "ExternalTools/" + _limb + "/PositionKinematicsNode/IKService"

    rospy.wait_for_service('last_marker')
    rospy.wait_for_service(ik_service_name)

    last_image_service = rospy.ServiceProxy('last_marker', ArLastMarker)
    iksvc = rospy.ServiceProxy(ik_service_name, SolvePositionIK)

    _rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
    _init_state = _rs.state().enabled
    print("Enabling robot... ")
    _rs.enable()

    print("Initialization done")

    rate = rospy.Rate(0.1)

    if _limb == "left":
        starting_joint_angles = {'left_w0': 0.6699952259595108,
                                 'left_w1': 1.030009435085784,
                                 'left_w2': -0.4999997247485215,
                                 'left_e0': -1.189968899785275,
                                 'left_e1': 1.9400238130755056,
                                 'left_s0': -0.08000397926829805,
                                 'left_s1': -0.9999781166910306}
    elif _limb == "right":
        starting_joint_angles = {'right_w0': 0.6699952259595108,
                                 'right_w1': 1.030009435085784,
                                 'right_w2': -0.4999997247485215,
                                 'right_e0': -1.189968899785275,
                                 'right_e1': 1.9400238130755056,
                                 'right_s0': -0.08000397926829805,
                                 'right_s1': -0.9999781166910306}

    print("Moving to start position")
    move(limb, starting_joint_angles)

    while not rospy.is_shutdown():
        markers = last_image_service().marker.markers
        if len(markers) > 0:
            print("Found markers: {0}".format(len(markers)))
            orientation = markers[0].pose.pose.orientation
            q = tftr.quaternion_about_axis(orientation.w, (orientation.x, orientation.y, orientation.z))
            #print("Position: \n{0}".format(markers[0].pose.pose.position))
            #print("Marker: {0}".format(markers[0]))

            (trans,rot) = listener.lookupTransform('/base', '/ar_marker_13n', rospy.Time(0))

            print("Trans: {0}".format(trans))

            overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)

            orientation = Quaternion(
                x=rot[0],
                y=rot[1],
                z=rot[2],
                w=rot[3]
            )

            pose = Pose(
                position=Point(x=trans[0], y=trans[1], z=trans[2]+0.1),
                orientation=overhead_orientation
            )

            print(pose)

            joint_angles = ik_request(iksvc, pose)
            if joint_angles != False:
                move(limb, joint_angles)

            '''
            position = np.array([   [markers[0].pose.pose.position.x],
                                    [markers[0].pose.pose.position.y],
                                    [markers[0].pose.pose.position.z],
                                    [1]])

            g = listener.fromTranslationRotation(trans, rot)
            print("g: {0}".format(g))

            rot_mat = tftr.quaternion_matrix(rot)
            rot_mat[0][3] = trans[0]
            rot_mat[1][3] = trans[1]
            rot_mat[2][3] = trans[2]
            print('r: {0}'.format(rot_mat))

            position2 = np.dot(rot_mat, position)

            pose2 = Pose()
            pose2.position.x = position2[0][0]
            pose2.position.y = position2[1][0]
            pose2.position.z = position2[2][0] + 0.2
            pose2.orientation = overhead_orientation

            print(pose2)

            try:
                pose = markers[0].pose
                pose.header = markers[0].header

                print(pose)

                #if listener.waitForTransform(pose.header.frame_id, "/base", pose.header.stamp, rospy.Duration(1)):
                pose2 = listener.transformPose("/base", pose)

                joint_angles = ik_request(iksvc, pose2)
                print("Joint angles: {0}".format(joint_angles))

                if joint_angles != False:
                    move(limb, joint_angles)
                #else:
                #    print("Wait for transform timeout")
            except tf.LookupException as e:
                print(e)
            '''


        rate.sleep()


if __name__ == "__main__":
    sys.exit(main())