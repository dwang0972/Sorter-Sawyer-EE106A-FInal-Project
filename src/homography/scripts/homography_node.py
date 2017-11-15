#! /usr/bin/python

import numpy as np
import struct
import sys

import rospy

from ar_track_alvar_msgs.msg import AlvarMarkers
from homography.srv import *

import tf.transformations as tftr

import intera_interface

from intera_core_msgs.srv import (
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

limb = None

def ik_request(srv, pose, hover=0.2):
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    ikreq = SolvePositionIKRequest()
    pose.position.z += hover
    ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
    try:
        resp = srv(ikreq)
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


def move(joint_angles):
    limb.move_to_joint_positions(joint_angles)

def main():
    rospy.init_node("homography")

    limb = intera_interface.Limb('right')

    ik_service_name = "ExternalTools/right/PositionKinematicsNode/IKService"

    rospy.wait_for_service('last_marker')
    rospy.wait_for_service(ik_service_name)

    last_image_service = rospy.ServiceProxy('last_marker', ArLastMarker)
    iksvc = rospy.ServiceProxy(ik_service_name, SolvePositionIK)

    _rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
    _init_state = _rs.state().enabled
    rospy.logdebug("Enabling robot... ")
    _rs.enable()

    rospy.logdebug("Initialization done")

    rate = rospy.Rate(0.1)

    starting_joint_angles = {'right_w0': 0.6699952259595108,
                             'right_w1': 1.030009435085784,
                             'right_w2': -0.4999997247485215,
                             'right_e0': -1.189968899785275,
                             'right_e1': 1.9400238130755056,
                             'right_s0': -0.08000397926829805,
                            'right_s1': -0.9999781166910306}

    #move(starting_joint_angles)

    while not rospy.is_shutdown():
        markers = last_image_service().marker.markers
        if len(markers) > 0:
            rospy.logdebug("Found marker")
            orientation = markers[0].pose.pose.orientation
            q = tftr.quaternion_about_axis(orientation.w, (orientation.x, orientation.y, orientation.z))
            rospy.logdebug("Position: \n{0}".format(markers[0].pose.pose.position))
            joint_angles = ik_request(iksvc, markers[0].pose.pose)
            rospy.logdebug("Joint angles: {0}".format(joint_angles))
        rate.sleep()


if __name__ == "__main__":
    sys.exit(main())