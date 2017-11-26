#!/usr/bin/env python
import rospy

from ar_track_alvar_msgs.msg import AlvarMarkers

from ar_marker.srv import *
import tf

class ArService:

  #Callback for when an image is received
  def markerReceived(self, message):
    #Save the image in the instance variable
    #rospy.logdebug("Marker found with ids: {0}".format([m.id for m in message.markers]))
    self.lastMarker = message

    #Print an alert to the console
    #print(rospy.get_name() + ":Image received!")

  #When another node calls the service, return the last image
  def getLastMarker(self, request):
    #Print an alert to the console
    #print("Image requested!")

    #Return the last image
    return ArLastMarkerResponse(self.lastMarker)

  def getMarkerPose(self, request):
    response = ArMarkerPoseResponse(pose=None)

    if self.lastMarker is None:
      return response

    if len(self.lastMarker.markers) == 0:
      return response

    marker = None
    
    if request.marker_type == 0:
      marker = self.lastMarker.markers[0]
    else:
      for m in self.lastMarker.markers:
        if m.id == request.marker_type:
          marker = m
          break

    if marker is None:
      return response

    pose = marker.pose
    pose.header = marker.header
    pose.header.stamp = rospy.Time(0)
    
    self.listener.waitForTransform(request.frame, pose.header.frame_id, rospy.Time(0), rospy.Duration(3.0))
    response.pose = self.listener.transformPose(request.frame, pose)
    
    return response

  def __init__(self):
    #Create an instance variable to store the last image received
    self.lastMarker = None
    self.listener = tf.TransformListener()

    #Initialize the node
    rospy.init_node('ar_marker_listener', log_level=rospy.DEBUG)

    #Subscribe to the image topic
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.markerReceived)

    #Create the service
    #rospy.Service('last_marker', ArLastMarker, self.getLastMarker)
    rospy.Service("marker_pose", ArMarkerPose, self.getMarkerPose)

  def run(self):
    rospy.spin()

#Python's syntax for a main() method
if __name__ == '__main__':
  node = ArService()
  node.run()
