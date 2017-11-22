cmake_minimum_required(VERSION 2.8.3)
project(object_detection)

find_package(catkin REQUIRED COMPONENTS
  ar_marker
  cv_bridge
  geometry_msgs
  message_generation
  rospy
  sensor_msgs
  std_msgs
)

add_service_files(
  FILES
  ObjectPose.srv
  ObjectPoses.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  ar_marker
  cv_bridge
  geometry_msgs
  message_runtime
  rospy
  sensor_msgs
  std_msgs
)

#############
## Install ##
#############

install(PROGRAMS
   scripts/*.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch models)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
