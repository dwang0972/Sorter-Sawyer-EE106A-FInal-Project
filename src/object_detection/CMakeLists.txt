cmake_minimum_required(VERSION 2.8.3)
project(object_detection)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
)

#############
## Install ##
#############

install(PROGRAMS
   scripts/circle.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch models)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
