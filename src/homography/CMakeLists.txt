cmake_minimum_required(VERSION 2.8.3)
project(homography)

find_package(catkin REQUIRED COMPONENTS
  rospy
  ar_marker
  baxter_pnp
  geometry_msgs
  object_detection
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  ar_marker
  baxter_pnp
  geometry_msgs
  object_detection
  sensor_msgs
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
