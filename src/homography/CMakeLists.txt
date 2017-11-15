cmake_minimum_required(VERSION 2.8.3)
project(homography)

find_package(catkin REQUIRED COMPONENTS
  rospy
  ar_track_alvar
  ar_track_alvar_msgs
  message_generation
  intera_core_msgs
  intera_interface
)

add_service_files(
  FILES
  ArLastMarker.srv
)

generate_messages(
  DEPENDENCIES
  ar_track_alvar_msgs
  intera_core_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  ar_track_alvar
  ar_track_alvar_msgs
  intera_core_msgs
  intera_interface
  message_runtime
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
