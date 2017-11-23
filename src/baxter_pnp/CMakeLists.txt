cmake_minimum_required(VERSION 2.8.3)
project(baxter_pnp)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rospack
  baxter_core_msgs
  baxter_gazebo
  baxter_interface
  baxter_tools
  baxter_tools
  gazebo_ros
  gazebo_msgs
  geometry_msgs
  intera_core_msgs
  intera_interface
  moveit_core
  moveit_msgs
  sensor_msgs
)

add_service_files(
  FILES
  JointMovement.srv
  PickAndPlace.srv
  PositionMovement.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  intera_core_msgs
  moveit_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  rospack
  baxter_core_msgs
  baxter_gazebo
  baxter_interface
  baxter_tools
  baxter_tools
  gazebo_ros
  gazebo_msgs
  geometry_msgs
  intera_core_msgs
  intera_interface
  moveit_core
  moveit_msgs
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

