cmake_minimum_required(VERSION 3.0.2)
project(baxter_joint_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin
  REQUIRED
  COMPONENTS
  rospy
  sensor_msgs
  baxter_interface
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  sensor_msgs
  baxter_interface
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

