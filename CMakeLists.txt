cmake_minimum_required(VERSION 3.0.2)
project(ur5_control)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  scripts/sin_wave_publisher.py
  scripts/motion_library.py   ## only for testing
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)