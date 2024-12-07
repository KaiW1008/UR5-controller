cmake_minimum_required(VERSION 3.0.2)
project(ur5_control)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

## catkin specific configuration
catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/cartesian.py
  scripts/sin_wave_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY scripts/ur5_motion_library/
    DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/ur5_motion_library
    FILES_MATCHING PATTERN "*.py"
)

