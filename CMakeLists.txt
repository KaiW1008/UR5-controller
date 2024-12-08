cmake_minimum_required(VERSION 3.0.2)
project(ur5_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)

catkin_install_python(PROGRAMS
  scripts/motion1_test.py
  scripts/motion2_test.py
  scripts/sin_wave_publisher.py
  scripts/API_demo.py
  scripts/API_LLM.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY scripts/motion_library/
    DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/motion_library
    FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY scripts/API/
    DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/API
    FILES_MATCHING PATTERN "*.py"
)