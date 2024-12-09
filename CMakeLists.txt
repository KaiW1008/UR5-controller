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
  src/motion1_test.py
  src/motion2_test.py
  src/sin_wave_publisher.py
  src/API_demo.py
  src/API_LLM.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY src/motion_library/
    DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/motion_library
    FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY src/API/
    DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/API
    FILES_MATCHING PATTERN "*.py"
)