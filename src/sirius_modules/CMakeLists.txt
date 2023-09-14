cmake_minimum_required(VERSION 3.0.2)
project(sirius_modules)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sirius_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS 
    roscpp
    rospy 
    sirius_msgs
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/science.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
