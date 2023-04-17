cmake_minimum_required(VERSION 3.0.2)
project(manip_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
