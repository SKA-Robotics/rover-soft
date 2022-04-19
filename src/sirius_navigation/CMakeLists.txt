cmake_minimum_required(VERSION 3.0.2)
project(sirius_navigation)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/simple_goal_relay.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
