cmake_minimum_required(VERSION 2.8.3)
project(sirius_gazebo_ros_p3d)

# Compile as C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS gazebo_ros roscpp std_msgs nav_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

# Find gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Declare ROS dynamic reconfigure parameters

# Catkin specific configuration
catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  nav_msgs
  INCLUDE_DIRS
  include
  LIBRARIES
  sirius_gazebo_ros_p3d)

# Build

add_library(sirius_gazebo_ros_p3d src/sirius_gazebo_ros_p3d.cpp)
target_link_libraries(sirius_gazebo_ros_p3d ${GAZEBO_LIBRARIES}
                      ${catkin_LIBRARIES})

# Install

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

install(
  TARGETS sirius_gazebo_ros_p3d
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
