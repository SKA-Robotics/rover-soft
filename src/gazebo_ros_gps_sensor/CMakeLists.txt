cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ros_gps_sensor)

# Compile as C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS gazebo_ros roscpp std_msgs sensor_msgs)
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
  sensor_msgs
  INCLUDE_DIRS
  include
  LIBRARIES
  gazebo_ros_gps_sensor)

# Build

add_library(gazebo_ros_gps_sensor src/gazebo_ros_gps_sensor.cpp)
target_link_libraries(gazebo_ros_gps_sensor ${GAZEBO_LIBRARIES}
                      ${catkin_LIBRARIES})

# Install

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

install(
  TARGETS gazebo_ros_gps_sensor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
