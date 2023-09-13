cmake_minimum_required(VERSION 3.0.2)
project(sirius_cameras)

find_package(
  catkin REQUIRED
  COMPONENTS camera_info_manager
             depthai_bridge
             depthai_ros_msgs
             roscpp
             sensor_msgs)
find_package(depthai CONFIG REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  camera_info_manager
  depthai_bridge
  depthai_ros_msgs
  roscpp
  sensor_msgs)
include_directories(include/ ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_oak_d_w src/oak_d_w.cpp)
target_link_libraries(${PROJECT_NAME}_oak_d_w PRIVATE ${catkin_LIBRARIES} depthai::core)

install(TARGETS ${PROJECT_NAME}_oak_d_w DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(${PROJECT_NAME}_oak_d_w_rect src/oak_d_w.cpp)
target_link_libraries(${PROJECT_NAME}_oak_d_w_rect PRIVATE ${catkin_LIBRARIES} depthai::core)

install(TARGETS ${PROJECT_NAME}_oak_d_w_rect DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
