cmake_minimum_required(VERSION 3.0.2)
project(sirius_cameras)

find_package(
  catkin REQUIRED
  COMPONENTS ar_track_alvar
             camera_info_manager
             depthai_bridge
             depthai_ros_msgs
             roscpp
             sensor_msgs
             tf_conversions)
find_package(depthai CONFIG REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  ar_track_alvar
  camera_info_manager
  depthai_bridge
  depthai_ros_msgs
  roscpp
  sensor_msgs
  tf_conversions)
include_directories(include/ ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_oak_d_w src/oak_d_w.cpp)
target_link_libraries(${PROJECT_NAME}_oak_d_w PRIVATE ${catkin_LIBRARIES} depthai::core)

add_library(ArtagDetector src/ArtagDetector.cpp)
target_link_libraries(ArtagDetector ${catkin_LIBRARIES})
add_dependencies(ArtagDetector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(marker_localization src/marker_localization.cpp)
target_link_libraries(marker_localization ${catkin_LIBRARIES} ArtagDetector)
add_dependencies(marker_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}_oak_d_w DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS marker_localization DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
