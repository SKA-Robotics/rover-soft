cmake_minimum_required(VERSION 3.0.2)
project(marker_based_localization)

find_package(
  catkin REQUIRED
  COMPONENTS ar_track_alvar
             camera_info_manager
             roscpp
             sensor_msgs
             tf2_eigen)

catkin_package(
  CATKIN_DEPENDS
  ar_track_alvar
  roscpp
  sensor_msgs
  tf2_eigen)
include_directories(include/ ${catkin_INCLUDE_DIRS})

add_library(ArtagDetection src/detection/artag_detection.cpp)
target_link_libraries(ArtagDetection ${catkin_LIBRARIES})
add_dependencies(ArtagDetection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(HybridLocalization src/nodes/hybrid_localization_ros.cpp)
target_link_libraries(HybridLocalization ${catkin_LIBRARIES} ArtagDetection)
add_dependencies(HybridLocalization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(MarkerBasedLocalization src/nodes/marker_based_localization_ros.cpp)
target_link_libraries(MarkerBasedLocalization ${catkin_LIBRARIES} ArtagDetection)
add_dependencies(MarkerBasedLocalization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
