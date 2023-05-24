cmake_minimum_required(VERSION 3.0.2)
project(sirius_cameras)

find_package(
  catkin REQUIRED COMPONENTS roscpp camera_info_manager depthai_ros_msgs
                             sensor_msgs depthai_bridge)
find_package(depthai CONFIG REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp camera_info_manager depthai_ros_msgs
               sensor_msgs depthai_bridge)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_oak_d_w src/oak_d_w.cpp)
target_link_libraries(${PROJECT_NAME}_oak_d_w PRIVATE ${catkin_LIBRARIES}
                                                      depthai::core)
install(TARGETS ${PROJECT_NAME}_oak_d_w
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
