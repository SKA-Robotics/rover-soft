cmake_minimum_required(VERSION 3.0.2)
project(odrive_driver)

find_package(catkin REQUIRED COMPONENTS
  combined_robot_hw
  hardware_interface
  realtime_tools
  roscpp
  rospy
  sirius_msgs
)

# catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS 
    combined_robot_hw
    hardware_interface
    realtime_tools
    roscpp
    rospy 
    sirius_msgs
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

add_library(odrive_hardware_interfaces src/control.cpp)
target_link_libraries(odrive_hardware_interfaces
  ${catkin_LIBRARIES}
)

# catkin_install_python(PROGRAMS
#   scripts/driver.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
install(FILES odrive_hw_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
