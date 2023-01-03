#include <ros/ros.h>
#include "gaja_driver/RosGajaDriver.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "gaja_driver");
  ros::NodeHandle nodeHandle;

  RosGajaDriver driver(nodeHandle);

  ros::spin();
  return 0;
}
