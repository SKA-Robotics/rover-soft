/*
 * Copyright 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * This code is based on the Gazebo ROS Magnetometer Sensor plugin. 
 * Original code repository: https://github.com/Darkproduct/gazebo_ros_magnetometer_sensor
 *
 * Modified versions of the original code are licensed under the same
 * Apache License, Version 2.0.
*/

#ifndef GAZEBO_ROS_GPS_SENSOR_H
#define GAZEBO_ROS_GPS_SENSOR_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/sensors/GpsSensor.hh>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <memory>

namespace gazebo
{
  namespace sensors
  {
    class GpsSensor;
  }

  class GazeboRosGpsSensor : public SensorPlugin
  {
  public:
    GazeboRosGpsSensor();
    virtual ~GazeboRosGpsSensor();
    virtual void Load(sensors::SensorPtr sensor_, sdf::ElementPtr sdf_);

  protected:
    virtual void UpdateChild(const gazebo::common::UpdateInfo& info);

  private:
    bool LoadParameters();
    
    std::unique_ptr<ros::NodeHandle> node;
    ros::Publisher gps_data_publisher;
    ros::Publisher gps_velocity_data_publisher;
    sensor_msgs::NavSatFix gps_msg;
    geometry_msgs::Vector3Stamped gps_velocity_msg;
    common::Time last_time;
    gazebo::event::ConnectionPtr connection;
    std::shared_ptr<sensors::GpsSensor> sensor;

    sdf::ElementPtr sdf;

    std::string robot_namespace;
    std::string topic_name;
    std::string body_name;
    double update_rate;
  };
}

#endif // GAZEBO_ROS_GPS_SENSOR_H
