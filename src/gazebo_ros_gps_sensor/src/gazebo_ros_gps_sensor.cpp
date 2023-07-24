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

#include <gazebo_ros_gps_sensor/gazebo_ros_gps_sensor.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <cmath>

GZ_REGISTER_SENSOR_PLUGIN(gazebo::GazeboRosGpsSensor)

namespace gazebo
{
  GazeboRosGpsSensor::GazeboRosGpsSensor() : SensorPlugin(), sensor(nullptr), node(nullptr)
  {
  }

  GazeboRosGpsSensor::~GazeboRosGpsSensor()
  {
    if (connection)
    {
      connection.reset();
    }

    if (node)
    {
      node->shutdown();
      delete node;
    }
  }

  void GazeboRosGpsSensor::Load(sensors::SensorPtr sensor_, sdf::ElementPtr sdf_)
  {
    sdf = sdf_;
    sensor = dynamic_cast<sensors::GpsSensor*>(sensor_.get());

    if (!sensor)
    {
      ROS_FATAL("Error: Sensor pointer is NULL!");
      return;
    }

    sensor->SetActive(true);

    if (!LoadParameters())
    {
      ROS_FATAL("Error Loading Parameters!");
      return;
    }

    if (!ros::isInitialized())
    {
      ROS_FATAL("ROS has not been initialized!");
      return;
    }

    node = new ros::NodeHandle(robot_namespace);

    gps_data_publisher = node->advertise<sensor_msgs::NavSatFix>(topic_name, 1);
    gps_velocity_data_publisher = node->advertise<geometry_msgs::Vector3Stamped>(topic_name + "_velocity", 1);
    connection = event::Events::ConnectWorldUpdateBegin([this](const gazebo::common::UpdateInfo& info) {
      UpdateChild(info);
    });

    last_time = sensor->LastUpdateTime();
  }

  void GazeboRosGpsSensor::UpdateChild(const gazebo::common::UpdateInfo& /*_info*/)
  {
    common::Time current_time = sensor->LastUpdateTime();

    if (update_rate > 0 && (current_time - last_time).Double() < 1.0 / update_rate)
    {
      return;
    }

    if (gps_data_publisher.getNumSubscribers() > 0)
    {
      gps_msg.latitude = sensor->Latitude().Degree();
      gps_msg.longitude = sensor->Longitude().Degree();
      gps_msg.altitude = sensor->Altitude();
      gps_velocity_msg.vector.x = sensor->VelocityEast();
      gps_velocity_msg.vector.y = sensor->VelocityNorth();
      gps_velocity_msg.vector.z = sensor->VelocityUp();

      gps_msg.header.frame_id = body_name;
      gps_msg.header.stamp.sec = current_time.sec;
      gps_msg.header.stamp.nsec = current_time.nsec;

      gps_velocity_msg.header.frame_id = body_name;
      gps_velocity_msg.header.stamp.sec = current_time.sec;
      gps_velocity_msg.header.stamp.nsec = current_time.nsec;

      gps_data_publisher.publish(gps_msg);
      gps_velocity_data_publisher.publish(gps_velocity_msg);

      ros::spinOnce();
    }

    last_time = current_time;
  }

  bool GazeboRosGpsSensor::LoadParameters()
  {
    if (sdf->HasElement("robotNamespace"))
    {
      robot_namespace = sdf->Get<std::string>("robotNamespace") + "/";
      ROS_INFO_STREAM("<robotNamespace> set to: " << robot_namespace);
    }
    else
    {
      std::string scoped_name = sensor->ParentName();
      std::size_t it = scoped_name.find("::");

      robot_namespace = "/" + scoped_name.substr(0, it) + "/";
      ROS_WARN_STREAM("missing <robotNamespace>, set to default: " << robot_namespace);
    }

    if (sdf->HasElement("topicName"))
    {
      topic_name = robot_namespace + sdf->Get<std::string>("topicName");
      ROS_INFO_STREAM("<topicName> set to: " << topic_name);
    }
    else
    {
      topic_name = robot_namespace + "/gps";
      ROS_WARN_STREAM("missing <topicName>, set to /namespace/default: " << topic_name);
    }

    if (sdf->HasElement("frameName"))
    {
      body_name = sdf->Get<std::string>("frameName");
      ROS_INFO_STREAM("<frameName> set to: " << body_name);
    }
    else
    {
      ROS_FATAL("missing <frameName>, cannot proceed");
      return false;
    }

    if (sdf->HasElement("updateRateHZ"))
    {
      update_rate = sdf->Get<double>("updateRateHZ");
      ROS_INFO_STREAM("<updateRateHZ> set to: " << update_rate);
    }
    else
    {
      update_rate = 1.0;
      ROS_WARN_STREAM("missing <updateRateHZ>, set to default: " << update_rate);
    }
    // Load spherical_coordinates parameters
    if (sdf->HasElement("spherical_coordinates"))
    {
      // Set Gazebo world origin to the spherical_coordinates values
      gazebo::physics::WorldPtr world = gazebo::physics::get_world(sensor->WorldName());
      if (world){
        sdf::ElementPtr spherical_coordinates = sdf->GetElement("spherical_coordinates");

        if (spherical_coordinates->HasElement("latitude_deg")){
          auto latitude = spherical_coordinates->Get<double>("latitude_deg") * M_PI / 180;
          world->SphericalCoords()->SetLatitudeReference(latitude);
          ROS_INFO_STREAM("Latitude: " << latitude);
        }

        if (spherical_coordinates->HasElement("longitude_deg")){
          auto longitude = spherical_coordinates->Get<double>("longitude_deg") * M_PI / 180;
          world->SphericalCoords()->SetLongitudeReference(longitude);
          ROS_INFO_STREAM("Longitude: " << longitude);
        }

        if (spherical_coordinates->HasElement("elevation")){
          auto elevation = spherical_coordinates->Get<double>("elevation");
          world->SphericalCoords()->SetElevationReference(elevation);
          ROS_INFO_STREAM("Elevation: " << elevation);
        }
        
        if (spherical_coordinates->HasElement("heading_deg")){
          auto heading = spherical_coordinates->Get<double>("heading_deg") * M_PI / 180;
          world->SphericalCoords()->SetHeadingOffset(heading);
          ROS_INFO_STREAM("Heading: " << heading);
        }
      }
    }
    return true;
  }
}
