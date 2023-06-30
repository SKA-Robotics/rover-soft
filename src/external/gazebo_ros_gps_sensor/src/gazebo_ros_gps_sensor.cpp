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
*/

#include <gazebo_ros_gps_sensor/gazebo_ros_gps_sensor.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

GZ_REGISTER_SENSOR_PLUGIN(gazebo::GazeboRosGpsSensor)

gazebo::GazeboRosGpsSensor::GazeboRosGpsSensor(): 
  SensorPlugin(),
  sensor(nullptr),
  node(nullptr)
{}

gazebo::GazeboRosGpsSensor::~GazeboRosGpsSensor()
{
  if (connection.get())
  {
    connection.reset();
    connection = event::ConnectionPtr();
  }

  if (node != nullptr)
  {
    node->shutdown();
    delete node;
  }
}

void gazebo::GazeboRosGpsSensor::Load(gazebo::sensors::SensorPtr sensor_, sdf::ElementPtr sdf_)
{
  sdf=sdf_;
  sensor=dynamic_cast<gazebo::sensors::GpsSensor*>(sensor_.get());

  if(sensor==nullptr)
  {
    ROS_FATAL("Error: Sensor pointer is NULL!");
    return;
  }

  sensor->SetActive(true);

  if(!LoadParameters())
  {
    ROS_FATAL("Error Loading Parameters!");
    return;
  }

  if (!ros::isInitialized()) //check if ros is initialized properly
  {
    ROS_FATAL("ROS has not been initialized!");
    return;
  }

  node = new ros::NodeHandle(this->robot_namespace);

  gps_data_publisher = node->advertise<sensor_msgs::NavSatFix>(topic_name,1);
  gps_velocity_data_publisher = node->advertise<geometry_msgs::Vector3Stamped>(topic_name + "_velocity",1);
  connection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosGpsSensor::UpdateChild, this, _1));

  last_time = sensor->LastUpdateTime();
}

void gazebo::GazeboRosGpsSensor::UpdateChild(const gazebo::common::UpdateInfo &/*_info*/)
{
  common::Time current_time = sensor->LastUpdateTime();

  if(update_rate>0 && (current_time-last_time).Double() < 1.0/update_rate) //update rate check
    return;

  if(gps_data_publisher.getNumSubscribers() > 0)
  {
    gps_msg.latitude  = sensor->Latitude().Degree();
    gps_msg.longitude = sensor->Longitude().Degree();
    gps_msg.altitude  = sensor->Altitude();
    gps_velocity_msg.vector.x = sensor->VelocityEast();
    gps_velocity_msg.vector.y = sensor->VelocityNorth();
    gps_velocity_msg.vector.z = sensor->VelocityUp();

    //preparing message header
    gps_msg.header.frame_id   = body_name;
    gps_msg.header.stamp.sec  = current_time.sec;
    gps_msg.header.stamp.nsec = current_time.nsec;

    gps_velocity_msg.header.frame_id   = body_name;
    gps_velocity_msg.header.stamp.sec  = current_time.sec;
    gps_velocity_msg.header.stamp.nsec = current_time.nsec;

    //publishing data
    gps_data_publisher.publish(gps_msg);
    gps_velocity_data_publisher.publish(gps_velocity_msg);

    ros::spinOnce();
  }

  last_time = current_time;
}

bool gazebo::GazeboRosGpsSensor::LoadParameters()
{
  //loading parameters from the sdf file

  //NAMESPACE
  if (sdf->HasElement("robotNamespace"))
  {
    robot_namespace = sdf->Get<std::string>("robotNamespace") + "/";
    ROS_INFO_STREAM("<robotNamespace> set to: " << robot_namespace);
  }
  else
  {
    std::string scoped_name = sensor->ParentName();
    std::size_t it          = scoped_name.find("::");

    robot_namespace = "/" + scoped_name.substr(0, it) + "/";
    ROS_WARN_STREAM("missing <robotNamespace>, set to default: " << robot_namespace);
  }

  //TOPIC
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

  //BODY NAME
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

  //UPDATE RATE
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

  return true;
}
