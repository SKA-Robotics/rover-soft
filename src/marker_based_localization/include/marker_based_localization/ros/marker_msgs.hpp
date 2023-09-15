#ifndef ROS_MARKER_MSGS_HPP
#define ROS_MARKER_MSGS_HPP

#include <array>
#include <marker_based_localization/markers/marker.hpp>
#include <marker_msgs/MarkerWithCovariance.h>
#include <marker_msgs/MarkerWithCovarianceStamped.h>
#include <marker_msgs/MarkerWithCovarianceArray.h>
#include <marker_based_localization/markers/marker_container.hpp>
#include <string>
#include "Eigen/src/Core/Matrix.h"
#include "ros/time.h"
#include <tf2_eigen/tf2_eigen.h>
#include <ros/ros.h>

template <typename T>
marker_msgs::MarkerWithCovariance toMsg(const T& marker, double confidence = 1.0)
{
  auto msg = marker_msgs::MarkerWithCovariance();
  msg.marker.ids.push_back(marker.id);
  msg.marker.ids_confidence.push_back(confidence);
  msg.marker.pose = tf2::toMsg(marker.transform);
  msg.covariance[0] = marker.error[0];
  msg.covariance[7] = marker.error[1];
  msg.covariance[14] = marker.error[2];
  if (marker.error.size() > 3)
  {
    msg.covariance[21] = marker.error[3];
    msg.covariance[28] = marker.error[4];
    msg.covariance[35] = marker.error[5];
  }
  return msg;
}

template <typename T>
marker_msgs::MarkerWithCovarianceStamped toMsg(const T& marker, ros::Time stamp, const std::string& camera_frame = {},
                                               double confidence = 1.0)
{
  auto msg = marker_msgs::MarkerWithCovarianceStamped();
  msg.header.stamp = stamp;
  msg.header.frame_id = camera_frame;
  msg.marker = toMsg(marker, confidence);
  return msg;
}

template <typename T>
marker_msgs::MarkerWithCovarianceArray toMsg(const MarkerContainer<T>& markers, ros::Time stamp,
                                             const std::string& camera_frame = {},
                                             std::vector<double> confidence = {})
{
  auto msg = marker_msgs::MarkerWithCovarianceArray();
  msg.header.stamp = stamp;
  msg.header.frame_id = camera_frame;

  for (int i = 0; i < markers.size(); i++)
  {
    if (confidence.size() <= i)
    {
      msg.markers.push_back(toMsg(markers[i], 1));
    }
    else
    {
      msg.markers.push_back(toMsg(markers[i], confidence[i]));
    }
  }
  return msg;
}

#endif  // ROS_MARKER_MSGS_HPP