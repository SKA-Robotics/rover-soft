#ifndef TF2_HELPERS_HPP
#define TF2_HELPERS_HPP

#include "boost/optional.hpp"
#include "geometry_msgs/TransformStamped.h"
#include "marker_based_localization/markers/marker_container.hpp"
#include "ros/ros.h"
#include "tf2/buffer_core.h"
#include "tf2/exceptions.h"
#include <tf2_eigen/tf2_eigen.h>

boost::optional<geometry_msgs::TransformStamped> get_cam_to_base_link_transform(const tf2::BufferCore& tf_buffer,
                                                                                const std::string& camera_tf,
                                                                                std::string base_link,
                                                                                const ros::Time stamp);

template <typename T>
MarkerContainer<T> lookup_markers_in_world_frame(const tf2::BufferCore& tf_buffer,
                                                 const MarkerContainer<T>& markers_in_camera_frame,
                                                 const std::string& world_frame, const ros::Time& stamp)
{
  MarkerContainer<T> markers_in_world_frame;
  for (auto& marker : markers_in_camera_frame)
  {
    // Look for marker pose in world frame in tf
    std::string marker_frame = "artag_" + std::to_string(marker.id);
    geometry_msgs::TransformStamped marker_in_world;
    try
    {
      marker_in_world = tf_buffer.lookupTransform(world_frame, marker_frame, stamp);
    }
    catch (tf2::TransformException ex)
    {
      ROS_WARN("%s", ex.what());
      continue;
    }
    markers_in_world_frame.add({ marker.id, tf2::transformToEigen(marker_in_world) });
  }
  return markers_in_world_frame;
}
#endif  // TF2_HELPERS_HPP