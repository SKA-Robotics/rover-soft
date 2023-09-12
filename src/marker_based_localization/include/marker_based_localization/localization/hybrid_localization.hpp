#ifndef HYBRIDMARKERLOCALIZATION_HPP
#define HYBRIDMARKERLOCALIZATION_HPP

// #include <Marker.hpp>
// #include <MarkerLocalization.hpp>
#include <marker_based_localization/localization/marker_based_localization.hpp>
#include <boost/optional.hpp>
#include "ros/ros.h"

using Eigen::Isometry3d;
using Eigen::Quaterniond;
using Eigen::Translation3d;
using Eigen::Vector3d;

class HybridMarkerLocalization : public MarkerLocalization<Marker>
{
public:
  MarkerContainer<Marker> markers_in_world_frame;
  Eigen::Quaterniond robot_orientation_in_world_frame;
  HybridMarkerLocalization()
  {
  }
  HybridMarkerLocalization(const MarkerContainer<Marker>& markers_in_world_frame)
    : markers_in_world_frame(markers_in_world_frame)
  {
  }
  HybridMarkerLocalization& setWorldFrames(const MarkerContainer<Marker>& markers_in_world_frame)
  {
    this->markers_in_world_frame = markers_in_world_frame;
    return *this;
  }

  HybridMarkerLocalization& setRobotOrientation(const Eigen::Quaterniond& robot_orientation_in_world_frame)
  {
    this->robot_orientation_in_world_frame = robot_orientation_in_world_frame;
    return *this;
  }

  virtual ~HybridMarkerLocalization() = default;

  boost::optional<Eigen::Isometry3d> localize(const MarkerContainer<Marker>& markers_in_camera_frame) const override
  {
    return localize(markers_in_camera_frame, markers_in_world_frame);
  };

  boost::optional<Eigen::Isometry3d> localize(const MarkerContainer<Marker>& markers_in_camera_frame,
                                              const MarkerContainer<Marker>& markers_in_world_frame) const override
  {
    if (markers_in_camera_frame.empty() || markers_in_world_frame.empty())
    {
      return boost::none;
    }

    MarkerContainer<Marker> markers_min_error = markers_in_camera_frame;

    auto min_error =
        std::min_element(markers_min_error.begin(), markers_min_error.end(), [](const auto& a, const auto& b) {
          return a.maxError() < b.maxError();
        })->maxError();
    for (auto& marker : markers_min_error)
    {
      if (marker.maxError() > min_error)
      {
        markers_min_error.removeById(marker.id);
      }
    }

    auto closest_marker = *std::min_element(markers_min_error.begin(), markers_min_error.end(),
                                            [](const auto& a, const auto& b) { return a.distance() < b.distance(); });

    auto marker_world = markers_in_world_frame.getById(closest_marker.id);
    if (!marker_world)
    {
      return boost::none;
    }

    // offset from robot to marker in world frame
    Eigen::Isometry3d marker_robot_offset_in_world = robot_orientation_in_world_frame * closest_marker.transform;
    // offset from marker to robot in world frame
    Eigen::Isometry3d robot_marker_offset_in_world =
        (Eigen::Translation3d(marker_robot_offset_in_world.translation()) * Eigen::Quaterniond::Identity()).inverse();
    // marker in world frame
    Eigen::Isometry3d marker_in_world = marker_world->transform;
    // robot in world frame
    Eigen::Isometry3d robot_in_world = marker_in_world * robot_marker_offset_in_world;
    // robot in world with orientation
    Eigen::Isometry3d robot_in_world_with_orientation = robot_in_world * robot_orientation_in_world_frame;
    return robot_in_world_with_orientation;
  };
};

#endif  // HYBRIDMARKERLOCALIZATION_HPP