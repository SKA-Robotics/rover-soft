#ifndef HYBRIDMARKERLOCALIZATION_HPP
#define HYBRIDMARKERLOCALIZATION_HPP

#include <Marker.hpp>
#include <MarkerLocalization.hpp>
#include <opencv2/core.hpp>
#include <boost/optional.hpp>

class HybridMarkerLocalization : public MarkerLocalization<OffsetMarker>
{
public:
  Markers<OffsetMarker> markers_in_world_frame;
  Eigen::Quaterniond robot_orientation;
  HybridMarkerLocalization()
  {
  }
  HybridMarkerLocalization(const Markers<OffsetMarker>& markers_in_world_frame)
    : markers_in_world_frame(markers_in_world_frame)
  {
  }
  HybridMarkerLocalization& setWorldFrames(const Markers<OffsetMarker>& markers_in_world_frame)
  {
    this->markers_in_world_frame = markers_in_world_frame;
    return *this;
  }
  
  HybridMarkerLocalization& setRobotOrientation(const Eigen::Quaterniond& robot_orientation)
  {
    this->robot_orientation = robot_orientation;
    return *this;
  }

  virtual ~HybridMarkerLocalization() = default;

  boost::optional<Eigen::Isometry3d> localize(const Markers<OffsetMarker>& markers_in_camera_frame) const override
  {
    return localize(markers_in_camera_frame, markers_in_world_frame);
  };

  boost::optional<Eigen::Isometry3d> localize(const Markers<OffsetMarker>& markers_in_camera_frame,
                                              const Markers<OffsetMarker>& markers_in_world_frame) const override
  {
    if (markers_in_camera_frame.empty() || markers_in_world_frame.empty())
    {
      return boost::none;
    }

    Markers<OffsetMarker> markers_min_error = markers_in_camera_frame;

    auto min_error =
        std::min_element(markers_min_error.begin(), markers_min_error.end(),
                         [](const auto& a, const auto& b) { return a.maxError() < b.maxError(); })
            ->maxError();
    for (auto& marker : markers_min_error)
    {
      if (marker.maxError() > min_error)
      {
        markers_min_error.removeById(marker.id);
      }
    }
    
    auto closest_marker =
        *std::min_element(markers_min_error.begin(), markers_min_error.end(),
                          [](const auto& a, const auto& b) { return a.distance() < b.distance(); });
    // camera in marker frame
    auto camera_in_marker = closest_marker.transform.inverse();

    // marker object from world container
    auto marker_world = markers_in_world_frame.getById(closest_marker.id);
    if (!marker_world)
    {
      return boost::none;
    }

    // marker in world frame
    auto marker_in_world = marker_world->transform;

    // camera in world frame
    return marker_in_world * camera_in_marker;
  };
};

#endif // HYBRIDMARKERLOCALIZATION_HPP