#ifndef ALIGNEDMARKERLOCALIZATION_HPP
#define ALIGNEDMARKERLOCALIZATION_HPP

#include <marker_based_localization/localization/marker_based_localization.hpp>
#include <marker_based_localization/markers/aligned_marker.hpp>
#include <boost/optional.hpp>

class AlignedMarkerLocalization : public MarkerLocalization<AlignedMarker>
{
public:
  MarkerContainer<AlignedMarker> markers_in_world_frame;
  AlignedMarkerLocalization()
  {
  }
  AlignedMarkerLocalization(const MarkerContainer<AlignedMarker>& markers_in_world_frame)
    : markers_in_world_frame(markers_in_world_frame)
  {
  }
  AlignedMarkerLocalization& setWorldFrames(const MarkerContainer<AlignedMarker>& markers_in_world_frame)
  {
    this->markers_in_world_frame = markers_in_world_frame;
    return *this;
  }
  virtual ~AlignedMarkerLocalization() = default;

  boost::optional<Eigen::Isometry3d>
  localize(const MarkerContainer<AlignedMarker>& markers_in_camera_frame) const override
  {
    return localize(markers_in_camera_frame, markers_in_world_frame);
  };

  boost::optional<Eigen::Isometry3d>
  localize(const MarkerContainer<AlignedMarker>& markers_in_camera_frame,
           const MarkerContainer<AlignedMarker>& markers_in_world_frame) const override
  {
    if (markers_in_camera_frame.empty() || markers_in_world_frame.empty())
    {
      return boost::none;
    }

    MarkerContainer<AlignedMarker> markers_min_error = markers_in_camera_frame;

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
#endif  // ALIGNEDMARKERLOCALIZATION_HPP