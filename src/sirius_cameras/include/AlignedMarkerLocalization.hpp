#ifndef MARKERDETECTOR_HPP
#define MARKERDETECTOR_HPP

#include "Marker.hpp"
#include "MarkerLocalization.hpp"
#include <opencv2/core.hpp>

class AlignedMarkerLocalization : public MarkerLocalization<AlignedMarker>
{
public:
  Markers<AlignedMarker> markers_in_world_frame;
  AlignedMarkerLocalization()
  {
  }
  AlignedMarkerLocalization(const Markers<AlignedMarker>& markers_in_world_frame)
    : markers_in_world_frame(markers_in_world_frame)
  {
  }
  AlignedMarkerLocalization& setWorldFrames(const Markers<AlignedMarker>& markers_in_world_frame)
  {
    this->markers_in_world_frame = markers_in_world_frame;
    return *this;
  }
  virtual ~AlignedMarkerLocalization() = default;
  virtual Markers<AlignedMarker> localize(const Markers<AlignedMarker>& markers_in_camera_frame)
  {
    return localize(markers_in_camera_frame, markers_in_world_frame);
  };
  virtual Markers<AlignedMarker> localize(const Markers<AlignedMarker>& markers_in_camera_frame,
                                          const Markers<AlignedMarker>& markers_in_world_frame)
  {
    Markers<AlignedMarker> markers_min_error = markers_in_camera_frame;
    auto min_error =
        std::min_element(markers_min_error.begin(), markers_min_error.end(),
                         [](const AlignedMarker& a, const AlignedMarker& b) { return a.maxError() < b.maxError(); })
            ->maxError();
    for (auto& marker : markers_min_error)
    {
      if (marker.maxError() > min_error)
      {
        markers_min_error.removeById(marker.id);
      }
    }
    // Select marker with smallest distance
    auto closest_marker =
        *std::min_element(markers_min_error.begin(), markers_min_error.end(),
                          [](const AlignedMarker& a, const AlignedMarker& b) { return a.distance() < b.distance(); });
    // camera in marker frame
    auto camera_in_marker = closest_marker.transform.inverse();
    // marker in world frame
    auto marker_in_world = markers_in_world_frame.getById(closest_marker.id).transform;
    // camera in world frame
    auto camera_in_world = marker_in_world * camera_in_marker;
  };
};
#endif  // MARKERDETECTOR_HPP