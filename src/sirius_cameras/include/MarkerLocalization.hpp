#ifndef MARKERDETECTOR_HPP
#define MARKERDETECTOR_HPP

#include "Marker.hpp"
#include <opencv2/core.hpp>

template <typename T>  // T is as Marker
class MarkerLocalization
{
public:
  Markers<T> markers_in_world_frame;
  MarkerLocalization()
  {
  }
  MarkerLocalization(const Markers<T>& markers_in_world_frame) : markers_in_world_frame(markers_in_world_frame)
  {
  }
  MarkerLocalization<T>& setWorldFrames(const Markers<T>& markers_in_world_frame)
  {
    this->markers_in_world_frame = markers_in_world_frame;
    return *this;
  }
  virtual ~MarkerLocalization() = default;
  virtual Markers<T> localize(const Markers<T>& markers_in_camera_frame)
  {
    return localize(markers_in_camera_frame, markers_in_world_frame);
  };
  virtual Markers<T> localize(const Markers<T>& markers_in_camera_frame, const Markers<T>& markers_in_world_frame) = 0;
};
#endif  // MARKERDETECTOR_HPP