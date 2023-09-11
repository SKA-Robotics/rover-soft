#ifndef MARKERLOCALIZATION_HPP
#define MARKERLOCALIZATION_HPP

#include "Marker.hpp"
#include <opencv2/core.hpp>
#include <boost/optional.hpp>

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
  virtual boost::optional<Eigen::Isometry3d> localize(const Markers<T>& markers_in_camera_frame) const
  {
    return localize(markers_in_camera_frame, markers_in_world_frame);
  };
  virtual boost::optional<Eigen::Isometry3d> localize(const Markers<T>& markers_in_camera_frame,
                                                      const Markers<T>& markers_in_world_frame) const = 0;

};
#endif  // MARKERLOCALIZATION_HPP