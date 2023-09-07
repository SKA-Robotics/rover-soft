#ifndef MARKERDETECTOR_HPP
#define MARKERDETECTOR_HPP

#include "Marker.hpp"
#include <opencv2/core.hpp>

template <typename T>  // T is as Marker
class MarkerDetector
{
public:
  virtual ~MarkerDetector() = default;
  virtual Markers<T> detect(const cv::Mat& image) = 0;
};
#endif  // MARKERDETECTOR_HPP