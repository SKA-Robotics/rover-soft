#ifndef MARKERDETECTOR_HPP
#define MARKERDETECTOR_HPP

#include "marker_based_localization/markers/marker_container.hpp"
#include <opencv2/core.hpp>

template <typename T>  // T is as Marker
class MarkerDetector
{
public:
  virtual ~MarkerDetector() = default;
  virtual MarkerContainer<T> detect(const cv::Mat& image) = 0;
};
#endif  // MARKERDETECTOR_HPP