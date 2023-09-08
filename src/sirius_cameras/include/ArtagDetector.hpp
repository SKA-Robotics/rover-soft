#ifndef ARTAGDETECTOR_HPP
#define ARTAGDETECTOR_HPP

#include "Marker.hpp"
#include "MarkerDetector.hpp"
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <ar_track_alvar/MarkerDetector.h>
#include <memory>

class ArtagDetector : public MarkerDetector<AlignedMarker>
{
private:
  std::unique_ptr<alvar::Camera> camera_;

public:
  double max_new_marker_error_;
  double max_track_error_;
  alvar::MarkerDetector<alvar::MarkerData> marker_detector;

  ArtagDetector() = default;
  ~ArtagDetector() override = default;
  Markers<AlignedMarker> detect(const cv::Mat& image) override;

  void init(ros::NodeHandle& nh, const std::string& camera_info_topic, double marker_size, double max_new_marker_error,
            double max_track_error);
};

#endif  // ARTAGDETECTOR_HPP