#ifndef ARTAGDETECTION_HPP
#define ARTAGDETECTION_HPP

#include "marker_based_localization/detection/marker_detection.hpp"
#include "marker_based_localization/markers/aligned_marker.hpp"
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <memory>
#include <ar_track_alvar/MarkerDetector.h>

class ArtagDetection : public MarkerDetector<AlignedMarker>
{
private:
  std::unique_ptr<alvar::Camera> camera_;

public:
  double max_new_marker_error_;
  double max_track_error_;
  alvar::MarkerDetector<alvar::MarkerData> marker_detector;

  ArtagDetection() = default;
  ~ArtagDetection() override = default;
  MarkerContainer<AlignedMarker> detect(const cv::Mat& image) override;

  void init(ros::NodeHandle& nh, const std::string& camera_info_topic, double marker_size, double max_new_marker_error,
            double max_track_error);
};

#endif  // ARTAGDETECTION_HPP