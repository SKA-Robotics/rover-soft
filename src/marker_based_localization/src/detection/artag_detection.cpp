#include <marker_based_localization/detection/artag_detection.hpp>

// constructor
void ArtagDetection::init(ros::NodeHandle& nh, const std::string& camera_info_topic, double marker_size,
                          double max_new_marker_error, double max_track_error)
{
  marker_detector.SetMarkerSize(marker_size * 100);
  camera_ = std::make_unique<alvar::Camera>(nh, camera_info_topic);
  max_new_marker_error_ = max_new_marker_error;
  max_track_error_ = max_track_error;
}

// detect method
MarkerContainer<AlignedMarker> ArtagDetection::detect(const cv::Mat& image)
{
  MarkerContainer<AlignedMarker> markers;
  if (camera_->getCamInfo_)
  {
    auto image_copy = image.clone();
    marker_detector.Detect(image_copy, camera_.get(), true, false, max_new_marker_error_, max_track_error_);

    for (auto& alvar_marker : *marker_detector.markers)
    {
      unsigned int id = alvar_marker.GetId();
      auto alvar_pose = alvar_marker.pose;
      auto alvar_quaternion = cv::Mat(4, 1, CV_64F);
      alvar_pose.GetQuaternion(alvar_quaternion);

      Translation3d position(alvar_pose.translation[0] / 100.0,  // x
                             alvar_pose.translation[1] / 100.0,  // y
                             alvar_pose.translation[2] / 100.0   // z
      );
      Quaterniond orientation(alvar_quaternion.at<double>(0, 0),   // qw
                              alvar_quaternion.at<double>(1, 0),   // qx
                              alvar_quaternion.at<double>(2, 0),   // qy
                              alvar_quaternion.at<double>(3, 0));  // qz

      Quaterniond optical_to_regular_orientation(0.5, -0.5, -0.5, -0.5);
      orientation = orientation * optical_to_regular_orientation;

      auto alvar_error = alvar_marker.GetError();
      auto error = Vector3d::Constant(alvar_error);

      AlignedMarker marker(id, Isometry3d(position * orientation), error);

      markers.add(marker, true);
    }
  }
  return markers;
}