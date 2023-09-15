#include <marker_based_localization/localization/hybrid_localization.hpp>
#include "Eigen/src/Core/Matrix.h"
#include "geometry_msgs/Transform.h"
#include "image_transport/camera_subscriber.h"
#include "marker_based_localization/detection/artag_detection.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include <algorithm>
#include <ar_track_alvar/MarkerDetector.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <marker_based_localization/detection/artag_detection.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <marker_based_localization/localization/aligned_marker_based_localization.hpp>
#include <string>
#include <boost/optional.hpp>
#include <nav_msgs/Odometry.h>
#include <marker_based_localization/ros/tf2_helpers.hpp>
#include <marker_based_localization/ros/marker_msgs.hpp>
#include <marker_msgs/MarkerWithCovarianceArray.h>

using namespace std;

ros::Publisher marker_publisher;
ros::Publisher pose_publisher;
tf2_ros::Buffer tf_buffer;
ArtagDetection artag_detector;

std::string base_link;
std::string world_frame;
double max_error;

void camera_callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  try
  {
    AlignedMarkerLocalization marker_localization;
    MarkerContainer<AlignedMarker> markers_in_world_frame;
    auto cv_image = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);
    auto markers = artag_detector.detect(cv_image->image);
    if (markers.empty())
    {
      return;
    }

    auto cam_to_base_link =
        get_cam_to_base_link_transform(tf_buffer, image->header.frame_id, base_link, image->header.stamp);
    if (!cam_to_base_link)
    {
      return;
    }

    markers.changeReferenceFrame(tf2::transformToEigen(*cam_to_base_link));

    markers_in_world_frame = lookup_markers_in_world_frame(tf_buffer, markers, world_frame, image->header.stamp);
    marker_localization.setWorldFrames(markers_in_world_frame);
    auto pose = marker_localization.localize(markers);

    // publish pose
    if (pose)
    {
      geometry_msgs::PoseWithCovarianceStamped pose_msg;
      pose_msg.header.frame_id = world_frame;
      pose_msg.header.stamp = image->header.stamp;
      pose_msg.pose.pose = tf2::toMsg(*pose);
      pose_publisher.publish(pose_msg);
    }

    auto markers_msg = toMsg(markers, image->header.stamp, image->header.frame_id);
    marker_publisher.publish(markers_msg);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
  }
}

int main(int argc, char* argv[])
{
  double marker_size;
  double max_new_marker_error;
  double max_track_error;
  double update_rate;

  ros::init(argc, argv, "marker_based_localization");
  ros::NodeHandle nh("~");
  tf2_ros::TransformListener tf_listener(tf_buffer);

  nh.param("marker_size", marker_size, 0.1);
  nh.param("max_new_marker_error", max_new_marker_error, 0.08);
  nh.param("max_track_error", max_track_error, 0.2);
  nh.param("update_rate", update_rate, 8.0);
  nh.getParam("base_link", base_link);
  nh.param<std::string>("world_frame", world_frame, "map");
  max_error = std::max(max_new_marker_error, max_track_error);

  marker_publisher = nh.advertise<marker_msgs::MarkerWithCovarianceArray>("markers", 0);
  pose_publisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 0);

  image_transport::ImageTransport it(nh);
  auto camera_subscriber = it.subscribeCamera("image", 1, &camera_callback);
  artag_detector.init(nh, camera_subscriber.getInfoTopic(), marker_size, max_new_marker_error, max_track_error);

  ros::Rate rate(update_rate);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
