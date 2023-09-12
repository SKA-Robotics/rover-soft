#include <marker_based_localization/localization/hybrid_localization.hpp>
#include "geometry_msgs/Transform.h"
#include "image_transport/camera_subscriber.h"
#include "marker_based_localization/detection/artag_detection.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include <algorithm>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar/MarkerDetector.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <marker_based_localization/detection/artag_detection.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <marker_based_localization/localization/aligned_marker_based_localization.hpp>
#include <string>
#include <boost/optional.hpp>
#include <nav_msgs/Odometry.h>

using namespace std;

ros::Publisher marker_publisher;
ros::Publisher pose_publisher;
tf2_ros::Buffer tf_buffer;
ArtagDetection artag_detector;
HybridMarkerLocalization hybrid_marker_localization;

std::string base_link;
std::string world_frame;
double max_error;
Eigen::Translation3d offset{ -0.09, 0, 0 };

boost::optional<geometry_msgs::TransformStamped> get_cam_to_base_link_transform(std::string camera_tf,
                                                                                std::string base_link, ros::Time stamp)
{
  geometry_msgs::TransformStamped cam_to_base_link;
  if (!base_link.empty() || camera_tf.substr(camera_tf.length() - 7) == "_optical")
  {
    try
    {
      if (base_link.empty())
      {
        base_link = camera_tf.substr(0, camera_tf.length() - 7);
      }
      cam_to_base_link = tf_buffer.lookupTransform(base_link, camera_tf, stamp);
      return cam_to_base_link;
    }
    catch (tf2::TransformException ex)
    {
      ROS_WARN("%s", ex.what());
      return boost::none;
    }
  }
  cam_to_base_link = geometry_msgs::TransformStamped();
  cam_to_base_link.transform.rotation.w = 1;
  return cam_to_base_link;
}

// lookup markers_in_world_frame
MarkerContainer<AlignedMarker>
lookup_markers_in_world_frame(const MarkerContainer<AlignedMarker>& markers_in_camera_frame,
                              const std::string& world_frame, const ros::Time& stamp)
{
  MarkerContainer<AlignedMarker> markers_in_world_frame;
  for (auto& marker : markers_in_camera_frame)
  {
    // Look for marker pose in world frame in tf
    std::string marker_frame = "artag_" + std::to_string(marker.id);
    geometry_msgs::TransformStamped marker_in_world;
    try
    {
      marker_in_world = tf_buffer.lookupTransform(world_frame, marker_frame, stamp);
    }
    catch (tf2::TransformException ex)
    {
      ROS_WARN("%s", ex.what());
      continue;
    }
    markers_in_world_frame.add({ marker.id, tf2::transformToEigen(marker_in_world) });
  }
  return markers_in_world_frame;
}
void odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
  hybrid_marker_localization.setRobotOrientation(
      Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z));
}
void camera_callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  try
  {
    ar_track_alvar_msgs::AlvarMarkers alvar_markers;
    AlignedMarkerLocalization marker_localization;
    MarkerContainer<AlignedMarker> aligned_markers_in_world_frame;
    MarkerContainer<Marker> markers_in_world_frame;
    MarkerContainer<Marker> markers;

    auto cv_image = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);
    auto aligned_markers = artag_detector.detect(cv_image->image);
    if (aligned_markers.empty())
    {
      return;
    }

    auto cam_to_base_link = get_cam_to_base_link_transform(image->header.frame_id, base_link, image->header.stamp);
    if (!cam_to_base_link)
    {
      return;
    }
    aligned_markers.changeReferenceFrame(tf2::transformToEigen(*cam_to_base_link));

    for (auto& marker : aligned_markers)
    {
      markers.add(marker.toMarker(offset));
    }

    aligned_markers_in_world_frame = lookup_markers_in_world_frame(aligned_markers, world_frame, image->header.stamp);
    marker_localization.setWorldFrames(aligned_markers_in_world_frame);
    auto pose2 = marker_localization.localize(aligned_markers);

    for (auto& marker : aligned_markers_in_world_frame)
    {
      markers_in_world_frame.add(marker.toMarker(offset));
    }
    hybrid_marker_localization.setWorldFrames(markers_in_world_frame);
    auto pose = hybrid_marker_localization.localize(markers);

    // publish pose
    if (pose)
    {
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.frame_id = world_frame;
      pose_msg.header.stamp = image->header.stamp;
      pose_msg.pose = tf2::toMsg(*pose);
      pose_publisher.publish(pose_msg);
    }

    for (auto& marker : aligned_markers)
    {
      auto marker_in_base_link = tf2::eigenToTransform(marker.transform);

      ar_track_alvar_msgs::AlvarMarker ar_pose_marker;
      tf2::Transform marker_in_base_link_tf;
      tf2::fromMsg(marker_in_base_link.transform, marker_in_base_link_tf);
      tf2::toMsg(marker_in_base_link_tf, ar_pose_marker.pose.pose);
      ar_pose_marker.header.frame_id = base_link;
      ar_pose_marker.header.stamp = image->header.stamp;
      ar_pose_marker.id = marker.id;

      auto error = marker.error.x();
      ar_pose_marker.confidence = 1.0 - std::min(error, max_error) / max_error;

      alvar_markers.markers.push_back(ar_pose_marker);
    }
    alvar_markers.header.stamp = image->header.stamp;
    alvar_markers.header.frame_id = base_link;
    marker_publisher.publish(alvar_markers);
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

  ros::init(argc, argv, "marker_localization");
  ros::NodeHandle nh("~");
  tf2_ros::TransformListener tf_listener(tf_buffer);

  nh.param("marker_size", marker_size, 0.1);
  nh.param("max_new_marker_error", max_new_marker_error, 0.08);
  nh.param("max_track_error", max_track_error, 0.2);
  nh.param("update_rate", update_rate, 8.0);
  nh.getParam("base_link", base_link);
  nh.param<std::string>("world_frame", world_frame, "map");
  max_error = std::max(max_new_marker_error, max_track_error);

  marker_publisher = nh.advertise<ar_track_alvar_msgs::AlvarMarkers>("markers", 0);
  pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("pose", 0);

  image_transport::ImageTransport it(nh);
  auto camera_subscriber = it.subscribeCamera("image", 1, &camera_callback);
  artag_detector.init(nh, camera_subscriber.getInfoTopic(), marker_size, max_new_marker_error, max_track_error);
  // subscribe to IMU message
  auto imu_subscriber = nh.subscribe("odom", 1, odom_callback);
  ros::Rate rate(update_rate);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
