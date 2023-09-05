#include "geometry_msgs/Transform.h"
#include "image_transport/camera_subscriber.h"
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

using namespace alvar;
using namespace std;

Camera* camera;
ros::Publisher marker_publisher;
ros::Publisher pose_publisher;
tf2_ros::Buffer tf_buffer;
MarkerDetector<MarkerData> marker_detector;

std::string base_link;
std::string world_frame;
double max_new_marker_error;
double max_track_error;
double max_error;
void publishPose(const ar_track_alvar_msgs::AlvarMarkers& markers)
{
  auto max_confidence_marker =
      std::max_element(markers.markers.begin(), markers.markers.end(),
                       [](const ar_track_alvar_msgs::AlvarMarker& a, const ar_track_alvar_msgs::AlvarMarker& b) {
                         return a.confidence < b.confidence;
                       });
  // Extract the marker pose from the message
  auto pose = max_confidence_marker->pose.pose;
  // Pose to tf transform
  tf2::Transform marker_in_base_link_tf;
  tf2::fromMsg(pose, marker_in_base_link_tf);
  auto base_link_in_marker_tf = marker_in_base_link_tf.inverse();

  std::string marker_frame = "artag_" + std::to_string(max_confidence_marker->id);

  geometry_msgs::TransformStamped marker_to_world;
  try
  {
    marker_to_world = tf_buffer.lookupTransform(world_frame, marker_frame, max_confidence_marker->header.stamp);
  }
  catch (tf2::TransformException ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
  tf2::Stamped<tf2::Transform> marker_to_world_tf;
  tf2::fromMsg(marker_to_world, marker_to_world_tf);
  tf2::Transform base_link_in_world_tf = marker_to_world_tf * base_link_in_marker_tf;

  geometry_msgs::PoseStamped base_link_in_world_pose;
  tf2::toMsg(base_link_in_world_tf, base_link_in_world_pose.pose);
  base_link_in_world_pose.header.frame_id = world_frame;
  base_link_in_world_pose.header.stamp = max_confidence_marker->header.stamp;

  pose_publisher.publish(base_link_in_world_pose);
}

void getCapCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  if (camera->getCamInfo_)
  {
    try
    {
      ar_track_alvar_msgs::AlvarMarkers markers;
      geometry_msgs::TransformStamped cam_to_base_link;

      auto cv_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
      marker_detector.Detect(cv_image->image, camera, true, false, max_new_marker_error, max_track_error);
      if (!base_link.empty())
      {
        try
        {
          cam_to_base_link = tf_buffer.lookupTransform(base_link, image->header.frame_id, image->header.stamp);
        }
        catch (tf2::TransformException ex)
        {
          ROS_WARN("%s", ex.what());
          return;
        }
      }
      else
      {
        cam_to_base_link = geometry_msgs::TransformStamped();
        cam_to_base_link.transform.rotation.w = 1;
      }

      for (size_t i = 0; i < marker_detector.markers->size(); i++)
      {
        int id = (*(marker_detector.markers))[i].GetId();
        auto pose = (*(marker_detector.markers))[i].pose;
        auto quat = cv::Mat(4, 1, CV_64F);
        pose.GetQuaternion(quat);

        tf2::Vector3 origin(pose.translation[0] / 100.0,  // x
                            pose.translation[1] / 100.0,  // y
                            pose.translation[2] / 100.0   // z
        );

        tf2::Quaternion rotation(quat.at<double>(1, 0),   // qx
                                 quat.at<double>(2, 0),   // qy
                                 quat.at<double>(3, 0),   // qz
                                 quat.at<double>(0, 0));  // qw

        tf2::Transform marker_in_camera_tf(rotation, origin);

        tf2::Stamped<tf2::Transform> cam_to_base_link_tf;
        tf2::fromMsg(cam_to_base_link, cam_to_base_link_tf);
        tf2::Transform marker_in_base_link_tf = cam_to_base_link_tf * marker_in_camera_tf;

        ar_track_alvar_msgs::AlvarMarker ar_pose_marker;
        tf2::toMsg(marker_in_base_link_tf, ar_pose_marker.pose.pose);
        ar_pose_marker.header.frame_id = base_link;
        ar_pose_marker.header.stamp = image->header.stamp;
        ar_pose_marker.id = id;

        auto error = (*(marker_detector.markers))[i].GetError();
        error = std::min(error, max_error) / max_error;
        ar_pose_marker.confidence = 1.0 - error;

        markers.markers.push_back(ar_pose_marker);
      }

      markers.header.stamp = image->header.stamp;
      markers.header.frame_id = base_link;
      marker_publisher.publish(markers);
      publishPose(markers);
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("%s", e.what());
    }
  }
}

int main(int argc, char* argv[])
{
  double marker_size;
  double update_rate;

  ros::init(argc, argv, "marker_localization");
  ros::NodeHandle nh("~");
  tf2_ros::TransformListener tf_listener(tf_buffer);

  nh.param("marker_size", marker_size, 10.0);
  nh.param("max_new_marker_error", max_new_marker_error, 0.08);
  nh.param("max_track_error", max_track_error, 0.2);
  nh.param("update_rate", update_rate, 8.0);
  nh.getParam("base_link", base_link);
  nh.param<std::string>("world_frame", world_frame, "map");
  max_error = std::max(max_new_marker_error, max_track_error);

  marker_detector.SetMarkerSize(marker_size);
  marker_publisher = nh.advertise<ar_track_alvar_msgs::AlvarMarkers>("markers", 0);
  pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("pose", 0);

  image_transport::ImageTransport it(nh);
  auto camera_subscriber = it.subscribeCamera("image", 1, &getCapCallback);
  camera = new Camera(nh, camera_subscriber.getInfoTopic());

  ros::Rate rate(update_rate);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
