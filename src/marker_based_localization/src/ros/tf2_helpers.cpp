#include <marker_based_localization/ros/tf2_helpers.hpp>

boost::optional<geometry_msgs::TransformStamped> get_cam_to_base_link_transform(const tf2::BufferCore& tf_buffer,
                                                                                const std::string& camera_tf,
                                                                                std::string base_link,
                                                                                const ros::Time stamp)
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