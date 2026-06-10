#include "gp7_task_executor/transform_utils.h"

#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace gp7_task_executor
{

TransformUtils::TransformUtils(
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    rclcpp::Node* node)
  : tf_buffer_(std::move(tf_buffer))
  , node_(node)
{
}

geometry_msgs::msg::PoseStamped
TransformUtils::transform_to_planning_frame(
    const geometry_msgs::msg::PoseStamped& input_pose,
    const std::string& planning_frame)
{
  if (input_pose.header.frame_id == planning_frame)
  {
    RCLCPP_DEBUG(logger(),
                 "[Transform] No transform needed: frame '%s' == planning frame '%s'",
                 input_pose.header.frame_id.c_str(), planning_frame.c_str());
    return input_pose;
  }

  geometry_msgs::msg::PoseStamped pose_stamped_in;
  pose_stamped_in.header.stamp = node_->get_clock()->now();
  pose_stamped_in.header.frame_id = input_pose.header.frame_id;
  pose_stamped_in.pose = input_pose.pose;

  const auto transformed = tf_buffer_->transform(pose_stamped_in, planning_frame);

  RCLCPP_INFO(logger(),
              "[Transform] TF '%s' -> '%s' succeeded.  xyz: (%.4f, %.4f, %.4f)",
              input_pose.header.frame_id.c_str(), planning_frame.c_str(),
              transformed.pose.position.x,
              transformed.pose.position.y,
              transformed.pose.position.z);

  return transformed;
}

std::pair<geometry_msgs::msg::PoseStamped, std::string>
TransformUtils::transform_to_viz_or_planning_frame(
    const geometry_msgs::msg::PoseStamped& input_pose,
    const std::string& planning_frame,
    const std::string& viz_frame)
{
  if (input_pose.header.frame_id == viz_frame)
  {
    return {input_pose, viz_frame};
  }

  if (input_pose.header.frame_id == planning_frame)
  {
    return {input_pose, planning_frame};
  }

  try
  {
    tf_buffer_->lookupTransform(viz_frame, planning_frame,
                               rclcpp::Time(0),
                               rclcpp::Duration::from_seconds(0.1));
    const auto transformed = transform_to_planning_frame(input_pose, planning_frame);
    return {transformed, planning_frame};
  }
  catch (const tf2::TransformException&)
  {
    RCLCPP_WARN(logger(),
                "[Transform] Viz frame '%s' not available via TF; using planning frame '%s'.",
                viz_frame.c_str(), planning_frame.c_str());
    const auto transformed = transform_to_planning_frame(input_pose, planning_frame);
    return {transformed, planning_frame};
  }
}

}  // namespace gp7_task_executor
