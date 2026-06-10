#pragma once

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

namespace gp7_task_executor
{

/**
 * Transform utilities for frame conversions.
 *
 * Wraps tf2_ros::Buffer to provide a clean API for transforming poses
 * between arbitrary frames. All transforms use rclcpp time with a 1s timeout.
 */
class TransformUtils
{
public:
  explicit TransformUtils(
      std::shared_ptr<tf2_ros::Buffer> tf_buffer,
      rclcpp::Node* node);

  /**
   * Transform a pose to the planning frame.
   * Returns the pose unchanged if its frame already matches the planning frame.
   * Throws tf2::TransformException on failure.
   */
  geometry_msgs::msg::PoseStamped transform_to_planning_frame(
      const geometry_msgs::msg::PoseStamped& input_pose,
      const std::string& planning_frame);

  /**
   * Synchronous version: tries viz_frame first, falls back to planning_frame.
   * Returns {pose, frame_used}.
   */
  std::pair<geometry_msgs::msg::PoseStamped, std::string>
  transform_to_viz_or_planning_frame(
      const geometry_msgs::msg::PoseStamped& input_pose,
      const std::string& planning_frame,
      const std::string& viz_frame);

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Node* node_;
  rclcpp::Logger logger() const { return node_->get_logger(); }
};

}  // namespace gp7_task_executor
