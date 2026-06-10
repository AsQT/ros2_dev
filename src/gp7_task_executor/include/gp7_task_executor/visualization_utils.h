#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "moveit/move_group_interface/move_group_interface.h"

namespace gp7_task_executor
{

/**
 * Visualization utilities for publishing planned motion paths.
 *
 * Converts a RobotTrajectory into TCP/tool0 poses via forward kinematics,
 * then publishes three simultaneous visualizations:
 *
 *   1. nav_msgs/Path         — full smooth TCP path on /planned_tool_path
 *   2. MarkerArray (SPHERE_LIST) — discrete waypoint spheres on /planned_waypoints_marker
 *   3. Marker (SPHERE, x2)  — green start + red goal on /planned_waypoints_marker
 *   4. Marker (LINE_STRIP)   — golden path line on /planned_path_line_marker
 *
 * All markers are published with action OVERWRITE so stale paths are replaced.
 * Frame is resolved at publish time: tries viz_frame, falls back to planning_frame.
 */
class TransformUtils;

class VisualizationUtils
{
public:
  VisualizationUtils() = default;

  void init(
      std::shared_ptr<rclcpp::Node> node,
      std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
      const std::string& viz_frame,
      TransformUtils* transform_utils);

  /**
   * Publish all visualizations for a planned trajectory.
   * Safe to call even if trajectory is empty (no-op with a warning log).
   *
   * @param trajectory  The planned RobotTrajectory to visualize
   * @param service_name  Descriptive name of the service that produced this plan
   */
  void publish_plan_visualization(
      const moveit_msgs::msg::RobotTrajectory& trajectory,
      const std::string& service_name);

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::string viz_frame_;
  TransformUtils* transform_utils_ = nullptr;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_tool_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_waypoint_markers_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_path_line_marker_;

  rclcpp::Logger logger() const { return node_->get_logger(); }

  std::string resolve_viz_frame(const std::string& planning_frame);
};

}  // namespace gp7_task_executor
