#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace gp7_task_executor
{

/**
 * Waypoint loader — parses YAML configuration files and provides
 * named joint and cartesian target data to the executor.
 *
 * Handles two configuration formats:
 *
 *   joint_waypoints.yaml
 *     waypoints:
 *       home:
 *         joints: [0, 0, 0, 0, 90, 0]    # degrees, converted to radians on load
 *
 *   cartesian_points.yaml
 *     cartesian_points:
 *       pose_A:
 *         description: "Point A"
 *         frame_id: "world"
 *         xyz: [0.5, 0.0, 0.6]           # metres
 *
 * Errors are logged per-entry; a malformed entry is skipped but the rest
 * of the file is still loaded.
 */
class WaypointLoader
{
public:
  explicit WaypointLoader(
      rclcpp::Node* node,
      const std::string& waypoints_config_path,
      const std::string& cartesian_points_config_path,
      const std::string& pose_waypoints_config_path,
      const std::vector<std::string>& joint_names,
      geometry_msgs::msg::Quaternion cartesian_quat);

  bool load();

  static constexpr size_t EXPECTED_JOINT_COUNT = 6;

  const std::map<std::string, std::vector<double>>& named_joint_targets() const
  {
    return named_joint_targets_;
  }

  const std::map<std::string, geometry_msgs::msg::PoseStamped>& named_cartesian_points() const
  {
    return named_cartesian_points_;
  }

  const std::map<std::string, geometry_msgs::msg::PoseStamped>& named_pose_targets() const
  {
    return named_pose_targets_;
  }

  bool has_joint_target(const std::string& name) const;
  bool has_cartesian_point(const std::string& name) const;
  bool has_pose_target(const std::string& name) const;
  const std::vector<double>* get_joint_target(const std::string& name) const;
  const geometry_msgs::msg::PoseStamped* get_cartesian_point(const std::string& name) const;
  const geometry_msgs::msg::PoseStamped* get_pose_target(const std::string& name) const;

  std::string available_joint_target_names() const;
  std::string available_cartesian_point_names() const;
  std::string available_pose_target_names() const;

  std::string pose_waypoints_config_path() const { return pose_waypoints_config_path_; }

  static void rpy_to_quaternion(double roll, double pitch, double yaw,
                                double& qx, double& qy, double& qz, double& qw);

private:
  rclcpp::Node* node_;
  std::string waypoints_config_path_;
  std::string cartesian_points_config_path_;
  std::vector<std::string> joint_names_;
  geometry_msgs::msg::Quaternion cartesian_quat_;

  std::map<std::string, std::vector<double>> named_joint_targets_;
  std::map<std::string, geometry_msgs::msg::PoseStamped> named_cartesian_points_;
  std::map<std::string, geometry_msgs::msg::PoseStamped> named_pose_targets_;
  std::string pose_waypoints_config_path_;

  rclcpp::Logger logger() const { return node_->get_logger(); }

  void load_joint_targets();
  void load_cartesian_points();
  void load_pose_targets();
};

}  // namespace gp7_task_executor
