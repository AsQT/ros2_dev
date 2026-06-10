#pragma once

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace gp7_task_executor
{

class WaypointLoader;
class TransformUtils;

struct PlanResult
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  bool success = false;
  int error_code = 0;
  double fraction = 0.0;

  explicit operator bool() const { return success; }
};

class PlannerUtils
{
public:
  PlannerUtils() = default;

  void init(
      std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
      std::shared_ptr<rclcpp::Node> node,
      double planning_time,
      int num_planning_attempts,
      double max_velocity_scaling,
      double max_acceleration_scaling,
      const WaypointLoader* waypoint_loader,
      TransformUtils* transform_utils,
      const std::string& task_frame);

  void set_task_frame(const std::string& frame);

  geometry_msgs::msg::Quaternion cartesian_quat() const { return cartesian_quat_; }
  double last_cartesian_fraction() const { return last_cartesian_fraction_; }
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> get_move_group() const { return move_group_; }

  PlanResult plan_joint_target(
      const std::string& target_name,
      const std::vector<double>& joint_positions,
      const std::vector<std::string>& joint_names,
      bool execute);

  PlanResult plan_named_pose_target(
      const std::string& target_name,
      const geometry_msgs::msg::PoseStamped& stored_pose,
      bool execute);

  PlanResult plan_cartesian_from_poses(
      const std::vector<geometry_msgs::msg::Pose>& waypoints,
      double eef_step,
      double jump_threshold,
      const std::string& service_name);

  PlanResult plan_cartesian_named_sequence(
      const std::vector<std::string>& waypoint_names);

  bool execute_trajectory(const moveit_msgs::msg::RobotTrajectory& trajectory);

  static void rpy_to_quaternion(double roll, double pitch, double yaw,
                                 double& qx, double& qy, double& qz, double& qw);

  static constexpr double DEFAULT_EEF_STEP = 0.01;
  static constexpr double DEFAULT_JUMP_THRESHOLD = 0.0;
  static constexpr double CARTESIAN_SUCCESS_THRESHOLD = 0.95;

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<rclcpp::Node> node_;
  const WaypointLoader* waypoint_loader_ = nullptr;
  TransformUtils* transform_utils_ = nullptr;
  double planning_time_ = 2.0;
  int num_planning_attempts_ = 5;
  double max_velocity_scaling_ = 0.5;
  double max_acceleration_scaling_ = 0.5;
  std::string task_frame_ = "base_link";
  double last_cartesian_fraction_ = 0.0;
  geometry_msgs::msg::Quaternion cartesian_quat_;

  rclcpp::Logger logger() const { return node_->get_logger(); }
};

}  // namespace gp7_task_executor
