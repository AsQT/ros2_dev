#pragma once

#include <limits>
#include <memory>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit/robot_state/robot_state.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "moveit_visual_tools/moveit_visual_tools.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "robot_task_executor/executor_experiment_logger.hpp"

namespace robot_task_manager
{

// Optional out-param for moveToPose(), filled in whenever a caller wants to
// build an action_metrics_logger.hpp CSV row without MoveItExecutor itself
// depending on that (higher-level) header. Left untouched (all fields keep
// their NaN/empty defaults) when moveToPose() is called without it — this
// is strictly additive so existing call sites need no changes.
struct MoveItPlanMetrics
{
  bool has_plan = false;
  double planning_time_s = std::numeric_limits<double>::quiet_NaN();
  double execution_time_s = std::numeric_limits<double>::quiet_NaN();
  int32_t moveit_error_code = 0;
  std::string planning_group;
  std::string planner_id;
  double allowed_planning_time_s = std::numeric_limits<double>::quiet_NaN();
  int32_t num_planning_attempts = -1;
  std::vector<geometry_msgs::msg::Point> tcp_waypoints;
};

class MoveItExecutor
{
public:
  using FeedbackCallback = std::function<void(const std::string &, float)>;

  MoveItExecutor() = default;

  void initialize(
    const rclcpp::Node::SharedPtr & node,
    const std::string & planning_group,
    const std::string & base_frame = "base_link");

  // Attaches the shared executor CSV logger (same format/writer as
  // task_executor_node's /move_cartesian_pose_sequence logging) to this
  // executor. action_name is the label written to the CSV action_name
  // column for every call made through this instance (e.g. "/gohome").
  // No-op (logging stays disabled) when enable=false.
  void initializeLogging(
    bool enable,
    const std::string & log_dir,
    double sample_rate_hz,
    const std::string & base_frame,
    const std::string & tcp_frame,
    const std::string & action_name);

  // Exposes the underlying CSV logger so the owning action server can also
  // write action-level lifecycle rows (e.g. action_goal_rejected) for events
  // that happen in handle_goal(), before execute()/MoveItExecutor is ever
  // invoked. Returns nullptr if logging is disabled.
  std::shared_ptr<robot_task_executor::ExecutorExperimentLogger> getLogger() const
  {
    return executor_logger_;
  }

  bool goNamedTarget(
    const std::string & target_name,
    std::string & error_msg,
    double velocity_scale = 0.1,
    double acceleration_scale = 0.3,
    double planning_time = 5.0,
    bool execute = true);

  bool moveToPose(
          const geometry_msgs::msg::Pose & target_pose,
          std::string & error_msg,
          double velocity_scale = 0.1,
          double acceleration_scale = 0.3,
          double planning_time = 5.0,
          bool execute = true,
          MoveItPlanMetrics * out_metrics = nullptr);
  bool moveToPoseCartesian(
          const geometry_msgs::msg::Pose & target_pose,
          std::string & error_msg,
          double velocity_scale = 0.1,
          double acceleration_scale = 0.3,
          double planning_time = 5.0,
          bool execute = true);
  
  bool checkerBoard(
          double step,
          std::string & error_msg,
          double velocity_scale = 0.1,
          double acceleration_scale = 0.3,
          double planning_time = 5.0,
          bool execute = true,
          double measurement_settle_time_s = 2.0,
          FeedbackCallback feedback_cb = nullptr);

  void stop();

  void publishText(const std::string & text);
  void publishTargetAxis(const geometry_msgs::msg::Pose & pose, const std::string & label = "goal");
  void deleteAllMarkers();

  bool applyCollisionObjects(
    const std::vector<moveit_msgs::msg::CollisionObject> & objects,
    std::string & error_msg);

  bool removeCollisionObjects(
    const std::vector<std::string> & object_ids,
    std::string & error_msg);

  std::string getPlanningFrame() const;
  std::string getEndEffectorLink() const;
  std::string getPlanningGroup() const;

private:
  bool validateScalingFactors(
    double velocity_scale,
    double acceleration_scale,
    std::string & error_msg) const;

  moveit::core::RobotStatePtr getCurrentStateForPlanning(
    double timeout_sec,
    std::string & error_msg);

  bool executeCartesianSegment(
    const geometry_msgs::msg::Pose & target_pose,
    std::string & error_msg,
    double velocity_scale,
    double acceleration_scale,
    double planning_time,
    bool execute,
    const geometry_msgs::msg::Pose * planned_start_pose = nullptr);

  uint64_t startExecutorLog(const std::string & execute_mode);
  void finishExecutorLog(
    uint64_t call_id,
    const std::string & status,
    bool success,
    const std::string & message,
    double fraction = 0.0);
  void logRefWaypoint(uint64_t call_id, const geometry_msgs::msg::Pose & pose);
  void logJointCommand(
    uint64_t call_id,
    const moveit_msgs::msg::RobotTrajectory & trajectory);
  bool executeWithLogging(
    uint64_t call_id,
    const std::string & execute_mode,
    const moveit::planning_interface::MoveGroupInterface::Plan & plan,
    const std::vector<geometry_msgs::msg::Pose> & refs);

  // Best-effort: forward-kinematics the planned joint trajectory into a
  // list of Cartesian end-effector positions. Returns an empty vector (and
  // never throws out of this function) if anything about the trajectory or
  // robot model is unusable — callers must treat an empty result as
  // "trajectory waypoints not available" rather than an error.
  std::vector<geometry_msgs::msg::Point> extractTcpWaypoints(
    const moveit_msgs::msg::RobotTrajectory & trajectory);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

  std::string planning_group_;
  std::string base_frame_;
  bool initialized_{false};
  mutable std::mutex motion_mutex_;

  std::shared_ptr<tf2_ros::Buffer> log_tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> log_tf_listener_;
  std::shared_ptr<robot_task_executor::ExecutorExperimentLogger> executor_logger_;
  std::string log_action_name_;
};

}  // namespace robot_task_manager
