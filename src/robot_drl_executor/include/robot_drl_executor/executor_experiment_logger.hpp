#pragma once

#include <array>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/buffer.h"

namespace robot_task_executor
{

// Shared CSV logger with two roles in the current architecture:
//   1. Execution-level logging (its original purpose): actual_sample /
//      joint_command / ref_waypoint / execute_start / execute_summary rows
//      for real motion, written by task_executor_node's
//      /move_cartesian_pose_sequence handler, by MoveItExecutor
//      (gohome/move_to_pose/move_to_pose_cartesian), and independently by
//      robot_drl_executor_node for its own Gazebo/real-hw demos.
//   2. Action-level lifecycle logging (added later, via
//      log_lifecycle_event()): action_goal_received / action_goal_rejected /
//      action_stage_failed / action_result rows for every robot_task_manager
//      action call, regardless of whether it ever reaches execution.
//
// This is NOT the per-call TCP logger — one instance here is shared across
// every goal handled by its owning node/process, so its CSV mixes together
// all calls made to that node. For "one CSV file per action call" logging,
// see PickPlaceTcpLogger (pickplace_server.cpp, /pickplace only) and
// PerCallTcpLogger (per_call_tcp_logger.hpp, opt-in via goal.enable_tcp_log
// on /move_to_pose, /move_to_pose_cartesian, /repeatability_test,
// /move_checker_board).
class ExecutorExperimentLogger
{
public:
  ExecutorExperimentLogger(
      const std::shared_ptr<rclcpp::Node>& node,
      const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
      const std::string& log_dir,
      double sample_rate_hz,
      const std::string& base_frame,
      const std::string& tcp_frame);
  ~ExecutorExperimentLogger();

  bool enabled() const { return enabled_; }

  uint64_t start_call(
      const std::string& action_name,
      const std::string& execute_mode,
      const std::string& note = "");

  void log_ref_waypoint(
      uint64_t action_call_id,
      size_t ref_index,
      const geometry_msgs::msg::Pose& pose,
      const rclcpp::Time& ref_time,
      const std::string& note = "");

  void log_joint_command(
      uint64_t action_call_id,
      const moveit_msgs::msg::RobotTrajectory& trajectory,
      const std::string& note = "");

  void start_sampling(
      uint64_t action_call_id,
      const std::string& execute_mode,
      const std::vector<geometry_msgs::msg::Pose>& refs);
  void stop_sampling(uint64_t action_call_id);

  void log_summary(
      uint64_t action_call_id,
      const std::string& status,
      bool success,
      const std::string& message,
      double fraction,
      const std::string& note = "");

  // Action-level lifecycle logging (goal received/accepted/rejected, stage
  // failures, final result, ...) — independent of whether the call ever
  // reaches execution. Writes into the same CSV/schema as the
  // execution-level rows above (start_call/log_summary/...), just with a
  // different row_type. When existing_call_id == 0, a new session is
  // registered (use for the first event of a call, e.g. action_goal_received)
  // and its id is returned; pass that id back in on subsequent calls for the
  // same action call (e.g. action_stage_failed, action_result).
  uint64_t log_lifecycle_event(
      const std::string& action_name,
      const std::string& row_type,
      const std::string& stage_name,
      const std::string& status,
      const std::string& message,
      const std::string& metadata_json = "",
      uint64_t existing_call_id = 0);

private:
  struct JointCommandPoint
  {
    double time_from_start_sec = 0.0;
    std::array<double, 6> positions{};
    std::array<double, 6> velocities{};
    std::array<double, 6> accelerations{};
    std::array<bool, 6> has_position{};
    std::array<bool, 6> has_velocity{};
    std::array<bool, 6> has_acceleration{};
  };

  struct Session
  {
    uint64_t action_call_id = 0;
    std::string action_name;
    std::string execute_mode;
    std::string start_iso;
    rclcpp::Time start_time;
    size_t ref_waypoints_count = 0;
    size_t joint_command_points_count = 0;
    size_t actual_samples_count = 0;
    std::vector<geometry_msgs::msg::Pose> refs;
    std::vector<JointCommandPoint> command_points;
    std::vector<double> joint_error_norms;
    std::vector<double> tcp_position_error_norms;
    double final_joint_error_norm = 0.0;
    bool has_final_joint_error_norm = false;
    double final_tcp_position_error_norm = 0.0;
    bool has_final_tcp_position_error_norm = false;
  };

  using Row = std::map<std::string, std::string>;

  bool prepare_output(const std::string& log_dir);
  void write_header();
  void write_row(const Row& row);
  Row base_row(const Session& session, const std::string& row_type) const;
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void sample_loop(uint64_t action_call_id);
  void write_actual_sample(uint64_t action_call_id);
  void stop_sampling_locked(uint64_t action_call_id);

  static std::vector<std::string> csv_header();
  static std::string csv_escape(const std::string& value);
  static std::string format_double(double value);
  static std::string format_bool(bool value);
  static std::string iso_now();
  static double elapsed_sec(const rclcpp::Time& start, const rclcpp::Time& end);
  static double duration_sec(const builtin_interfaces::msg::Duration& duration);
  static double quaternion_angle_error_roll_pitch_yaw(
      const geometry_msgs::msg::Quaternion& actual,
      const geometry_msgs::msg::Quaternion& ref,
      double& roll,
      double& pitch,
      double& yaw);
  static void quaternion_to_rpy(
      const geometry_msgs::msg::Quaternion& q,
      double& roll,
      double& pitch,
      double& yaw);
  static double mean(const std::vector<double>& values);
  static double max_value(const std::vector<double>& values);
  static double rmse(const std::vector<double>& values);
  static bool mkdirs(const std::string& path);

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Logger logger_;
  std::ofstream csv_;
  mutable std::mutex mutex_;
  std::map<uint64_t, Session> sessions_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  uint64_t next_action_call_id_ = 1;
  uint64_t active_sampling_id_ = 0;
  bool enabled_ = false;
  bool sampling_stop_requested_ = false;
  std::thread sampling_thread_;
  std::string run_id_;
  std::string csv_path_;
  std::string base_frame_;
  std::string tcp_frame_;
  double sample_rate_hz_ = 50.0;
  std::array<std::string, 6> joint_names_{
      "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
};

}  // namespace robot_task_executor
