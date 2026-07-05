#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_drl_executor/executor_experiment_logger.hpp"
#include "robot_task_executor_msgs/srv/move_cartesian_pose_sequence.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

struct PlanResult
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  bool success = false;
  int error_code = 0;
  double fraction = 0.0;
};

bool is_finite_pose_position(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) &&
         std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z);
}

double quaternion_norm(const geometry_msgs::msg::Quaternion & q)
{
  return std::sqrt(
    q.x * q.x +
    q.y * q.y +
    q.z * q.z +
    q.w * q.w);
}

geometry_msgs::msg::Quaternion default_cartesian_quaternion()
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.7071068;
  q.y = 0.7071068;
  q.z = 0.0;
  q.w = 0.0;
  return q;
}

}  // namespace

class RobotDrlExecutorNode : public rclcpp::Node
{
public:
  RobotDrlExecutorNode()
  : Node("robot_drl_executor_node")
  {
    init_parameters();
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    create_services();

    RCLCPP_INFO(get_logger(), "robot_drl_executor_node constructed.");
    RCLCPP_INFO(get_logger(), "  move_group:         '%s'", move_group_name_.c_str());
    RCLCPP_INFO(get_logger(), "  base_frame:         '%s'", base_frame_.c_str());
    RCLCPP_INFO(get_logger(), "  ee_link:            '%s'", ee_link_.c_str());
    RCLCPP_INFO(get_logger(), "  service:            '%s'", pose_sequence_service_name_.c_str());
    RCLCPP_INFO(get_logger(), "  planning_time:      %.2f s", planning_time_);
    RCLCPP_INFO(get_logger(), "  planning_attempts:  %d", num_planning_attempts_);
    RCLCPP_INFO(get_logger(), "  velocity_scale:     %.2f", max_velocity_scaling_factor_);
    RCLCPP_INFO(get_logger(), "  acceleration_scale: %.2f", max_acceleration_scaling_factor_);
    RCLCPP_INFO(get_logger(), "Call init_move_group() after shared_ptr creation.");
  }

  void init_move_group()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), move_group_name_);

    move_group_->setPoseReferenceFrame(base_frame_);
    move_group_->setEndEffectorLink(ee_link_);
    move_group_->setPlanningTime(planning_time_);
    move_group_->setNumPlanningAttempts(num_planning_attempts_);
    move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor_);
    move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor_);

    const auto raw_planning_frame = move_group_->getPlanningFrame();
    const auto raw_pose_ref_frame = move_group_->getPoseReferenceFrame();
    const auto raw_ee_link = move_group_->getEndEffectorLink();

    RCLCPP_INFO(get_logger(), "MoveGroupInterface ready for DRL executor:");
    RCLCPP_INFO(get_logger(), "  Planning frame:       '%s'", raw_planning_frame.c_str());
    RCLCPP_INFO(get_logger(), "  Pose reference frame: '%s'", raw_pose_ref_frame.c_str());
    RCLCPP_INFO(get_logger(), "  EE link:              '%s'", raw_ee_link.c_str());
    init_executor_logger();
  }

private:
  void init_parameters()
  {
    declare_parameter<std::string>("move_group_name", "arm");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("ee_link", "tcp_link");
    declare_parameter<std::string>(
      "cartesian_pose_sequence_service_name", "/move_cartesian_pose_sequence");
    declare_parameter<double>("planning_time", 2.0);
    declare_parameter<int>("num_planning_attempts", 5);
    declare_parameter<double>("max_velocity_scaling_factor", 0.1);
    declare_parameter<double>("max_acceleration_scaling_factor", 0.5);
    declare_parameter<double>("cartesian_eef_step", 0.01);
    declare_parameter<double>("cartesian_jump_threshold", 0.0);
    declare_parameter<double>("cartesian_success_threshold", 0.95);
    declare_parameter<bool>("enable_executor_logging", false);
    declare_parameter<std::string>("log_root_dir", "/home/minhquang/ros2_dev/Log_robot_data");
    declare_parameter<std::string>(
      "executor_log_dir", "/home/minhquang/ros2_dev/Log_robot_data/mock/rl/drl_executor_internal");
    declare_parameter<double>("executor_sample_rate_hz", 50.0);
    declare_parameter<std::string>("executor_base_frame", "base_link");
    declare_parameter<std::string>("executor_tcp_frame", "tcp_link");

    get_parameter("move_group_name", move_group_name_);
    get_parameter("base_frame", base_frame_);
    get_parameter("ee_link", ee_link_);
    get_parameter("cartesian_pose_sequence_service_name", pose_sequence_service_name_);
    get_parameter("planning_time", planning_time_);
    get_parameter("num_planning_attempts", num_planning_attempts_);
    get_parameter("max_velocity_scaling_factor", max_velocity_scaling_factor_);
    get_parameter("max_acceleration_scaling_factor", max_acceleration_scaling_factor_);
    get_parameter("cartesian_eef_step", cartesian_eef_step_);
    get_parameter("cartesian_jump_threshold", cartesian_jump_threshold_);
    get_parameter("cartesian_success_threshold", cartesian_success_threshold_);
    get_parameter("enable_executor_logging", enable_executor_logging_);
    get_parameter("executor_log_dir", executor_log_dir_);
    get_parameter("executor_sample_rate_hz", executor_sample_rate_hz_);
    get_parameter("executor_base_frame", executor_base_frame_);
    get_parameter("executor_tcp_frame", executor_tcp_frame_);
  }

  void init_executor_logger()
  {
    if (!enable_executor_logging_) {
      RCLCPP_INFO(get_logger(), "Executor experiment logging disabled.");
      return;
    }

    try {
      executor_logger_ = std::make_shared<robot_task_executor::ExecutorExperimentLogger>(
        shared_from_this(),
        tf_buffer_,
        executor_log_dir_,
        executor_sample_rate_hz_,
        executor_base_frame_,
        executor_tcp_frame_);
    } catch (const std::exception & ex) {
      executor_logger_.reset();
      RCLCPP_WARN(get_logger(), "Executor experiment logger unavailable: %s", ex.what());
    }
  }

  void create_services()
  {
    service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    pose_sequence_service_ = create_service<
      robot_task_executor_msgs::srv::MoveCartesianPoseSequence>(
      pose_sequence_service_name_,
      [this](
        const std::shared_ptr<
          robot_task_executor_msgs::srv::MoveCartesianPoseSequence::Request> request,
        const std::shared_ptr<
          robot_task_executor_msgs::srv::MoveCartesianPoseSequence::Response> response)
      {
        handle_cartesian_pose_sequence(request, response);
      },
      rmw_qos_profile_services_default,
      service_callback_group_);

    // Stop service (codex.md section 7.5/7.6): halt the in-flight cartesian
    // trajectory so an RL action-wrapper's Stop actually stops the robot.
    // Same Reentrant group so it can run while the sequence handler blocks.
    stop_service_ = create_service<std_srvs::srv::Trigger>(
      "/move_cartesian_stop",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        if (!move_group_) {
          response->success = false;
          response->message = "move_group not initialized";
          return;
        }
        RCLCPP_WARN(get_logger(), "/move_cartesian_stop: stopping active trajectory");
        move_group_->stop();
        response->success = true;
        response->message = "move_group stop requested";
      },
      rmw_qos_profile_services_default,
      service_callback_group_);

    RCLCPP_INFO(
      get_logger(),
      "DRL executor service ready: %s, /move_cartesian_stop",
      pose_sequence_service_name_.c_str());
  }

  uint64_t start_executor_log(
    const std::string & action_name,
    const std::string & execute_mode,
    const std::string & note = "")
  {
    if (!executor_logger_ || !executor_logger_->enabled()) {
      return 0;
    }
    return executor_logger_->start_call(action_name, execute_mode, note);
  }

  void finish_executor_log(
    const uint64_t action_call_id,
    const std::string & status,
    const bool success,
    const std::string & message,
    const double fraction,
    const std::string & note = "")
  {
    if (executor_logger_ && executor_logger_->enabled() && action_call_id != 0) {
      executor_logger_->log_summary(action_call_id, status, success, message, fraction, note);
    }
  }

  void log_ref_waypoint(
    const uint64_t action_call_id,
    const size_t index,
    const geometry_msgs::msg::Pose & pose,
    const rclcpp::Time & stamp,
    const std::string & note = "")
  {
    if (executor_logger_ && executor_logger_->enabled() && action_call_id != 0) {
      executor_logger_->log_ref_waypoint(action_call_id, index, pose, stamp, note);
    }
  }

  void log_joint_command(
    const uint64_t action_call_id,
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    const std::string & note)
  {
    if (executor_logger_ && executor_logger_->enabled() && action_call_id != 0) {
      executor_logger_->log_joint_command(action_call_id, trajectory, note);
    }
  }

  void start_sampling(
    const uint64_t action_call_id,
    const std::string & execute_mode,
    const std::vector<geometry_msgs::msg::Pose> & refs)
  {
    if (executor_logger_ && executor_logger_->enabled() && action_call_id != 0) {
      executor_logger_->start_sampling(action_call_id, execute_mode, refs);
    }
  }

  void stop_sampling(const uint64_t action_call_id)
  {
    if (executor_logger_ && executor_logger_->enabled() && action_call_id != 0) {
      executor_logger_->stop_sampling(action_call_id);
    }
  }

  void handle_cartesian_pose_sequence(
    const std::shared_ptr<
      robot_task_executor_msgs::srv::MoveCartesianPoseSequence::Request> request,
    const std::shared_ptr<
      robot_task_executor_msgs::srv::MoveCartesianPoseSequence::Response> response)
  {
    const auto & poses = request->poses;
    const bool execute = request->execute;
    const uint64_t action_call_id = start_executor_log(
      pose_sequence_service_name_,
      execute ? "cartesian" : "plan_only",
      "callback_start");

    RCLCPP_INFO(
      get_logger(),
      "%s: %zu poses, execute=%d",
      pose_sequence_service_name_.c_str(),
      poses.size(),
      execute);

    if (poses.empty()) {
      fail_response(response, "poses must not be empty", 0.0);
      finish_executor_log(action_call_id, "failed", false, response->message, response->fraction);
      return;
    }

    if (!move_group_) {
      fail_response(response, "MoveGroup not initialized. Call init_move_group() first.", 0.0);
      finish_executor_log(action_call_id, "failed", false, response->message, response->fraction);
      return;
    }

    std::vector<geometry_msgs::msg::Pose> waypoints;
    if (!normalize_request_poses(poses, waypoints, response, action_call_id)) {
      finish_executor_log(action_call_id, "failed", false, response->message, response->fraction);
      return;
    }

    const auto cartesian = plan_cartesian_from_poses(waypoints);
    response->fraction = cartesian.fraction;

    RCLCPP_INFO(
      get_logger(),
      "[%s] computeCartesianPath fraction=%.6f trajectory_points=%zu threshold=%.2f success=%d",
      pose_sequence_service_name_.c_str(),
      response->fraction,
      cartesian.trajectory.joint_trajectory.points.size(),
      cartesian_success_threshold_,
      cartesian.success);

    if (!cartesian.success) {
      RCLCPP_WARN(
        get_logger(),
        "[%s] Cartesian path fraction %.6f below threshold %.2f; trying PTP fallback.",
        pose_sequence_service_name_.c_str(),
        response->fraction,
        cartesian_success_threshold_);

      const auto fallback = execute_pose_waypoints_ptp(waypoints, execute, action_call_id);
      if (!fallback.success) {
        response->success = false;
        response->message =
          "Cartesian path fraction below threshold: " + std::to_string(response->fraction) +
          "; PTP fallback failed at fraction: " + std::to_string(fallback.fraction);
        RCLCPP_ERROR(get_logger(), "[%s] %s", pose_sequence_service_name_.c_str(),
          response->message.c_str());
        finish_executor_log(
          action_call_id,
          "failed",
          false,
          response->message,
          response->fraction,
          "ptp_fallback_failed");
        return;
      }

      response->success = true;
      response->fraction = fallback.fraction;
      response->message = execute
        ? "Cartesian path was partial; PTP waypoint fallback executed successfully"
        : "Cartesian path was partial; PTP waypoint fallback planned successfully";
      RCLCPP_INFO(get_logger(), "[%s] %s", pose_sequence_service_name_.c_str(),
        response->message.c_str());
      finish_executor_log(
        action_call_id,
        "completed",
        response->success,
        response->message,
        response->fraction,
        "ptp_fallback");
      return;
    }

    if (execute) {
      const bool timing_ok = trajectory_has_strictly_increasing_time(cartesian.trajectory);
      bool exec_ok = false;
      if (timing_ok) {
        log_joint_command(action_call_id, cartesian.trajectory, "robot_drl_executor cartesian");
        start_sampling(action_call_id, "cartesian", waypoints);
        exec_ok = execute_trajectory(cartesian.trajectory);
        stop_sampling(action_call_id);
      } else {
        RCLCPP_WARN(
          get_logger(),
          "[%s] Cartesian trajectory has non-increasing timing; trying PTP fallback.",
          pose_sequence_service_name_.c_str());
      }

      if (exec_ok) {
        response->success = true;
        response->message =
          "Cartesian pose sequence executed successfully (fraction=" +
          std::to_string(response->fraction) + ")";
        RCLCPP_INFO(get_logger(), "[%s] %s", pose_sequence_service_name_.c_str(),
          response->message.c_str());
        finish_executor_log(action_call_id, "completed", true, response->message, response->fraction);
        return;
      }

      RCLCPP_WARN(
        get_logger(),
        "[%s] Cartesian execution failed or was not executable; trying PTP fallback.",
        pose_sequence_service_name_.c_str());
      const auto fallback = execute_pose_waypoints_ptp(waypoints, true, action_call_id);
      response->success = fallback.success;
      response->fraction = fallback.fraction;
      response->message = fallback.success
        ? "Cartesian execution fallback executed PTP waypoints successfully"
        : "Cartesian pose sequence planned (fraction=" +
          std::to_string(cartesian.fraction) + ") but execution and PTP fallback failed";

      if (fallback.success) {
        RCLCPP_INFO(get_logger(), "[%s] %s", pose_sequence_service_name_.c_str(),
          response->message.c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "[%s] %s", pose_sequence_service_name_.c_str(),
          response->message.c_str());
      }
      finish_executor_log(
        action_call_id,
        response->success ? "completed" : "failed",
        response->success,
        response->message,
        response->fraction,
        "cartesian_execute_ptp_fallback");
      return;
    }

    log_joint_command(action_call_id, cartesian.trajectory, "robot_drl_executor plan_only");
    response->success = true;
    response->message =
      "Cartesian pose sequence planned successfully (fraction=" +
      std::to_string(response->fraction) + "), plan-only";
    RCLCPP_INFO(get_logger(), "[%s] %s", pose_sequence_service_name_.c_str(),
      response->message.c_str());
    finish_executor_log(action_call_id, "completed", true, response->message, response->fraction);
  }

  bool normalize_request_poses(
    const std::vector<geometry_msgs::msg::PoseStamped> & poses,
    std::vector<geometry_msgs::msg::Pose> & waypoints,
    const std::shared_ptr<
      robot_task_executor_msgs::srv::MoveCartesianPoseSequence::Response> response,
    const uint64_t action_call_id)
  {
    const auto fallback_quat = default_cartesian_quaternion();

    try {
      const auto current_eef = move_group_->getCurrentPose().pose;
      RCLCPP_INFO(
        get_logger(),
        "[%s] Current tcp_link xyz=(%.4f, %.4f, %.4f) quat=(%.6f, %.6f, %.6f, %.6f)",
        pose_sequence_service_name_.c_str(),
        current_eef.position.x,
        current_eef.position.y,
        current_eef.position.z,
        current_eef.orientation.x,
        current_eef.orientation.y,
        current_eef.orientation.z,
        current_eef.orientation.w);
    } catch (const std::exception & ex) {
      RCLCPP_WARN(
        get_logger(),
        "[%s] Could not get current tcp_link pose: %s",
        pose_sequence_service_name_.c_str(),
        ex.what());
    }

    waypoints.reserve(poses.size());
    for (size_t i = 0; i < poses.size(); ++i) {
      const auto & pose_stamped = poses[i];
      std::string frame_id = pose_stamped.header.frame_id;
      if (frame_id.empty()) {
        RCLCPP_WARN(
          get_logger(),
          "[%s] pose[%zu]: frame_id is empty, defaulting to '%s'",
          pose_sequence_service_name_.c_str(),
          i,
          base_frame_.c_str());
        frame_id = base_frame_;
      }

      if (frame_id != base_frame_) {
        fail_response(
          response,
          "Unsupported frame_id '" + frame_id + "': only '" + base_frame_ +
          "' is supported. TF transform to planning frame is not implemented in robot_drl_executor.",
          0.0);
        return false;
      }

      if (!is_finite_pose_position(pose_stamped.pose)) {
        fail_response(
          response,
          "Pose[" + std::to_string(i) + "] has non-finite position: (" +
          std::to_string(pose_stamped.pose.position.x) + ", " +
          std::to_string(pose_stamped.pose.position.y) + ", " +
          std::to_string(pose_stamped.pose.position.z) + ")",
          0.0);
        return false;
      }

      geometry_msgs::msg::Pose waypoint_pose = pose_stamped.pose;
      const bool replaced_quaternion = quaternion_norm(waypoint_pose.orientation) < 1e-9;
      if (replaced_quaternion) {
        RCLCPP_WARN(
          get_logger(),
          "[%s] pose[%zu]: quaternion has zero norm, replacing with fallback quat=(%.6f, %.6f, %.6f, %.6f)",
          pose_sequence_service_name_.c_str(),
          i,
          fallback_quat.x,
          fallback_quat.y,
          fallback_quat.z,
          fallback_quat.w);
        waypoint_pose.orientation = fallback_quat;
      }

      waypoints.push_back(waypoint_pose);
      log_ref_waypoint(
        action_call_id,
        i,
        waypoint_pose,
        rclcpp::Time(pose_stamped.header.stamp),
        replaced_quaternion ? "zero_quaternion_replaced_with_default" : "");
      log_pose(i, waypoints.size(), waypoint_pose, frame_id);
    }

    return true;
  }

  void log_pose(
    const size_t index,
    const size_t total_seen,
    const geometry_msgs::msg::Pose & pose,
    const std::string & frame_id) const
  {
    const bool is_first = index == 0;
    const bool is_last_seen = total_seen > 1;
    if (!is_first && !is_last_seen) {
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "[%s] %s pose[%zu] xyz=(%.4f, %.4f, %.4f) quat=(%.6f, %.6f, %.6f, %.6f) frame='%s'",
      pose_sequence_service_name_.c_str(),
      is_first ? "FIRST" : "POSE",
      index,
      pose.position.x,
      pose.position.y,
      pose.position.z,
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w,
      frame_id.c_str());
  }

  PlanResult plan_cartesian_from_poses(const std::vector<geometry_msgs::msg::Pose> & waypoints)
  {
    PlanResult result;

    move_group_->setStartStateToCurrentState();
    move_group_->setPoseReferenceFrame(base_frame_);

    RCLCPP_INFO(
      get_logger(),
      "[Cartesian] %s: %zu waypoints, eef_step=%.4f, jump_threshold=%.4f",
      pose_sequence_service_name_.c_str(),
      waypoints.size(),
      cartesian_eef_step_,
      cartesian_jump_threshold_);

    result.fraction = move_group_->computeCartesianPath(
      waypoints,
      cartesian_eef_step_,
      cartesian_jump_threshold_,
      result.trajectory,
      true,
      nullptr);
    result.success = result.fraction >= cartesian_success_threshold_;
    return result;
  }

  PlanResult execute_pose_waypoints_ptp(
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    const bool execute,
    const uint64_t action_call_id)
  {
    PlanResult result;
    if (waypoints.empty()) {
      return result;
    }

    const auto ptp_waypoints = collapse_degenerate_loop_to_final_pose(waypoints);

    const std::string ee_link = move_group_->getEndEffectorLink();
    move_group_->setPoseReferenceFrame(base_frame_);
    move_group_->setPlanningTime(planning_time_);
    move_group_->setNumPlanningAttempts(num_planning_attempts_);
    move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor_);
    move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor_);

    size_t completed = 0;
    for (size_t i = 0; i < ptp_waypoints.size(); ++i) {
      move_group_->setStartStateToCurrentState();
      move_group_->clearPoseTargets();

      bool using_position_only = false;
      moveit::planning_interface::MoveGroupInterface::Plan plan;

      auto plan_position_only = [&]() -> moveit::core::MoveItErrorCode {
          move_group_->setStartStateToCurrentState();
          move_group_->clearPoseTargets();
          const auto & pos = ptp_waypoints[i].position;
          RCLCPP_WARN(
            get_logger(),
            "[PTP] %s: trying position-only fallback at waypoint %zu/%zu xyz=(%.4f, %.4f, %.4f)",
            pose_sequence_service_name_.c_str(),
            i,
            ptp_waypoints.size(),
            pos.x,
            pos.y,
            pos.z);
          if (!move_group_->setPositionTarget(pos.x, pos.y, pos.z, ee_link)) {
            return moveit::core::MoveItErrorCode(moveit_msgs::msg::MoveItErrorCodes::FAILURE);
          }
          using_position_only = true;
          return move_group_->plan(plan);
        };

      auto plan_result = moveit::core::MoveItErrorCode(moveit_msgs::msg::MoveItErrorCodes::FAILURE);
      if (move_group_->setPoseTarget(ptp_waypoints[i], ee_link)) {
        plan_result = move_group_->plan(plan);
        if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_WARN(
            get_logger(),
            "[PTP] %s: pose plan failed at waypoint %zu/%zu; trying position-only.",
            pose_sequence_service_name_.c_str(),
            i,
            ptp_waypoints.size());
          plan_result = plan_position_only();
        }
      } else {
        plan_result = plan_position_only();
      }

      if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
        result.error_code = plan_result.val;
        result.fraction = static_cast<double>(completed) / static_cast<double>(ptp_waypoints.size());
        move_group_->clearPoseTargets();
        return result;
      }

      if (execute) {
        log_joint_command(
          action_call_id,
          plan.trajectory,
          "robot_drl_executor ptp_fallback waypoint=" + std::to_string(i));
        start_sampling(action_call_id, "ptp_fallback", ptp_waypoints);
        const auto exec_result = move_group_->execute(plan.trajectory);
        stop_sampling(action_call_id);
        if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
          result.error_code = exec_result.val;
          result.fraction = static_cast<double>(completed) / static_cast<double>(ptp_waypoints.size());
          move_group_->clearPoseTargets();
          return result;
        }
      } else {
        log_joint_command(
          action_call_id,
          plan.trajectory,
          "robot_drl_executor ptp_fallback plan_only waypoint=" + std::to_string(i));
      }

      ++completed;
      RCLCPP_INFO(
        get_logger(),
        "[PTP] %s: completed waypoint %zu/%zu%s.",
        pose_sequence_service_name_.c_str(),
        completed,
        ptp_waypoints.size(),
        using_position_only ? " (position-only)" : "");
    }

    move_group_->clearPoseTargets();
    result.success = true;
    result.fraction = 1.0;
    return result;
  }

  std::vector<geometry_msgs::msg::Pose> collapse_degenerate_loop_to_final_pose(
    const std::vector<geometry_msgs::msg::Pose> & waypoints) const
  {
    if (waypoints.size() < 3) {
      return waypoints;
    }

    const auto & first = waypoints.front().position;
    const auto & last = waypoints.back().position;
    const double dx = first.x - last.x;
    const double dy = first.y - last.y;
    const double dz = first.z - last.z;
    const double first_last_distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (first_last_distance > 1e-4) {
      return waypoints;
    }

    RCLCPP_WARN(
      get_logger(),
      "[%s] PTP fallback received a degenerate loop whose first and last poses match within %.6f m; executing final pose only.",
      pose_sequence_service_name_.c_str(),
      first_last_distance);
    return {waypoints.back()};
  }

  bool execute_trajectory(const moveit_msgs::msg::RobotTrajectory & trajectory)
  {
    move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor_);
    move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor_);

    auto trajectory_to_execute = trajectory;
    auto & joint_trajectory = trajectory_to_execute.joint_trajectory;
    if (!joint_trajectory.points.empty() && !joint_trajectory.joint_names.empty()) {
      move_group_->setStartStateToCurrentState();
      const auto current_state = move_group_->getCurrentState(1.0);
      if (current_state) {
        auto & first_point = joint_trajectory.points.front();
        if (first_point.positions.size() == joint_trajectory.joint_names.size()) {
          double max_abs_delta = 0.0;
          std::string max_delta_joint;
          for (size_t i = 0; i < joint_trajectory.joint_names.size(); ++i) {
            const auto & joint_name = joint_trajectory.joint_names[i];
            const double current_position = current_state->getVariablePosition(joint_name);
            const double delta = std::abs(first_point.positions[i] - current_position);
            if (delta > max_abs_delta) {
              max_abs_delta = delta;
              max_delta_joint = joint_name;
            }
            first_point.positions[i] = current_position;
          }
          RCLCPP_INFO(
            get_logger(),
            "[Executor] Synced trajectory start to current robot state before execute (max_delta=%.6f rad at '%s').",
            max_abs_delta,
            max_delta_joint.c_str());
        }
      } else {
        RCLCPP_WARN(
          get_logger(),
          "[Executor] Current robot state unavailable; executing trajectory as planned.");
      }
    }

    const auto exec_result = move_group_->execute(trajectory_to_execute);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "[Executor] execute() failed (error code: %d)", exec_result.val);
      return false;
    }

    RCLCPP_INFO(get_logger(), "[Executor] Trajectory executed successfully.");
    return true;
  }

  bool trajectory_has_strictly_increasing_time(
    const moveit_msgs::msg::RobotTrajectory & trajectory) const
  {
    const auto & points = trajectory.joint_trajectory.points;
    if (points.empty()) {
      return false;
    }

    int64_t previous_ns = -1;
    for (size_t i = 0; i < points.size(); ++i) {
      const auto & t = points[i].time_from_start;
      const int64_t current_ns =
        static_cast<int64_t>(t.sec) * 1000000000LL + static_cast<int64_t>(t.nanosec);
      if (i > 0 && current_ns <= previous_ns) {
        RCLCPP_WARN(
          get_logger(),
          "[%s] trajectory time is not strictly increasing at point %zu: previous=%ld ns current=%ld ns",
          pose_sequence_service_name_.c_str(),
          i,
          previous_ns,
          current_ns);
        return false;
      }
      previous_ns = current_ns;
    }
    return true;
  }

  void fail_response(
    const std::shared_ptr<
      robot_task_executor_msgs::srv::MoveCartesianPoseSequence::Response> response,
    const std::string & message,
    const double fraction) const
  {
    response->success = false;
    response->message = message;
    response->fraction = fraction;
    RCLCPP_ERROR(get_logger(), "[%s] %s", pose_sequence_service_name_.c_str(), message.c_str());
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<robot_task_executor::ExecutorExperimentLogger> executor_logger_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::Service<robot_task_executor_msgs::srv::MoveCartesianPoseSequence>::SharedPtr
    pose_sequence_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;

  std::string move_group_name_;
  std::string base_frame_;
  std::string ee_link_;
  std::string pose_sequence_service_name_;
  double planning_time_ = 2.0;
  int num_planning_attempts_ = 5;
  double max_velocity_scaling_factor_ = 0.1;
  double max_acceleration_scaling_factor_ = 0.5;
  double cartesian_eef_step_ = 0.01;
  double cartesian_jump_threshold_ = 0.0;
  double cartesian_success_threshold_ = 0.95;
  bool enable_executor_logging_ = false;
  std::string executor_log_dir_;
  double executor_sample_rate_hz_ = 50.0;
  std::string executor_base_frame_;
  std::string executor_tcp_frame_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RobotDrlExecutorNode>();
  node->init_move_group();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
