#include <memory>
#include <string>
#include <vector>
#include <map>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "robot_task_executor/executor_experiment_logger.hpp"
#include "robot_task_executor/planner_utils.h"
#include "robot_task_executor/visualization_utils.h"
#include "robot_task_executor/waypoint_loader.h"
#include "robot_task_executor/transform_utils.h"

#include "robot_task_executor_msgs/srv/move_to_named_target.hpp"
#include "robot_task_executor_msgs/srv/move_to_joint_target.hpp"
#include "robot_task_executor_msgs/srv/move_to_pose_target.hpp"
#include "robot_task_executor_msgs/srv/move_to_named_pose_target.hpp"
#include "robot_task_executor_msgs/srv/move_to_cartesian_target.hpp"
#include "robot_task_executor_msgs/srv/move_to_named_cartesian_target.hpp"
#include "robot_task_executor_msgs/srv/move_cartesian_sequence.hpp"
#include "robot_task_executor_msgs/srv/move_cartesian_pose_sequence.hpp"
#include "robot_task_executor_msgs/srv/move_sequence.hpp"

class TaskExecutorNode : public rclcpp::Node
{
public:
  TaskExecutorNode()
  : Node("task_executor_node")
  {
    init_parameters();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    service_callback_group_ = create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

    create_modules();
    create_services();

    RCLCPP_INFO(get_logger(), "Task executor node constructed.");
    RCLCPP_INFO(get_logger(), "  move_group:              '%s'", move_group_name_.c_str());
    RCLCPP_INFO(get_logger(), "  base_frame:              '%s'", base_frame_.c_str());
    RCLCPP_INFO(get_logger(), "  ee_link:                 '%s'", ee_link_.c_str());
    RCLCPP_INFO(get_logger(), "  planning_time:           %.2f s", planning_time_);
    RCLCPP_INFO(get_logger(), "  planning_attempts:       %d", num_planning_attempts_);
    RCLCPP_INFO(get_logger(), "  max_velocity_scale:      %.2f", max_velocity_scaling_factor_);
    RCLCPP_INFO(get_logger(), "  max_accel_scale:         %.2f", max_acceleration_scaling_factor_);
    RCLCPP_INFO(get_logger(), "  waypoints_config:        '%s'", waypoints_config_path_.c_str());
    RCLCPP_INFO(get_logger(), "  cartesian_points_config: '%s'", cartesian_points_config_path_.c_str());
    RCLCPP_INFO(get_logger(), "  pose_waypoints_config:   '%s'", pose_waypoints_config_path_.c_str());
    RCLCPP_INFO(get_logger(), "Call init_move_group() after shared_ptr creation to enable planning.");
  }

  void init_move_group()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), move_group_name_);

    move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor_);
    move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor_);

    const std::string raw_planning_frame = move_group_->getPlanningFrame();
    const std::string raw_pose_ref_frame = move_group_->getPoseReferenceFrame();
    const std::string raw_ee_link = move_group_->getEndEffectorLink();

    RCLCPP_INFO(get_logger(), "MoveGroupInterface raw settings for group '%s':", move_group_name_.c_str());
    RCLCPP_INFO(get_logger(), "  Planning frame:        '%s'", raw_planning_frame.c_str());
    RCLCPP_INFO(get_logger(), "  Pose reference frame:  '%s'", raw_pose_ref_frame.c_str());
    RCLCPP_INFO(get_logger(), "  EE link:               '%s'", raw_ee_link.c_str());

    move_group_->setPoseReferenceFrame(base_frame_);
    move_group_->setEndEffectorLink(ee_link_);

    const std::string enforced_pose_ref = move_group_->getPoseReferenceFrame();
    const std::string enforced_ee_link = move_group_->getEndEffectorLink();
    const std::string enforced_planning = move_group_->getPlanningFrame();

    RCLCPP_INFO(get_logger(), "MoveGroupInterface ENFORCED settings:");
    RCLCPP_INFO(get_logger(), "  Planning frame:        '%s'  (raw was '%s')",
                enforced_planning.c_str(), raw_planning_frame.c_str());
    RCLCPP_INFO(get_logger(), "  Pose reference frame:  '%s'  (raw was '%s')",
                enforced_pose_ref.c_str(), raw_pose_ref_frame.c_str());
    RCLCPP_INFO(get_logger(), "  EE link:               '%s'  (raw was '%s')",
                enforced_ee_link.c_str(), raw_ee_link.c_str());

    const std::string task_frame = (enforced_planning == "world") ? base_frame_ : enforced_planning;
    if (enforced_planning == "world")
    {
      RCLCPP_WARN(get_logger(),
                  "getPlanningFrame() returned 'world' — using base_frame_='%s' as task_frame instead.",
                  base_frame_.c_str());
    }

    planner_->init(
        move_group_,
        shared_from_this(),
        planning_time_,
        num_planning_attempts_,
        max_velocity_scaling_factor_,
        max_acceleration_scaling_factor_,
        waypoint_loader_.get(),
        transform_.get(),
        task_frame);
    init_executor_logger();

    viz_->init(shared_from_this(), move_group_, base_frame_, transform_.get());

    if (!waypoint_loader_->load())
    {
      RCLCPP_ERROR(get_logger(), "Failed to load waypoints.");
    }
  }

private:
  void init_parameters()
  {
    declare_parameter<std::string>("move_group_name", "arm");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("ee_link", "tcp_link");
    declare_parameter<std::string>("waypoints_config_path", "");
    declare_parameter<std::string>("cartesian_points_config_path", "");
    declare_parameter<std::string>("pose_waypoints_config_path", "");
    declare_parameter<double>("planning_time", 2.0);
    declare_parameter<int>("num_planning_attempts", 5);
    declare_parameter<double>("max_velocity_scaling_factor", 0.1);
    declare_parameter<double>("max_acceleration_scaling_factor", 0.5);
    declare_parameter<bool>("enable_executor_logging", false);
    declare_parameter<std::string>("log_root_dir", "/home/minhquang/ros2_dev/Log_robot_data");
    declare_parameter<std::string>(
        "executor_log_dir", "/home/minhquang/ros2_dev/Log_robot_data/mock/baseline/task_executor_internal");
    declare_parameter<double>("executor_sample_rate_hz", 50.0);
    declare_parameter<std::string>("executor_base_frame", "base_link");
    declare_parameter<std::string>("executor_tcp_frame", "tcp_link");

    get_parameter("move_group_name", move_group_name_);
    get_parameter("base_frame", base_frame_);
    get_parameter("ee_link", ee_link_);
    get_parameter("waypoints_config_path", waypoints_config_path_);
    get_parameter("cartesian_points_config_path", cartesian_points_config_path_);
    get_parameter("pose_waypoints_config_path", pose_waypoints_config_path_);
    get_parameter("planning_time", planning_time_);
    get_parameter("num_planning_attempts", num_planning_attempts_);
    get_parameter("max_velocity_scaling_factor", max_velocity_scaling_factor_);
    get_parameter("max_acceleration_scaling_factor", max_acceleration_scaling_factor_);
    get_parameter("enable_executor_logging", enable_executor_logging_);
    get_parameter("executor_log_dir", executor_log_dir_);
    get_parameter("executor_sample_rate_hz", executor_sample_rate_hz_);
    get_parameter("executor_base_frame", executor_base_frame_);
    get_parameter("executor_tcp_frame", executor_tcp_frame_);
  }

  void create_modules()
  {
    static const std::vector<std::string> joint_order = {
        "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

    geometry_msgs::msg::Quaternion cartesian_quat;
    cartesian_quat.x = 0.7071068;
    cartesian_quat.y = 0.7071068;
    cartesian_quat.z = 0.0;
    cartesian_quat.w = 0.0;

    RCLCPP_INFO(get_logger(),
                "[Node] Cartesian orientation FIXED: quat=(%.6f, %.6f, %.6f, %.6f)",
                cartesian_quat.x, cartesian_quat.y,
                cartesian_quat.z, cartesian_quat.w);

    transform_ = std::make_unique<robot_task_executor::TransformUtils>(
        tf_buffer_, this);

    waypoint_loader_ = std::make_unique<robot_task_executor::WaypointLoader>(
        this,
        waypoints_config_path_,
        cartesian_points_config_path_,
        pose_waypoints_config_path_,
        joint_order,
        cartesian_quat);

    planner_ = std::make_unique<robot_task_executor::PlannerUtils>();
    viz_ = std::make_unique<robot_task_executor::VisualizationUtils>();
  }

  void init_executor_logger()
  {
    if (!enable_executor_logging_)
    {
      RCLCPP_INFO(get_logger(), "Executor experiment logging disabled.");
      return;
    }

    try
    {
      executor_logger_ = std::make_shared<robot_task_executor::ExecutorExperimentLogger>(
          shared_from_this(),
          tf_buffer_,
          executor_log_dir_,
          executor_sample_rate_hz_,
          executor_base_frame_,
          executor_tcp_frame_);
      planner_->set_executor_logger(executor_logger_);
    }
    catch (const std::exception& ex)
    {
      executor_logger_.reset();
      RCLCPP_WARN(get_logger(), "Executor experiment logger unavailable: %s", ex.what());
    }
  }

  void create_services()
  {
    service_named_target_ = create_service<robot_task_executor_msgs::srv::MoveToNamedTarget>(
        "/move_to_named_target",
        [this](
            const std::shared_ptr<robot_task_executor_msgs::srv::MoveToNamedTarget::Request>& request,
            const std::shared_ptr<robot_task_executor_msgs::srv::MoveToNamedTarget::Response>& response)
        { this->handle_named_target(request, response); },
        rmw_qos_profile_services_default,
        service_callback_group_);

    service_joint_target_ = create_service<robot_task_executor_msgs::srv::MoveToJointTarget>(
        "/move_to_joint_target",
        [this](
            const std::shared_ptr<robot_task_executor_msgs::srv::MoveToJointTarget::Request>& request,
            const std::shared_ptr<robot_task_executor_msgs::srv::MoveToJointTarget::Response>& response)
        { this->handle_joint_target(request, response); },
        rmw_qos_profile_services_default,
        service_callback_group_);

    service_pose_target_ = create_service<robot_task_executor_msgs::srv::MoveToPoseTarget>(
        "/move_to_pose_target",
        [this](
            const std::shared_ptr<robot_task_executor_msgs::srv::MoveToPoseTarget::Request>& request,
            const std::shared_ptr<robot_task_executor_msgs::srv::MoveToPoseTarget::Response>& response)
        { this->handle_pose_target(request, response); },
        rmw_qos_profile_services_default,
        service_callback_group_);

    service_named_pose_target_ = create_service<robot_task_executor_msgs::srv::MoveToNamedPoseTarget>(
        "/move_to_named_pose_target",
        [this](
            const std::shared_ptr<robot_task_executor_msgs::srv::MoveToNamedPoseTarget::Request>& request,
            const std::shared_ptr<robot_task_executor_msgs::srv::MoveToNamedPoseTarget::Response>& response)
        { this->handle_named_pose_target(request, response); },
        rmw_qos_profile_services_default,
        service_callback_group_);

    service_cartesian_target_ = create_service<robot_task_executor_msgs::srv::MoveToCartesianTarget>(
        "/move_to_cartesian_target",
        [this](
            const std::shared_ptr<robot_task_executor_msgs::srv::MoveToCartesianTarget::Request>& request,
            const std::shared_ptr<robot_task_executor_msgs::srv::MoveToCartesianTarget::Response>& response)
        { this->handle_cartesian_target(request, response); },
        rmw_qos_profile_services_default,
        service_callback_group_);

    service_named_cartesian_target_ = create_service<robot_task_executor_msgs::srv::MoveToNamedCartesianTarget>(
        "/move_to_named_cartesian_target",
        [this](
            const std::shared_ptr<robot_task_executor_msgs::srv::MoveToNamedCartesianTarget::Request>& request,
            const std::shared_ptr<robot_task_executor_msgs::srv::MoveToNamedCartesianTarget::Response>& response)
        { this->handle_named_cartesian_target(request, response); },
        rmw_qos_profile_services_default,
        service_callback_group_);

    service_cartesian_sequence_ = create_service<robot_task_executor_msgs::srv::MoveCartesianSequence>(
        "/move_cartesian_sequence",
        [this](
            const std::shared_ptr<robot_task_executor_msgs::srv::MoveCartesianSequence::Request>& request,
            const std::shared_ptr<robot_task_executor_msgs::srv::MoveCartesianSequence::Response>& response)
        { this->handle_cartesian_sequence(request, response); },
        rmw_qos_profile_services_default,
        service_callback_group_);

    service_move_sequence_ = create_service<robot_task_executor_msgs::srv::MoveSequence>(
        "/move_sequence",
        [this](
            const std::shared_ptr<robot_task_executor_msgs::srv::MoveSequence::Request>& request,
            const std::shared_ptr<robot_task_executor_msgs::srv::MoveSequence::Response>& response)
        { this->handle_move_sequence(request, response); },
        rmw_qos_profile_services_default,
        service_callback_group_);

    service_cartesian_pose_sequence_ = create_service<robot_task_executor_msgs::srv::MoveCartesianPoseSequence>(
        "/move_cartesian_pose_sequence",
        [this](
            const std::shared_ptr<robot_task_executor_msgs::srv::MoveCartesianPoseSequence::Request>& request,
            const std::shared_ptr<robot_task_executor_msgs::srv::MoveCartesianPoseSequence::Response>& response)
        { this->handle_cartesian_pose_sequence(request, response); },
        rmw_qos_profile_services_default,
        service_callback_group_);

    // Stop service (codex.md section 7.5/7.6): lets an action wrapper (e.g.
    // MovePoseRL/MoveTargetRL, whose real robot motion runs here via
    // /move_cartesian_pose_sequence) actually halt the in-flight trajectory.
    // Runs in the Reentrant service_callback_group_ so it executes
    // concurrently with the blocking sequence handler and can interrupt it.
    service_cartesian_stop_ = create_service<std_srvs::srv::Trigger>(
        "/move_cartesian_stop",
        [this](
            const std::shared_ptr<std_srvs::srv::Trigger::Request>&,
            const std::shared_ptr<std_srvs::srv::Trigger::Response>& response)
        { this->handle_cartesian_stop(response); },
        rmw_qos_profile_services_default,
        service_callback_group_);

    RCLCPP_INFO(get_logger(),
                "Services: /move_to_named_target, /move_to_joint_target, "
                "/move_to_pose_target, /move_to_named_pose_target, "
                "/move_to_cartesian_target, /move_to_named_cartesian_target, "
                "/move_cartesian_sequence, /move_cartesian_pose_sequence, /move_sequence, "
                "/move_cartesian_stop");
  }

  void handle_cartesian_stop(
      const std::shared_ptr<std_srvs::srv::Trigger::Response>& response)
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
  }

  uint64_t start_executor_log(
      const std::string& action_name,
      const std::string& execute_mode,
      const std::string& note = "")
  {
    if (!executor_logger_ || !executor_logger_->enabled())
    {
      return 0;
    }
    return executor_logger_->start_call(action_name, execute_mode, note);
  }

  void set_planner_log_context(
      const uint64_t action_call_id,
      const std::string& execute_mode,
      const std::vector<geometry_msgs::msg::Pose>& refs = {})
  {
    if (planner_)
    {
      planner_->set_log_context(action_call_id, execute_mode, refs);
    }
  }

  void finish_executor_log(
      const uint64_t action_call_id,
      const std::string& status,
      const bool success,
      const std::string& message,
      const double fraction = 0.0,
      const std::string& note = "")
  {
    if (executor_logger_ && executor_logger_->enabled() && action_call_id != 0)
    {
      executor_logger_->log_summary(action_call_id, status, success, message, fraction, note);
    }
    if (planner_)
    {
      planner_->clear_log_context();
    }
  }

  void log_ref_waypoint(
      const uint64_t action_call_id,
      const size_t index,
      const geometry_msgs::msg::Pose& pose,
      const rclcpp::Time& stamp,
      const std::string& note = "")
  {
    if (executor_logger_ && executor_logger_->enabled() && action_call_id != 0)
    {
      executor_logger_->log_ref_waypoint(action_call_id, index, pose, stamp, note);
    }
  }

  void log_joint_command(
      const uint64_t action_call_id,
      const moveit_msgs::msg::RobotTrajectory& trajectory,
      const std::string& note)
  {
    if (executor_logger_ && executor_logger_->enabled() && action_call_id != 0)
    {
      executor_logger_->log_joint_command(action_call_id, trajectory, note);
    }
  }

  bool execute_plan_with_logging(
      const uint64_t action_call_id,
      const std::string& execute_mode,
      const moveit_msgs::msg::RobotTrajectory& trajectory,
      const std::vector<geometry_msgs::msg::Pose>& refs,
      const std::string& note)
  {
    log_joint_command(action_call_id, trajectory, note);
    if (executor_logger_ && executor_logger_->enabled() && action_call_id != 0)
    {
      executor_logger_->start_sampling(action_call_id, execute_mode, refs);
    }
    const auto exec_result = move_group_->execute(trajectory);
    if (executor_logger_ && executor_logger_->enabled() && action_call_id != 0)
    {
      executor_logger_->stop_sampling(action_call_id);
    }
    return exec_result == moveit::core::MoveItErrorCode::SUCCESS;
  }

  void handle_named_target(
      const std::shared_ptr<robot_task_executor_msgs::srv::MoveToNamedTarget::Request>& request,
      const std::shared_ptr<robot_task_executor_msgs::srv::MoveToNamedTarget::Response>& response)
  {
    const std::string& target_name = request->target_name;
    const bool execute = request->execute;
    const uint64_t action_call_id = start_executor_log(
        "/move_to_named_target",
        execute ? "joint_target" : "plan_only",
        "callback_start");

    if (target_name.empty())
    {
      response->success = false;
      response->message = "target_name is empty";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      finish_executor_log(action_call_id, "failed", false, response->message);
      return;
    }

    if (!move_group_)
    {
      response->success = false;
      response->message = "MoveGroup not initialized. Call init_move_group() first.";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      finish_executor_log(action_call_id, "failed", false, response->message);
      return;
    }

    if (waypoint_loader_->has_joint_target(target_name))
    {
      const auto* joint_target = waypoint_loader_->get_joint_target(target_name);
      if (!joint_target)
      {
        response->success = false;
        response->message = "Internal error: joint target exists but cannot be retrieved.";
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        finish_executor_log(action_call_id, "failed", false, response->message);
        return;
      }

      static const std::vector<std::string> joint_order = {
          "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

      set_planner_log_context(
          action_call_id,
          execute ? "joint_target" : "plan_only");
      auto result = planner_->plan_joint_target(
          target_name, *joint_target, joint_order, execute);

      if (result.success)
      {
        viz_->publish_plan_visualization(result.trajectory, "/move_to_named_target (joint)");
      }

      response->success = result.success;
      response->message = result.success
          ? "Named joint target '" + target_name + "' " + (execute ? "executed" : "planned") + " successfully"
          : "Failed to " + std::string(execute ? "plan/execute" : "plan") + " named joint target '" + target_name + "'";
      finish_executor_log(
          action_call_id,
          response->success ? "completed" : "failed",
          response->success,
          response->message);
      return;
    }

    RCLCPP_INFO(get_logger(), "SRDF named target: '%s', execute=%d", target_name.c_str(), execute);
    move_group_->setStartStateToCurrentState();
    move_group_->setNamedTarget(target_name);
    move_group_->setPlanningTime(planning_time_);
    move_group_->setNumPlanningAttempts(num_planning_attempts_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = move_group_->plan(plan);

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      response->success = false;
      response->message = "SRDF target '" + target_name + "' planning failed (code=" +
                         std::to_string(plan_result.val) + ")";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      finish_executor_log(action_call_id, "failed", false, response->message);
      return;
    }

    RCLCPP_INFO(get_logger(), "SRDF target '%s' plan succeeded (%lu points).",
                 target_name.c_str(), plan.trajectory.joint_trajectory.points.size());

    viz_->publish_plan_visualization(plan.trajectory, "/move_to_named_target (SRDF)");

    if (execute)
    {
      RCLCPP_INFO(get_logger(), "Executing SRDF target '%s'...", target_name.c_str());
      const bool exec_ok = execute_plan_with_logging(
          action_call_id,
          "joint_target",
          plan.trajectory,
          {},
          "/move_to_named_target SRDF");
      if (!exec_ok)
      {
        response->success = false;
        response->message = "SRDF target '" + target_name + "' execution failed";
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        finish_executor_log(action_call_id, "failed", false, response->message);
        return;
      }
      RCLCPP_INFO(get_logger(), "SRDF target '%s' executed successfully.", target_name.c_str());
    }
    else
    {
      log_joint_command(action_call_id, plan.trajectory, "/move_to_named_target SRDF plan_only");
      RCLCPP_INFO(get_logger(), "SRDF target '%s': plan-only, not executed.", target_name.c_str());
    }

    response->success = true;
    response->message = "Named target '" + target_name + "' " +
                       (execute ? "executed" : "planned") + " successfully";
    finish_executor_log(action_call_id, "completed", true, response->message);
  }

  void handle_joint_target(
      const std::shared_ptr<robot_task_executor_msgs::srv::MoveToJointTarget::Request>& request,
      const std::shared_ptr<robot_task_executor_msgs::srv::MoveToJointTarget::Response>& response)
  {
    const auto& joint_names = request->joint_names;
    const auto& positions = request->positions;
    const bool execute = request->execute;
    const uint64_t action_call_id = start_executor_log(
        "/move_to_joint_target",
        execute ? "joint_target" : "plan_only",
        "callback_start");

    RCLCPP_INFO(get_logger(), "/move_to_joint_target: %zu joints, execute=%d",
                joint_names.size(), execute);

    if (joint_names.empty() || positions.empty())
    {
      response->success = false;
      response->message = "joint_names and positions must not be empty";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      finish_executor_log(action_call_id, "failed", false, response->message);
      return;
    }

    if (joint_names.size() != positions.size())
    {
      response->success = false;
      response->message = "joint_names size (" + std::to_string(joint_names.size()) +
                          ") must match positions size (" + std::to_string(positions.size()) + ")";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      finish_executor_log(action_call_id, "failed", false, response->message);
      return;
    }

    set_planner_log_context(
        action_call_id,
        execute ? "joint_target" : "plan_only");
    auto result = planner_->plan_joint_target("direct_joint", positions, joint_names, execute);

    if (result.success)
    {
      viz_->publish_plan_visualization(result.trajectory, "/move_to_joint_target");
    }

    response->success = result.success;
    response->message = result.success
        ? "Joint target " + std::string(execute ? "executed" : "planned") + " successfully"
        : "Failed to " + std::string(execute ? "plan/execute" : "plan") + " joint target";
    finish_executor_log(
        action_call_id,
        response->success ? "completed" : "failed",
        response->success,
        response->message);
  }

  void handle_pose_target(
      const std::shared_ptr<robot_task_executor_msgs::srv::MoveToPoseTarget::Request>& request,
      const std::shared_ptr<robot_task_executor_msgs::srv::MoveToPoseTarget::Response>& response)
  {
    const auto& pose = request->pose;
    const auto& requested_frame = request->frame_id;
    const bool execute = request->execute;
    const uint64_t action_call_id = start_executor_log(
        "/move_to_pose_target",
        execute ? "ptp" : "plan_only",
        "callback_start");

    RCLCPP_INFO(get_logger(), "/move_to_pose_target: frame='%s', xyz=(%.4f, %.4f, %.4f), execute=%d",
                requested_frame.c_str(),
                pose.position.x, pose.position.y, pose.position.z, execute);

    if (requested_frame.empty())
    {
      response->success = false;
      response->message = "frame_id must not be empty";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      finish_executor_log(action_call_id, "failed", false, response->message);
      return;
    }

    const double q_norm = std::sqrt(
        pose.orientation.x * pose.orientation.x +
        pose.orientation.y * pose.orientation.y +
        pose.orientation.z * pose.orientation.z +
        pose.orientation.w * pose.orientation.w);

    if (q_norm < 1e-9)
    {
      response->success = false;
      response->message = "Quaternion has zero norm";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      finish_executor_log(action_call_id, "failed", false, response->message);
      return;
    }

    const std::string ee_link = move_group_->getEndEffectorLink();
    const std::string task_frame = base_frame_;

    geometry_msgs::msg::Pose target_pose = pose;

    if (requested_frame != task_frame)
    {
      RCLCPP_INFO(get_logger(),
                  "/move_to_pose_target: transforming from '%s' to '%s'",
                  requested_frame.c_str(), task_frame.c_str());
      geometry_msgs::msg::PoseStamped pose_in;
      pose_in.header.stamp = get_clock()->now();
      pose_in.header.frame_id = requested_frame;
      pose_in.pose = pose;

      try
      {
        const auto transformed = transform_->transform_to_planning_frame(pose_in, task_frame);
        target_pose = transformed.pose;
      }
      catch (const tf2::TransformException& ex)
      {
        response->success = false;
        response->message = "TF transform failed: " + std::string(ex.what());
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        finish_executor_log(action_call_id, "failed", false, response->message);
        return;
      }
    }

    log_ref_waypoint(action_call_id, 0, target_pose, get_clock()->now(), requested_frame);

    move_group_->setStartStateToCurrentState();
    move_group_->setPoseReferenceFrame(task_frame);
    move_group_->setPlanningTime(planning_time_);
    move_group_->setNumPlanningAttempts(num_planning_attempts_);

    if (!move_group_->setPoseTarget(target_pose, ee_link))
    {
      response->success = false;
      response->message = "setPoseTarget() failed for link '" + ee_link + "'";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      move_group_->clearPoseTargets();
      finish_executor_log(action_call_id, "failed", false, response->message);
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = move_group_->plan(plan);

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      response->success = false;
      response->message = "Pose target planning failed (code=" +
                          std::to_string(plan_result.val) + ")";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      move_group_->clearPoseTargets();
      finish_executor_log(action_call_id, "failed", false, response->message);
      return;
    }

    RCLCPP_INFO(get_logger(), "/move_to_pose_target plan succeeded (%lu points).",
                 plan.trajectory.joint_trajectory.points.size());

    viz_->publish_plan_visualization(plan.trajectory, "/move_to_pose_target");

    if (execute)
    {
      RCLCPP_INFO(get_logger(), "/move_to_pose_target executing...");
      const bool exec_ok = execute_plan_with_logging(
          action_call_id,
          "ptp",
          plan.trajectory,
          {target_pose},
          "/move_to_pose_target");
      if (!exec_ok)
      {
        response->success = false;
        response->message = "Pose target execution failed";
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        move_group_->clearPoseTargets();
        finish_executor_log(action_call_id, "failed", false, response->message);
        return;
      }
      RCLCPP_INFO(get_logger(), "/move_to_pose_target executed successfully.");
    }
    else
    {
      log_joint_command(action_call_id, plan.trajectory, "/move_to_pose_target plan_only");
      RCLCPP_INFO(get_logger(), "/move_to_pose_target: plan-only.");
    }

    move_group_->clearPoseTargets();
    response->success = true;
    response->message = "Pose target " + std::string(execute ? "executed" : "planned") + " successfully";
    finish_executor_log(action_call_id, "completed", true, response->message);
  }

  void handle_named_pose_target(
      const std::shared_ptr<robot_task_executor_msgs::srv::MoveToNamedPoseTarget::Request>& request,
      const std::shared_ptr<robot_task_executor_msgs::srv::MoveToNamedPoseTarget::Response>& response)
  {
    const std::string& target_name = request->target_name;
    const bool execute = request->execute;
    const uint64_t action_call_id = start_executor_log(
        "/move_to_named_pose_target",
        execute ? "ptp" : "plan_only",
        "callback_start");

    if (target_name.empty())
    {
      response->success = false;
      response->message = "target_name is empty";
      RCLCPP_WARN(get_logger(), "/move_to_named_pose_target: %s", response->message.c_str());
      finish_executor_log(action_call_id, "failed", false, response->message);
      return;
    }

    const auto* stored_pose = waypoint_loader_->get_pose_target(target_name);
    if (!stored_pose)
    {
      response->success = false;
      response->message = "Unknown pose target: '" + target_name +
                         "'. Available: " + waypoint_loader_->available_pose_target_names();
      RCLCPP_ERROR(get_logger(), "/move_to_named_pose_target: %s", response->message.c_str());
      finish_executor_log(action_call_id, "failed", false, response->message);
      return;
    }

    log_ref_waypoint(
        action_call_id,
        0,
        stored_pose->pose,
        rclcpp::Time(stored_pose->header.stamp),
        target_name);
    set_planner_log_context(
        action_call_id,
        execute ? "ptp" : "plan_only",
        {stored_pose->pose});
    auto result = planner_->plan_named_pose_target(target_name, *stored_pose, execute);

    if (result.success)
    {
      viz_->publish_plan_visualization(result.trajectory, "/move_to_named_pose_target");
    }

    response->success = result.success;
    response->message = result.success
        ? "Named pose target '" + target_name + "' " + (execute ? "executed" : "planned") + " successfully"
        : "Failed to " + std::string(execute ? "plan/execute" : "plan") + " named pose target '" + target_name + "'";
    finish_executor_log(
        action_call_id,
        response->success ? "completed" : "failed",
        response->success,
        response->message);
  }

  // ── shared cartesian planning helper ───────────────────────────────────────
  // Both direct and named Cartesian services call this.  It is the single
  // source of truth for how a Cartesian target is planned, executed, and
  // visualised.  No logic may diverge here — any difference between the two
  // callers must be eliminated at the call-site (handle_*) before invoking
  // this helper.
  //
  // Both response types (MoveToCartesianTarget and MoveToNamedCartesianTarget)
  // share the same fields: { bool success, string message, float64 fraction }.

  template<typename ResT>
  void _plan_cartesian_to_pose(
      ResT* response,
      const geometry_msgs::msg::Pose& target_pose,
      const std::string& source_frame_id,
      bool execute,
      const char* service_name,
      uint64_t action_call_id)
  {
    RCLCPP_INFO(get_logger(), "======================================================");
    RCLCPP_INFO(get_logger(), "[%s] CARTESIAN PLANNING", service_name);
    RCLCPP_INFO(get_logger(), "  target frame_id : '%s'", source_frame_id.c_str());
    RCLCPP_INFO(get_logger(), "  target xyz      : (%.4f, %.4f, %.4f)",
                target_pose.position.x, target_pose.position.y, target_pose.position.z);
    RCLCPP_INFO(get_logger(), "  target quat    : (%.6f, %.6f, %.6f, %.6f)",
                target_pose.orientation.x, target_pose.orientation.y,
                target_pose.orientation.z, target_pose.orientation.w);
    RCLCPP_INFO(get_logger(), "  execute        : %s", execute ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  planning method: computeCartesianPath");
    RCLCPP_INFO(get_logger(), "  eef_step       : %.4f m",
                robot_task_executor::PlannerUtils::DEFAULT_EEF_STEP);
    RCLCPP_INFO(get_logger(), "  jump_threshold  : %.4f",
                robot_task_executor::PlannerUtils::DEFAULT_JUMP_THRESHOLD);
    RCLCPP_INFO(get_logger(), "  waypoint count : 1  (start state = current pose)");
    RCLCPP_INFO(get_logger(), "======================================================");
    log_ref_waypoint(action_call_id, 0, target_pose, get_clock()->now(), source_frame_id);
    set_planner_log_context(
        action_call_id,
        execute ? "cartesian" : "plan_only",
        {target_pose});

    auto result = planner_->plan_cartesian_from_poses(
        {target_pose},
        robot_task_executor::PlannerUtils::DEFAULT_EEF_STEP,
        robot_task_executor::PlannerUtils::DEFAULT_JUMP_THRESHOLD,
        service_name);

    response->fraction = planner_->last_cartesian_fraction();

    if (!result.success)
    {
      response->success = false;
      response->message = "Cartesian path failed or fraction < 0.95 (fraction=" +
                         std::to_string(response->fraction) + ")";
      RCLCPP_ERROR(get_logger(), "[%s] %s", service_name, response->message.c_str());
      finish_executor_log(
          action_call_id,
          "failed",
          false,
          response->message,
          response->fraction);
      return;
    }

    viz_->publish_plan_visualization(result.trajectory, service_name);

    if (execute)
    {
      const bool exec_ok = planner_->execute_trajectory(result.trajectory);
      response->success = exec_ok;
      response->message = exec_ok
          ? (std::string(service_name) + " executed successfully (fraction=" +
             std::to_string(response->fraction) + ")")
          : (std::string(service_name) + " planned but execution failed (fraction=" +
             std::to_string(response->fraction) + ")");
    }
    else
    {
      log_joint_command(action_call_id, result.trajectory, std::string(service_name) + " plan_only");
      response->success = true;
      response->message = std::string(service_name) +
                         " planned successfully (fraction=" +
                         std::to_string(response->fraction) + "), plan-only";
    }
    finish_executor_log(
        action_call_id,
        response->success ? "completed" : "failed",
        response->success,
        response->message,
        response->fraction);
  }

  // ── service callbacks ──────────────────────────────────────────────────────

  void handle_cartesian_target(
      const std::shared_ptr<robot_task_executor_msgs::srv::MoveToCartesianTarget::Request>& request,
      const std::shared_ptr<robot_task_executor_msgs::srv::MoveToCartesianTarget::Response>& response)
  {
    // Initialise shared response pointer for the helper
    const double x = request->x;
    const double y = request->y;
    const double z = request->z;
    const std::string& frame_id = request->frame_id;
    const bool execute = request->execute;
    const uint64_t action_call_id = start_executor_log(
        "/move_to_cartesian_target",
        execute ? "cartesian" : "plan_only",
        "callback_start");

    RCLCPP_INFO(get_logger(),
                "/move_to_cartesian_target: xyz=(%.4f, %.4f, %.4f) frame='%s' execute=%d",
                x, y, z, frame_id.c_str(), execute);

    if (frame_id.empty())
    {
      response->success = false;
      response->message = "frame_id must not be empty";
      response->fraction = 0.0;
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      finish_executor_log(action_call_id, "failed", false, response->message, response->fraction);
      return;
    }

    // Build target pose with fixed orientation (pi, 0, 0) — identical to what
    // the waypoint loader applies to YAML cartesian_points.
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    target_pose.orientation = planner_->cartesian_quat();

    RCLCPP_INFO(get_logger(),
                "[DIRECT] orientation source: planner_->cartesian_quat() "
                "(fixed rpy=(pi,0,0))  quat=(%.6f, %.6f, %.6f, %.6f)",
                target_pose.orientation.x, target_pose.orientation.y,
                target_pose.orientation.z, target_pose.orientation.w);

    // Transform to task frame if needed — same policy as named cartesian.
    const std::string task_frame = base_frame_;
    if (frame_id != task_frame)
    {
      RCLCPP_INFO(get_logger(),
                  "[DIRECT] frame_id='%s' != task_frame='%s' — transforming via TF",
                  frame_id.c_str(), task_frame.c_str());
      geometry_msgs::msg::PoseStamped pose_in;
      pose_in.header.stamp = get_clock()->now();
      pose_in.header.frame_id = frame_id;
      pose_in.pose = target_pose;

      try
      {
        const auto transformed = transform_->transform_to_planning_frame(pose_in, task_frame);
        target_pose = transformed.pose;
        RCLCPP_INFO(get_logger(),
                    "[DIRECT] TF result: xyz=(%.4f, %.4f, %.4f) in '%s'  "
                    "quat=(%.6f, %.6f, %.6f, %.6f)",
                    target_pose.position.x, target_pose.position.y, target_pose.position.z,
                    task_frame.c_str(),
                    target_pose.orientation.x, target_pose.orientation.y,
                    target_pose.orientation.z, target_pose.orientation.w);
      }
      catch (const tf2::TransformException& ex)
      {
        response->success = false;
        response->message = "TF transform failed: " + std::string(ex.what());
        response->fraction = 0.0;
        RCLCPP_ERROR(get_logger(), "[DIRECT] %s", response->message.c_str());
        finish_executor_log(action_call_id, "failed", false, response->message, response->fraction);
        return;
      }
    }
    else
    {
      RCLCPP_INFO(get_logger(),
                  "[DIRECT] frame_id='%s' == task_frame — no TF needed",
                  frame_id.c_str());
    }

    _plan_cartesian_to_pose(
        response.get(),
        target_pose,
        frame_id,
        execute,
        "/move_to_cartesian_target",
        action_call_id);
  }

  void handle_named_cartesian_target(
      const std::shared_ptr<robot_task_executor_msgs::srv::MoveToNamedCartesianTarget::Request>& request,
      const std::shared_ptr<robot_task_executor_msgs::srv::MoveToNamedCartesianTarget::Response>& response)
  {
    // Initialise shared response pointer for the helper
    const std::string& target_name = request->target_name;
    const bool execute = request->execute;
    const uint64_t action_call_id = start_executor_log(
        "/move_to_named_cartesian_target",
        execute ? "cartesian" : "plan_only",
        "callback_start");

    RCLCPP_INFO(get_logger(), "/move_to_named_cartesian_target: '%s' execute=%d",
                target_name.c_str(), execute);

    if (target_name.empty())
    {
      response->success = false;
      response->message = "target_name is empty";
      response->fraction = 0.0;
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      finish_executor_log(action_call_id, "failed", false, response->message, response->fraction);
      return;
    }

    if (!waypoint_loader_->has_cartesian_point(target_name))
    {
      response->success = false;
      response->message = "Unknown cartesian point: '" + target_name +
                          "'. Available: " + waypoint_loader_->available_cartesian_point_names();
      response->fraction = 0.0;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      finish_executor_log(action_call_id, "failed", false, response->message, response->fraction);
      return;
    }

    const auto* stored_pose = waypoint_loader_->get_cartesian_point(target_name);
    RCLCPP_INFO(get_logger(),
                "[NAMED] '%s' from YAML: frame_id='%s'  "
                "xyz=(%.4f, %.4f, %.4f)  "
                "quat=(%.6f, %.6f, %.6f, %.6f)",
                target_name.c_str(),
                stored_pose->header.frame_id.c_str(),
                stored_pose->pose.position.x,
                stored_pose->pose.position.y,
                stored_pose->pose.position.z,
                stored_pose->pose.orientation.x,
                stored_pose->pose.orientation.y,
                stored_pose->pose.orientation.z,
                stored_pose->pose.orientation.w);
    RCLCPP_INFO(get_logger(),
                "[NAMED] NOTE: YAML poses are assumed to already be in base_link. "
                "No TF transform is applied.  "
                "Verify stored_pose->header.frame_id matches base_frame_='%s'.",
                base_frame_.c_str());

    // Pass through the stored pose directly — same way the YAML loader returned it.
    // The waypoint loader already applied cartesian_quat_ (fixed pi,0,0) to it.
    _plan_cartesian_to_pose(
        response.get(),
        stored_pose->pose,
        stored_pose->header.frame_id,
        execute,
        "/move_to_named_cartesian_target",
        action_call_id);
  }

  void handle_cartesian_sequence(
      const std::shared_ptr<robot_task_executor_msgs::srv::MoveCartesianSequence::Request>& request,
      const std::shared_ptr<robot_task_executor_msgs::srv::MoveCartesianSequence::Response>& response)
  {
    const auto& waypoint_names = request->waypoint_names;
    const bool execute = request->execute;
    const uint64_t action_call_id = start_executor_log(
        "/move_cartesian_sequence",
        execute ? "cartesian" : "plan_only",
        "callback_start");

    RCLCPP_INFO(get_logger(), "/move_cartesian_sequence: %zu waypoints, execute=%d",
                waypoint_names.size(), execute);

    if (waypoint_names.empty())
    {
      response->success = false;
      response->message = "waypoint_names must not be empty";
      response->fraction = 0.0;
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      finish_executor_log(action_call_id, "failed", false, response->message, response->fraction);
      return;
    }

    for (const auto& name : waypoint_names)
    {
      if (!waypoint_loader_->has_cartesian_point(name))
      {
        response->success = false;
        response->message = "Unknown cartesian point: '" + name +
                            "'. Available: " + waypoint_loader_->available_cartesian_point_names();
        response->fraction = 0.0;
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        finish_executor_log(action_call_id, "failed", false, response->message, response->fraction);
        return;
      }
    }

    std::vector<geometry_msgs::msg::Pose> refs;
    refs.reserve(waypoint_names.size());
    for (size_t i = 0; i < waypoint_names.size(); ++i)
    {
      const auto* stored_pose = waypoint_loader_->get_cartesian_point(waypoint_names[i]);
      if (stored_pose)
      {
        refs.push_back(stored_pose->pose);
        log_ref_waypoint(
            action_call_id,
            i,
            stored_pose->pose,
            rclcpp::Time(stored_pose->header.stamp),
            waypoint_names[i]);
      }
    }
    set_planner_log_context(
        action_call_id,
        execute ? "cartesian" : "plan_only",
        refs);

    auto result = planner_->plan_cartesian_named_sequence(
        std::vector<std::string>(waypoint_names.begin(), waypoint_names.end()));

    response->fraction = planner_->last_cartesian_fraction();

    if (!result.success)
    {
      response->success = false;
      response->message = "Cartesian sequence path failed or fraction < 0.95 (fraction=" +
                         std::to_string(response->fraction) + ")";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      finish_executor_log(action_call_id, "failed", false, response->message, response->fraction);
      return;
    }

    viz_->publish_plan_visualization(result.trajectory, "/move_cartesian_sequence");

    if (execute)
    {
      const bool exec_ok = planner_->execute_trajectory(result.trajectory);
      response->success = exec_ok;
      response->message = exec_ok
          ? "Cartesian sequence executed (" + std::to_string(waypoint_names.size()) +
            " waypoints, fraction=" + std::to_string(response->fraction) + ")"
          : "Cartesian sequence path computed but execution failed";
    }
    else
    {
      log_joint_command(action_call_id, result.trajectory, "plan_only_cartesian_sequence");
      response->success = true;
      response->message = "Cartesian sequence planned (" +
                          std::to_string(waypoint_names.size()) +
                          " waypoints, fraction=" + std::to_string(response->fraction) + "), plan-only";
    }
    finish_executor_log(
        action_call_id,
        response->success ? "completed" : "failed",
        response->success,
        response->message,
        response->fraction);
  }

  void handle_move_sequence(
      const std::shared_ptr<robot_task_executor_msgs::srv::MoveSequence::Request>& request,
      const std::shared_ptr<robot_task_executor_msgs::srv::MoveSequence::Response>& response)
  {
    const auto& waypoint_names = request->waypoint_names;
    const bool execute = request->execute;
    const uint64_t action_call_id = start_executor_log(
        "/move_sequence",
        execute ? "joint_target" : "plan_only",
        "callback_start");

    if (waypoint_names.empty())
    {
      response->success = false;
      response->message = "waypoint_names must not be empty";
      RCLCPP_WARN(get_logger(), "/move_sequence: %s", response->message.c_str());
      finish_executor_log(action_call_id, "failed", false, response->message);
      return;
    }

    RCLCPP_INFO(get_logger(), "/move_sequence: %zu waypoints, execute=%d",
                waypoint_names.size(), execute);
    for (size_t i = 0; i < waypoint_names.size(); ++i)
    {
      RCLCPP_INFO(get_logger(), "  step %zu: '%s'", i + 1, waypoint_names[i].c_str());
    }

    static const std::vector<std::string> joint_order = {
        "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

    for (size_t i = 0; i < waypoint_names.size(); ++i)
    {
      const auto& name = waypoint_names[i];
      const auto* joint_target = waypoint_loader_->get_joint_target(name);

      if (!joint_target)
      {
        response->success = false;
        response->message = "Unknown waypoint '" + name + "' at step " +
                            std::to_string(i + 1) + " of " +
                            std::to_string(waypoint_names.size());
        RCLCPP_ERROR(get_logger(), "/move_sequence: %s", response->message.c_str());
        finish_executor_log(action_call_id, "failed", false, response->message);
        return;
      }

      RCLCPP_INFO(get_logger(), "/move_sequence step %zu/%zu: '%s'",
                  i + 1, waypoint_names.size(), name.c_str());

      set_planner_log_context(
          action_call_id,
          execute ? "joint_target" : "plan_only");
      auto result = planner_->plan_joint_target(name, *joint_target, joint_order, execute);

      if (result.success)
      {
        viz_->publish_plan_visualization(result.trajectory,
            "/move_sequence step " + std::to_string(i + 1));
      }

      if (!result.success)
      {
        response->success = false;
        response->message = "Failed at step " + std::to_string(i + 1) + " / '" + name + "'";
        RCLCPP_ERROR(get_logger(), "/move_sequence: %s", response->message.c_str());
        finish_executor_log(action_call_id, "failed", false, response->message);
        return;
      }

      if (i < waypoint_names.size() - 1)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }

    RCLCPP_INFO(get_logger(), "/move_sequence: all %zu waypoints completed.", waypoint_names.size());
    response->success = true;
    response->message = "MoveSequence completed (" + std::to_string(waypoint_names.size()) + " waypoints)";
    finish_executor_log(action_call_id, "completed", true, response->message);
  }

  void handle_cartesian_pose_sequence(
      const std::shared_ptr<robot_task_executor_msgs::srv::MoveCartesianPoseSequence::Request>& request,
      const std::shared_ptr<robot_task_executor_msgs::srv::MoveCartesianPoseSequence::Response>& response)
  {
    const auto& poses = request->poses;
    const bool execute = request->execute;
    const uint64_t action_call_id = start_executor_log(
        "/move_cartesian_pose_sequence",
        execute ? "cartesian" : "plan_only",
        "callback_start");

    RCLCPP_INFO(get_logger(),
                "/move_cartesian_pose_sequence: %zu poses, execute=%d",
                poses.size(), execute);

    // 1. Validate: poses must not be empty
    if (poses.empty())
    {
      response->success = false;
      response->message = "poses must not be empty";
      response->fraction = 0.0;
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      finish_executor_log(action_call_id, "failed", false, response->message, response->fraction);
      return;
    }

    if (!move_group_)
    {
      response->success = false;
      response->message = "MoveGroup not initialized. Call init_move_group() first.";
      response->fraction = 0.0;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      finish_executor_log(action_call_id, "failed", false, response->message, response->fraction);
      return;
    }

    // 2. Validate and normalise each pose
    const std::string default_frame = base_frame_;
    // Fallback orientation: Cartesian tool orientation used by DRL mock hardware.
    // quaternion xyzw = [0.7071068, 0.7071068, 0.0, 0.0]
    geometry_msgs::msg::Quaternion default_quat;
    default_quat.x = 0.7071068;
    default_quat.y = 0.7071068;
    default_quat.z = 0.0;
    default_quat.w = 0.0;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(poses.size());

    // Log current tcp_link pose from MoveIt (if available)
    try
    {
      const auto current_eef = move_group_->getCurrentPose().pose;
      RCLCPP_INFO(get_logger(),
                  "[%s] Current tcp_link xyz=(%.4f, %.4f, %.4f)  "
                  "quat=(%.6f, %.6f, %.6f, %.6f)",
                  "/move_cartesian_pose_sequence",
                  current_eef.position.x, current_eef.position.y, current_eef.position.z,
                  current_eef.orientation.x, current_eef.orientation.y,
                  current_eef.orientation.z, current_eef.orientation.w);
    }
    catch (const std::exception& ex)
    {
      RCLCPP_WARN(get_logger(), "[%s] Could not get current tcp_link pose: %s",
                   "/move_cartesian_pose_sequence", ex.what());
    }

    for (size_t i = 0; i < poses.size(); ++i)
    {
      const auto& pose_stamped = poses[i];

      // Default empty frame_id to base_link
      std::string frame_id = pose_stamped.header.frame_id;
      if (frame_id.empty())
      {
        RCLCPP_WARN(get_logger(),
                    "[%s] pose[%zu]: frame_id is empty, defaulting to '%s'",
                    "/move_cartesian_pose_sequence", i, default_frame.c_str());
        frame_id = default_frame;
      }

      // Reject non-base_link frames (TF transform not yet implemented)
      if (frame_id != default_frame && frame_id != base_frame_)
      {
        response->success = false;
        response->message = "Unsupported frame_id '" + frame_id + "': only '" +
                          default_frame + "' is supported. "
                          "TF transform to planning frame is not yet implemented.";
        response->fraction = 0.0;
        RCLCPP_ERROR(get_logger(), "[%s] %s",
                     "/move_cartesian_pose_sequence", response->message.c_str());
        finish_executor_log(action_call_id, "failed", false, response->message, response->fraction);
        return;
      }

      // Validate position: all values must be finite
      const auto& pos = pose_stamped.pose.position;
      if (!std::isfinite(pos.x) || !std::isfinite(pos.y) || !std::isfinite(pos.z))
      {
        response->success = false;
        response->message = "Pose[" + std::to_string(i) + "] has non-finite position: (" +
                          std::to_string(pos.x) + ", " + std::to_string(pos.y) + ", " +
                          std::to_string(pos.z) + ")";
        response->fraction = 0.0;
        RCLCPP_ERROR(get_logger(), "[%s] %s",
                     "/move_cartesian_pose_sequence", response->message.c_str());
        finish_executor_log(action_call_id, "failed", false, response->message, response->fraction);
        return;
      }

      // Use orientation as-is if valid (non-zero norm), otherwise use default
      geometry_msgs::msg::Pose waypoint_pose = pose_stamped.pose;
      const double q_norm = std::sqrt(
          waypoint_pose.orientation.x * waypoint_pose.orientation.x +
          waypoint_pose.orientation.y * waypoint_pose.orientation.y +
          waypoint_pose.orientation.z * waypoint_pose.orientation.z +
          waypoint_pose.orientation.w * waypoint_pose.orientation.w);

      if (q_norm < 1e-9)
      {
        RCLCPP_WARN(get_logger(),
                    "[%s] pose[%zu]: quaternion has zero norm, replacing with Cartesian "
                    "fallback quat=(%.6f, %.6f, %.6f, %.6f)",
                    "/move_cartesian_pose_sequence", i,
                    default_quat.x, default_quat.y,
                    default_quat.z, default_quat.w);
        waypoint_pose.orientation = default_quat;
      }

      waypoints.push_back(waypoint_pose);
      log_ref_waypoint(
          action_call_id,
          i,
          waypoint_pose,
          rclcpp::Time(pose_stamped.header.stamp),
          q_norm < 1e-9 ? "zero_quaternion_replaced_with_default" : "");

      if (i == 0)
      {
        RCLCPP_INFO(get_logger(),
                    "[%s]   FIRST pose[%zu] xyz=(%.4f, %.4f, %.4f)  "
                    "quat=(%.6f, %.6f, %.6f, %.6f)  frame='%s'",
                    "/move_cartesian_pose_sequence", i,
                    waypoint_pose.position.x, waypoint_pose.position.y, waypoint_pose.position.z,
                    waypoint_pose.orientation.x, waypoint_pose.orientation.y,
                    waypoint_pose.orientation.z, waypoint_pose.orientation.w,
                    frame_id.c_str());
      }
      else if (i == poses.size() - 1)
      {
        RCLCPP_INFO(get_logger(),
                    "[%s]   LAST  pose[%zu] xyz=(%.4f, %.4f, %.4f)  "
                    "quat=(%.6f, %.6f, %.6f, %.6f)  frame='%s'",
                    "/move_cartesian_pose_sequence", i,
                    waypoint_pose.position.x, waypoint_pose.position.y, waypoint_pose.position.z,
                    waypoint_pose.orientation.x, waypoint_pose.orientation.y,
                    waypoint_pose.orientation.z, waypoint_pose.orientation.w,
                    frame_id.c_str());
      }
      else
      {
        RCLCPP_DEBUG(get_logger(),
                    "[%s]   pose[%zu] xyz=(%.4f, %.4f, %.4f)  "
                    "quat=(%.6f, %.6f, %.6f, %.6f)  frame='%s'",
                    "/move_cartesian_pose_sequence", i,
                    waypoint_pose.position.x, waypoint_pose.position.y, waypoint_pose.position.z,
                    waypoint_pose.orientation.x, waypoint_pose.orientation.y,
                    waypoint_pose.orientation.z, waypoint_pose.orientation.w,
                    frame_id.c_str());
      }
    }

    // 3. Plan Cartesian path through all waypoints
    RCLCPP_INFO(get_logger(),
                "[%s] Calling computeCartesianPath with %zu waypoints  "
                "(eef_step=%.4f, jump_threshold=%.4f, avoid_collisions=true)",
                "/move_cartesian_pose_sequence", waypoints.size(),
                robot_task_executor::PlannerUtils::DEFAULT_EEF_STEP,
                robot_task_executor::PlannerUtils::DEFAULT_JUMP_THRESHOLD);
    set_planner_log_context(
        action_call_id,
        execute ? "cartesian" : "plan_only",
        waypoints);

    auto result = planner_->plan_cartesian_from_poses(
        waypoints,
        robot_task_executor::PlannerUtils::DEFAULT_EEF_STEP,
        robot_task_executor::PlannerUtils::DEFAULT_JUMP_THRESHOLD,
        "/move_cartesian_pose_sequence");

    response->fraction = planner_->last_cartesian_fraction();

    RCLCPP_INFO(get_logger(),
                "[%s] computeCartesianPath fraction=%.6f  trajectory_points=%lu  "
                "threshold=%.2f  success=%d",
                "/move_cartesian_pose_sequence",
                response->fraction,
                result.trajectory.joint_trajectory.points.size(),
                robot_task_executor::PlannerUtils::CARTESIAN_SUCCESS_THRESHOLD,
                result.success);

    // 4. Check fraction threshold
    if (!result.success)
    {
      RCLCPP_WARN(
          get_logger(),
          "[%s] Cartesian path fraction %.6f below threshold %.2f; trying PTP fallback.",
          "/move_cartesian_pose_sequence",
          response->fraction,
          robot_task_executor::PlannerUtils::CARTESIAN_SUCCESS_THRESHOLD);

      auto fallback = planner_->execute_pose_waypoints_ptp(
          waypoints,
          "/move_cartesian_pose_sequence",
          execute);

      if (!fallback.success)
      {
        response->success = false;
        response->message = "Cartesian path fraction below threshold: " +
                           std::to_string(response->fraction) +
                           "; PTP fallback failed at fraction: " +
                           std::to_string(fallback.fraction);
        RCLCPP_ERROR(get_logger(), "[%s] %s",
                     "/move_cartesian_pose_sequence", response->message.c_str());
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
      RCLCPP_INFO(get_logger(), "[%s] %s",
                  "/move_cartesian_pose_sequence", response->message.c_str());
      finish_executor_log(
          action_call_id,
          "completed",
          response->success,
          response->message,
          response->fraction,
          "ptp_fallback");
      return;
    }

    // 5. Visualise the plan
    viz_->publish_plan_visualization(result.trajectory, "/move_cartesian_pose_sequence");

    // 6. Execute or plan-only
    if (execute)
    {
      const bool exec_ok = planner_->execute_trajectory(result.trajectory);
      response->success = exec_ok;
      response->message = exec_ok
          ? ("Cartesian pose sequence executed successfully (fraction=" +
             std::to_string(response->fraction) + ")")
          : ("Cartesian pose sequence planned (fraction=" +
             std::to_string(response->fraction) + ") but execution failed");
      if (exec_ok)
      {
        RCLCPP_INFO(get_logger(), "[%s] %s",
                    "/move_cartesian_pose_sequence", response->message.c_str());
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "[%s] %s",
                     "/move_cartesian_pose_sequence", response->message.c_str());
      }
    }
    else
    {
      log_joint_command(
          action_call_id,
          result.trajectory,
          "plan_only_cartesian_pose_sequence");
      response->success = true;
      response->message = "Cartesian pose sequence planned successfully (fraction=" +
                        std::to_string(response->fraction) + "), plan-only";
      RCLCPP_INFO(get_logger(), "[%s] %s",
                  "/move_cartesian_pose_sequence", response->message.c_str());
    }
    finish_executor_log(
        action_call_id,
        response->success ? "completed" : "failed",
        response->success,
        response->message,
        response->fraction);
  }

  std::string move_group_name_;
  std::string base_frame_;
  std::string ee_link_;
  std::string waypoints_config_path_;
  std::string cartesian_points_config_path_;
  std::string pose_waypoints_config_path_;
  double planning_time_;
  int num_planning_attempts_;
  double max_velocity_scaling_factor_;
  double max_acceleration_scaling_factor_;
  bool enable_executor_logging_ = false;
  std::string executor_log_dir_;
  double executor_sample_rate_hz_ = 50.0;
  std::string executor_base_frame_;
  std::string executor_tcp_frame_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<robot_task_executor::PlannerUtils> planner_;
  std::unique_ptr<robot_task_executor::VisualizationUtils> viz_;
  std::unique_ptr<robot_task_executor::WaypointLoader> waypoint_loader_;
  std::unique_ptr<robot_task_executor::TransformUtils> transform_;
  std::shared_ptr<robot_task_executor::ExecutorExperimentLogger> executor_logger_;

  rclcpp::Service<robot_task_executor_msgs::srv::MoveToNamedTarget>::SharedPtr service_named_target_;
  rclcpp::Service<robot_task_executor_msgs::srv::MoveToJointTarget>::SharedPtr service_joint_target_;
  rclcpp::Service<robot_task_executor_msgs::srv::MoveToPoseTarget>::SharedPtr service_pose_target_;
  rclcpp::Service<robot_task_executor_msgs::srv::MoveToNamedPoseTarget>::SharedPtr service_named_pose_target_;
  rclcpp::Service<robot_task_executor_msgs::srv::MoveToCartesianTarget>::SharedPtr service_cartesian_target_;
  rclcpp::Service<robot_task_executor_msgs::srv::MoveToNamedCartesianTarget>::SharedPtr service_named_cartesian_target_;
  rclcpp::Service<robot_task_executor_msgs::srv::MoveCartesianSequence>::SharedPtr service_cartesian_sequence_;
  rclcpp::Service<robot_task_executor_msgs::srv::MoveCartesianPoseSequence>::SharedPtr service_cartesian_pose_sequence_;
  rclcpp::Service<robot_task_executor_msgs::srv::MoveSequence>::SharedPtr service_move_sequence_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_cartesian_stop_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskExecutorNode>();
  node->init_move_group();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
