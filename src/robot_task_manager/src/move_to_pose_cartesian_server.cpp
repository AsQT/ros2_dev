#include <memory>
#include <sstream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_task_manager/action/move_to_pose_cartesian.hpp"
#include "robot_task_manager/log_paths.hpp"
#include "robot_task_manager/moveit_executor.hpp"
#include "robot_task_manager/per_call_tcp_logger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class MoveToPoseCartesianActionServer : public rclcpp::Node
{
public:
  using MoveToPoseCartesian = robot_task_manager::action::MoveToPoseCartesian;
  using GoalHandleMoveToPoseCartesian = rclcpp_action::ServerGoalHandle<MoveToPoseCartesian>;

  MoveToPoseCartesianActionServer()
  : Node("move_to_pose_cartesian_action_server")
  {
    planning_group_ = declare_parameter<std::string>("planning_group", "arm");
    base_frame_ = declare_parameter<std::string>("base_frame", "world");

    enable_executor_logging_ = declare_parameter<bool>("enable_executor_logging", false);
    enable_debug_logging_ = declare_parameter<bool>("enable_debug_logging", false);
    log_root_dir_            = declare_parameter<std::string>("log_root_dir", robot_task_manager::kDefaultLogRootDir);
    runtime_mode_            = declare_parameter<std::string>("runtime_mode", "mock");
    executor_log_dir_        = declare_parameter<std::string>(
      "executor_log_dir", robot_task_manager::executorLogBaseDir(log_root_dir_));
    executor_sample_rate_hz_ = declare_parameter<double>("executor_sample_rate_hz", 50.0);
    executor_base_frame_     = declare_parameter<std::string>("executor_base_frame", "base_link");
    executor_tcp_frame_      = declare_parameter<std::string>("executor_tcp_frame", "tcp_link");
    declare_parameter<bool>("use_mock", true);
    declare_parameter<std::string>("hardware_plugin", "unknown");
    declare_parameter<bool>("enable_log_plots", true);

    action_server_ = rclcpp_action::create_server<MoveToPoseCartesian>(
      this,
      "move_to_pose_cartesian",
      std::bind(&MoveToPoseCartesianActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveToPoseCartesianActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MoveToPoseCartesianActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "MoveToPoseCartesian action server ready");
  }

  void initialize_moveit()
  {
    executor_ = std::make_shared<robot_task_manager::MoveItExecutor>();
    executor_->initialize(shared_from_this(), planning_group_, base_frame_);
    executor_->initializeLogging(
      enable_executor_logging_,
      robot_task_manager::executorActionLogDir(log_root_dir_, executor_log_dir_, runtime_mode_, "MoveToPoseCartesian"),
      executor_sample_rate_hz_,
      executor_base_frame_,
      executor_tcp_frame_,
      "/move_to_pose_cartesian");

    try {
      tcp_log_tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      tcp_log_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tcp_log_tf_buffer_);
      tcp_logger_ = std::make_shared<robot_task_manager::PerCallTcpLogger>(
        shared_from_this(), tcp_log_tf_buffer_,
        robot_task_manager::executorActionLogDir(log_root_dir_, executor_log_dir_, runtime_mode_, "MoveToPoseCartesian"),
        executor_sample_rate_hz_,
        executor_base_frame_, executor_tcp_frame_, "move_to_pose_cartesian", "/move_to_pose_cartesian");
      robot_task_manager::applyLogProvenanceFromParams(this, tcp_logger_);
    } catch (const std::exception & e) {
      tcp_logger_.reset();
      RCLCPP_WARN(get_logger(), "MoveToPoseCartesian per-call TCP logger unavailable: %s", e.what());
    }
  }

private:
  std::string planning_group_;
  std::string base_frame_;

  bool enable_executor_logging_{false};
  bool enable_debug_logging_{false};
  std::string log_root_dir_;
  std::string runtime_mode_;
  std::string executor_log_dir_;
  double executor_sample_rate_hz_{50.0};
  std::string executor_base_frame_;
  std::string executor_tcp_frame_;
  std::shared_ptr<tf2_ros::Buffer> tcp_log_tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tcp_log_tf_listener_;
  std::shared_ptr<robot_task_manager::PerCallTcpLogger> tcp_logger_;

  std::shared_ptr<robot_task_manager::MoveItExecutor> executor_;
  rclcpp_action::Server<MoveToPoseCartesian>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveToPoseCartesian::Goal> goal)
  {
    if (executor_ && executor_->getLogger()) {
      executor_->getLogger()->log_lifecycle_event(
        "/move_to_pose_cartesian", "action_goal_received", "handle_goal", "received", "");
    }
    if (goal->velocity_scale <= 0.0 || goal->velocity_scale > 1.0) {
      RCLCPP_WARN(get_logger(), "Reject goal because velocity_scale must be in (0,1]");
      if (executor_ && executor_->getLogger()) {
        executor_->getLogger()->log_lifecycle_event(
          "/move_to_pose_cartesian", "action_goal_rejected", "handle_goal", "rejected",
          "velocity_scale must be in (0,1]");
      }
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveToPoseCartesian> goal_handle)
  {
    RCLCPP_WARN(get_logger(), "Cancel request received");
    (void)goal_handle;

    try {
      if (executor_) {
        executor_->stop();
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Exception during stop(): %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Unknown exception during stop()");
    }

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveToPoseCartesian> goal_handle)
  {
    std::thread(&MoveToPoseCartesianActionServer::execute, this, goal_handle).detach();
  }

  void execute(const std::shared_ptr<GoalHandleMoveToPoseCartesian> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Execute started");

    auto feedback = std::make_shared<MoveToPoseCartesian::Feedback>();
    auto result = std::make_shared<MoveToPoseCartesian::Result>();

    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "goal_handle is null");
      return;
    }

    if (!executor_) {
      RCLCPP_ERROR(this->get_logger(), "executor_ is null");
      result->success = false;
      result->message = "Internal error: executor is null";
      goal_handle->abort(result);
      return;
    }

    if (goal_handle->is_canceling()) {
      RCLCPP_WARN(this->get_logger(), "Goal already canceling before execution");
      result->success = false;
      result->message = "Goal canceled before execution";
      goal_handle->canceled(result);
      return;
    }

    auto goal = goal_handle->get_goal();
    if (!goal) {
      RCLCPP_ERROR(this->get_logger(), "Goal is null");
      result->success = false;
      result->message = "Internal error: goal is null";
      goal_handle->abort(result);
      return;
    }
    RCLCPP_INFO(
      get_logger(), "[move_to_pose_cartesian server] enable_tcp_log=%s",
      goal->enable_tcp_log ? "true" : "false");

    // codex.md §4: evaluation logging gated ONLY by the per-goal flag.
    std::shared_ptr<robot_task_manager::PerCallTcpLogger::Call> tcp_call;
    if (tcp_logger_ && goal->enable_tcp_log) {
      std::ostringstream meta;
      meta << "{\"velocity_scale\":" << goal->velocity_scale
           << ",\"execute\":" << (goal->execute ? "true" : "false") << "}";
      tcp_call = tcp_logger_->startCall(meta.str());
      if (tcp_call) {
        tcp_logger_->logEvent(tcp_call, "cartesian_start", "received", "MoveToPoseCartesian goal accepted");
        tcp_logger_->updateStage(tcp_call, "cartesian_planning", goal->target_pose);
        tcp_logger_->logEvent(tcp_call, "cartesian_planning", "stage_start", "planning cartesian path", &goal->target_pose);
        tcp_logger_->startSampling(tcp_call);
      }
    }

    feedback->stage = goal->execute ? "Planning Cartesian path" : "Planning Cartesian path (plan-only)";
    feedback->progress = 30.0f;
    goal_handle->publish_feedback(feedback);

    if (tcp_logger_ && tcp_call) {
      tcp_logger_->updateStage(tcp_call, "cartesian_execution", goal->target_pose);
    }

    std::string error_msg;
    bool ok = false;

    try {
      RCLCPP_INFO(this->get_logger(), "Calling executor_->moveToPoseCartesian()");
      ok = executor_->moveToPoseCartesian(
        goal->target_pose,
        error_msg,
        goal->velocity_scale,
        0.3,
        5.0,
        goal->execute);
      RCLCPP_INFO(this->get_logger(), "Returned from moveToPoseCartesian(), ok=%s", ok ? "true" : "false");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in moveToPoseCartesian: %s", e.what());
      result->success = false;
      result->message = std::string("Exception: ") + e.what();
      if (executor_ && executor_->getLogger()) {
        executor_->getLogger()->log_lifecycle_event(
          "/move_to_pose_cartesian", "action_result", "move_to_pose_cartesian", "aborted", result->message);
      }
      if (tcp_logger_ && tcp_call) {
        tcp_logger_->logEvent(tcp_call, "cartesian_execution", "stage_failed", result->message, &goal->target_pose);
        tcp_logger_->finishCall(tcp_call, "aborted", false, result->message);
      }
      goal_handle->abort(result);
      return;
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown exception in moveToPoseCartesian");
      result->success = false;
      result->message = "Unknown exception in moveToPoseCartesian";
      if (executor_ && executor_->getLogger()) {
        executor_->getLogger()->log_lifecycle_event(
          "/move_to_pose_cartesian", "action_result", "move_to_pose_cartesian", "aborted", result->message);
      }
      if (tcp_logger_ && tcp_call) {
        tcp_logger_->logEvent(tcp_call, "cartesian_execution", "stage_failed", result->message, &goal->target_pose);
        tcp_logger_->finishCall(tcp_call, "aborted", false, result->message);
      }
      goal_handle->abort(result);
      return;
    }

    if (goal_handle->is_canceling()) {
      RCLCPP_WARN(this->get_logger(), "Goal canceled during execution");
      result->success = false;
      result->message = "Goal canceled";
      if (executor_ && executor_->getLogger()) {
        executor_->getLogger()->log_lifecycle_event(
          "/move_to_pose_cartesian", "action_result", "move_to_pose_cartesian", "canceled", result->message);
      }
      if (tcp_logger_ && tcp_call) {
        tcp_logger_->finishCall(tcp_call, "canceled", false, result->message);
      }
      goal_handle->canceled(result);
      return;
    }

    if (!ok) {
      result->success = false;
      result->message = error_msg.empty() ? "Cartesian motion failed" : error_msg;
      RCLCPP_ERROR(this->get_logger(), "Aborting goal: %s", result->message.c_str());
      if (executor_ && executor_->getLogger()) {
        executor_->getLogger()->log_lifecycle_event(
          "/move_to_pose_cartesian", "action_result", "move_to_pose_cartesian", "aborted", result->message);
      }
      if (tcp_logger_ && tcp_call) {
        tcp_logger_->logEvent(tcp_call, "cartesian_execution", "stage_failed", result->message, &goal->target_pose);
        tcp_logger_->finishCall(tcp_call, "aborted", false, result->message);
      }
      goal_handle->abort(result);
      return;
    }

    feedback->stage = goal->execute ? "Pose reached" : "Cartesian plan validated (execution skipped)";
    feedback->progress = 100.0f;
    goal_handle->publish_feedback(feedback);

    result->success = true;
    result->message = goal->execute ?
      "Robot reached target pose successfully" :
      "MoveToPoseCartesian planning success; execution skipped";
    if (executor_ && executor_->getLogger()) {
      executor_->getLogger()->log_lifecycle_event(
        "/move_to_pose_cartesian", "action_result", "move_to_pose_cartesian", "succeeded", result->message);
    }
    if (tcp_logger_ && tcp_call) {
      tcp_logger_->logEvent(tcp_call, "cartesian_execution", "stage_end", "reached target pose", &goal->target_pose);
      tcp_logger_->logEvent(tcp_call, "cartesian_end", "cartesian_end", result->message);
      tcp_logger_->finishCall(tcp_call, "completed", true, result->message);
    }

    RCLCPP_INFO(this->get_logger(), "Calling goal_handle->succeed()");
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Execute finished");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToPoseCartesianActionServer>();
  node->initialize_moveit();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
