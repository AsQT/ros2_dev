#include <memory>
#include <sstream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_task_manager/action/move_to_pose.hpp"
#include "robot_task_manager/log_paths.hpp"
#include "robot_task_manager/moveit_executor.hpp"
#include "robot_task_manager/per_call_tcp_logger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class MoveToPoseActionServer : public rclcpp::Node
{
public:
  using MoveToPose = robot_task_manager::action::MoveToPose;
  using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;

  MoveToPoseActionServer(): Node("move_to_pose_action_server")
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
    // codex.md §5/§6: evaluation logging provenance + plot toggle.
    declare_parameter<bool>("use_mock", true);
    declare_parameter<std::string>("hardware_plugin", "unknown");
    declare_parameter<bool>("enable_log_plots", true);

    action_server_ = rclcpp_action::create_server<MoveToPose>(
      this,
      "move_to_pose",
      std::bind(&MoveToPoseActionServer::handle_goal,     this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveToPoseActionServer::handle_cancel,   this, std::placeholders::_1),
      std::bind(&MoveToPoseActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "MoveToPose action server ready");
  }

  void initialize_moveit()
  {
    executor_ = std::make_shared<robot_task_manager::MoveItExecutor>();
    executor_->initialize(shared_from_this(), planning_group_, base_frame_);
    executor_->initializeLogging(
      enable_executor_logging_,
      robot_task_manager::executorActionLogDir(log_root_dir_, executor_log_dir_, runtime_mode_, "MoveToPose"),
      executor_sample_rate_hz_,
      executor_base_frame_,
      executor_tcp_frame_,
      "/move_to_pose");

    try {
      tcp_log_tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      tcp_log_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tcp_log_tf_buffer_);
      tcp_logger_ = std::make_shared<robot_task_manager::PerCallTcpLogger>(
        shared_from_this(), tcp_log_tf_buffer_,
        robot_task_manager::executorActionLogDir(log_root_dir_, executor_log_dir_, runtime_mode_, "MoveToPose"),
        executor_sample_rate_hz_,
        executor_base_frame_, executor_tcp_frame_, "move_to_pose", "/move_to_pose");
      robot_task_manager::applyLogProvenanceFromParams(this, tcp_logger_);
    } catch (const std::exception & e) {
      tcp_logger_.reset();
      RCLCPP_WARN(get_logger(), "MoveToPose per-call TCP logger unavailable: %s", e.what());
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
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
                                const rclcpp_action::GoalUUID &,
                                std::shared_ptr<const MoveToPose::Goal> goal)
  {
    if (executor_ && executor_->getLogger()) {
      executor_->getLogger()->log_lifecycle_event(
        "/move_to_pose", "action_goal_received", "handle_goal", "received", "");
    }
    if (goal->velocity_scale <= 0.0 || goal->velocity_scale > 1.0) {
      RCLCPP_WARN(get_logger(), "Reject goal because velocity_scale must be in (0,1]");
      if (executor_ && executor_->getLogger()) {
        executor_->getLogger()->log_lifecycle_event(
          "/move_to_pose", "action_goal_rejected", "handle_goal", "rejected",
          "velocity_scale must be in (0,1]");
      }
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveToPose>)
  {
    RCLCPP_WARN(get_logger(), "Cancel request received");
    if (executor_) {
      executor_->stop();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    std::thread(&MoveToPoseActionServer::execute, this, goal_handle).detach();
  }

  void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    auto feedback = std::make_shared<MoveToPose::Feedback>();
    auto result = std::make_shared<MoveToPose::Result>();

    const auto goal = goal_handle->get_goal();
    RCLCPP_INFO(
      get_logger(), "[move_to_pose server] enable_tcp_log=%s",
      goal->enable_tcp_log ? "true" : "false");

    // Per-call TCP CSV logging is opt-in per goal (goal->enable_tcp_log),
    // e.g. when calling /move_to_pose directly to evaluate the model. When
    // called as a sub-action (from /pickplace, /repeatability_test), the
    // caller leaves this field at its default (false), so no extra file is
    // created and the parent action's own log is not drawn over.
    // codex.md §4: evaluation logging is gated ONLY by the per-goal flag
    // (enable_tcp_log). enable_debug_logging_ must never block the evaluation
    // CSV — it is kept solely for terminal/debug verbosity elsewhere.
    std::shared_ptr<robot_task_manager::PerCallTcpLogger::Call> tcp_call;
    if (tcp_logger_ && goal->enable_tcp_log) {
      std::ostringstream meta;
      meta << "{\"velocity_scale\":" << goal->velocity_scale
           << ",\"execute\":" << (goal->execute ? "true" : "false") << "}";
      tcp_call = tcp_logger_->startCall(meta.str());
      if (tcp_call) {
        tcp_logger_->logEvent(tcp_call, "move_to_pose_start", "received", "MoveToPose goal accepted");
        tcp_logger_->updateStage(tcp_call, "planning", goal->target_pose);
        tcp_logger_->logEvent(tcp_call, "planning", "stage_start", "planning to target pose", &goal->target_pose);
        tcp_logger_->startSampling(tcp_call);
      }
    }

    feedback->stage = goal->execute ? "Planning to pose" : "Planning to pose (plan-only)";
    feedback->progress = 30.0f;
    goal_handle->publish_feedback(feedback);

    if (tcp_logger_ && tcp_call) {
      tcp_logger_->updateStage(tcp_call, "execution", goal->target_pose);
    }

    std::string error_msg;
    const bool ok = executor_->moveToPose(
                                goal->target_pose,
                                error_msg,
                                goal->velocity_scale,
                                0.3,
                                5.0,
                                goal->execute);

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Goal canceled";
      if (executor_ && executor_->getLogger()) {
        executor_->getLogger()->log_lifecycle_event(
          "/move_to_pose", "action_result", "move_to_pose", "canceled", result->message);
      }
      if (tcp_logger_ && tcp_call) {
        tcp_logger_->finishCall(tcp_call, "canceled", false, result->message);
      }
      goal_handle->canceled(result);
      return;
    }

    if (!ok) {
      result->success = false;
      result->message = error_msg;
      if (executor_ && executor_->getLogger()) {
        executor_->getLogger()->log_lifecycle_event(
          "/move_to_pose", "action_result", "move_to_pose", "aborted", result->message);
      }
      if (tcp_logger_ && tcp_call) {
        tcp_logger_->logEvent(tcp_call, "execution", "stage_failed", result->message, &goal->target_pose);
        tcp_logger_->finishCall(tcp_call, "aborted", false, result->message);
      }
      goal_handle->abort(result);
      return;
    }

    feedback->stage = goal->execute ? "Pose reached" : "Pose plan validated (execution skipped)";
    feedback->progress = 100.0f;
    goal_handle->publish_feedback(feedback);

    result->success = true;
    result->message = goal->execute ?
      "Robot reached target pose successfully" :
      "MoveToPose planning success; execution skipped";
    if (executor_ && executor_->getLogger()) {
      executor_->getLogger()->log_lifecycle_event(
        "/move_to_pose", "action_result", "move_to_pose", "succeeded", result->message);
    }
    if (tcp_logger_ && tcp_call) {
      tcp_logger_->logEvent(tcp_call, "execution", "stage_end", "reached target pose", &goal->target_pose);
      tcp_logger_->logEvent(tcp_call, "move_to_pose_end", "move_to_pose_end", result->message);
      tcp_logger_->finishCall(tcp_call, "completed", true, result->message);
    }
    goal_handle->succeed(result);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToPoseActionServer>();
  node->initialize_moveit();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
