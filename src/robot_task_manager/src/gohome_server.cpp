#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_task_manager/action/go_home.hpp"
#include "robot_task_manager/log_paths.hpp"
#include "robot_task_manager/moveit_executor.hpp"
#include "robot_task_manager/standard_action_logger.hpp"

class GoHomeActionServer : public rclcpp::Node
{
public:
  using GoHome            = robot_task_manager::action::GoHome;
  using GoalHandleGoHome  = rclcpp_action::ServerGoalHandle<GoHome>;

  GoHomeActionServer()
  : Node("gohome_action_server")
  {
    planning_group_ = declare_parameter<std::string>("planning_group",  "arm");
    home_target_    = declare_parameter<std::string>("home_target",     "home");
    base_frame_     = declare_parameter<std::string>("base_frame",      "world");
    // codex.md Phase 4: allow a second GoHome instance to serve a distinct
    // action name (e.g. /gohome_2 -> home_2) without a new executable or any
    // change to the GoHome interface. Default "gohome" keeps every existing
    // launch/client untouched.
    action_name_    = declare_parameter<std::string>("action_name",     "gohome");

    enable_executor_logging_ = declare_parameter<bool>("enable_executor_logging", false);
    enable_standard_logging_ = declare_parameter<bool>("enable_standard_logging", true);
    log_root_dir_            = declare_parameter<std::string>("log_root_dir", robot_task_manager::kDefaultLogRootDir);
    runtime_mode_            = declare_parameter<std::string>("runtime_mode", "mock");
    executor_log_dir_        = declare_parameter<std::string>(
      "executor_log_dir", robot_task_manager::executorLogBaseDir(log_root_dir_));
    executor_sample_rate_hz_ = declare_parameter<double>("executor_sample_rate_hz", 50.0);
    executor_base_frame_     = declare_parameter<std::string>("executor_base_frame", "base_link");
    executor_tcp_frame_      = declare_parameter<std::string>("executor_tcp_frame", "tcp_link");

    action_server_ = rclcpp_action::create_server<GoHome>(
                                        this,
                                        action_name_,
                                        std::bind(&GoHomeActionServer::handle_goal, 
                                                  this, std::placeholders::_1, std::placeholders::_2),
                                        std::bind(&GoHomeActionServer::handle_cancel, 
                                                  this, std::placeholders::_1),
                                        std::bind(&GoHomeActionServer::handle_accepted, 
                                                  this, std::placeholders::_1));

    // Log identifiers derived from the action name so a second instance
    // (/gohome_2 -> home_2) writes to its own log stream / directory.
    log_id_ = "/" + action_name_;
    log_action_dir_ = (action_name_ == "gohome") ? "GoHome" : "GoHome2";

    RCLCPP_INFO(
      get_logger(), "GoHome action server ready (action=%s, home_target=%s)",
      action_name_.c_str(), home_target_.c_str());
  }

  void initialize_moveit()
  {
    executor_ = std::make_shared<robot_task_manager::MoveItExecutor>();
    executor_->initialize(shared_from_this(), planning_group_, base_frame_);
    executor_->initializeLogging(
      enable_executor_logging_,
      robot_task_manager::executorActionLogDir(log_root_dir_, executor_log_dir_, runtime_mode_, log_action_dir_),
      executor_sample_rate_hz_,
      executor_base_frame_,
      executor_tcp_frame_,
      log_id_);
    if (enable_standard_logging_) {
      standard_logger_ = std::make_shared<robot_task_manager::StandardActionLogger>(
        shared_from_this(), log_root_dir_, runtime_mode_, "GoHome",
        "", executor_base_frame_, executor_tcp_frame_, "", "", "baseline", "");
    }
  }

private:
  std::string planning_group_;
  std::string home_target_;
  std::string base_frame_;
  std::string action_name_{"gohome"};
  std::string log_id_{"/gohome"};
  std::string log_action_dir_{"GoHome"};

  bool enable_executor_logging_{false};
  bool enable_standard_logging_{true};
  std::string log_root_dir_;
  std::string runtime_mode_;
  std::string executor_log_dir_;
  double executor_sample_rate_hz_{50.0};
  std::string executor_base_frame_;
  std::string executor_tcp_frame_;

  std::shared_ptr<robot_task_manager::MoveItExecutor> executor_;
  std::shared_ptr<robot_task_manager::StandardActionLogger> standard_logger_;
  rclcpp_action::Server<GoHome>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
                                const rclcpp_action::GoalUUID &,
                                std::shared_ptr<const GoHome::Goal>)
                    {
                      if (executor_ && executor_->getLogger()) {
                        executor_->getLogger()->log_lifecycle_event(
                          log_id_, "action_goal_received", "handle_goal", "received", "");
                      }
                      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
                    }

  rclcpp_action::CancelResponse handle_cancel(
                                  const std::shared_ptr<GoalHandleGoHome>)
                    {
                      RCLCPP_WARN(get_logger(), "Cancel request received");
                      if (executor_) {
                        executor_->stop();
                      }
                      return rclcpp_action::CancelResponse::ACCEPT;
                    }

  void handle_accepted(
                  const std::shared_ptr<GoalHandleGoHome> goal_handle)
          {
            std::thread(&GoHomeActionServer::execute, this, goal_handle).detach();
          }

  void execute(
        const std::shared_ptr<GoalHandleGoHome> goal_handle)
    {
      auto feedback   = std::make_shared<GoHome::Feedback>();
      auto result     = std::make_shared<GoHome::Result>();

      const auto goal = goal_handle->get_goal();
      auto log_call = standard_logger_ ? standard_logger_->startCall() : nullptr;
      if (!goal->start) {
        result->success = false;
        result->message = "GoHome rejected because start=false";
        if (executor_ && executor_->getLogger()) {
          executor_->getLogger()->log_lifecycle_event(
            log_id_, "action_result", "validate_goal", "aborted", result->message);
        }
        if (standard_logger_ && log_call) {
          robot_task_manager::StandardSummary summary;
          summary.execute_requested = goal->execute;
          summary.success = false;
          summary.failed_stage = "validate_goal";
          summary.failure_reason = "start_false";
          summary.message = result->message;
          summary.total_time_s = (now() - log_call->start_time).seconds();
          standard_logger_->writeSummary(*log_call, summary);
          standard_logger_->appendEvent(*log_call, "validate_goal", "action_result", false, result->message, summary.total_time_s);
        }
        goal_handle->abort(result);
        return;
      }

      feedback->current_step = goal->execute ? "Planning to home" : "Planning to home (plan-only)";
      feedback->progress = 30.0f;
      goal_handle->publish_feedback(feedback);

      std::string error_msg;
      const bool ok = executor_->goNamedTarget(home_target_, error_msg, 0.3, 0.3, 5.0, goal->execute);

      if (!ok) {
        result->success = false;
        result->message = error_msg;
        if (executor_ && executor_->getLogger()) {
          executor_->getLogger()->log_lifecycle_event(
            log_id_, "action_result", "go_named_target", "aborted", result->message);
        }
        if (standard_logger_ && log_call) {
          robot_task_manager::StandardSummary summary;
          summary.execute_requested = goal->execute;
          summary.success = false;
          summary.failed_stage = "go_named_target";
          summary.failure_reason = "planner_or_executor_failed";
          summary.message = result->message;
          summary.total_time_s = (now() - log_call->start_time).seconds();
          standard_logger_->writeSummary(*log_call, summary);
          standard_logger_->appendEvent(*log_call, "go_named_target", "action_result", false, result->message, summary.total_time_s);
        }
        goal_handle->abort(result);
        return;
      }

      feedback->current_step = goal->execute ? "Home_reached" : "Home plan validated (execution skipped)";
      feedback->progress = 100.0f;
      goal_handle->publish_feedback(feedback);

      result->success = true;
      result->message = goal->execute ?
        "Robot moved to home successfully" :
        "GoHome plan succeeded; execution skipped because execute=false";
      if (executor_ && executor_->getLogger()) {
        executor_->getLogger()->log_lifecycle_event(
          log_id_, "action_result", "go_named_target", "succeeded", result->message);
      }
      if (standard_logger_ && log_call) {
        robot_task_manager::StandardSummary summary;
        summary.execute_requested = goal->execute;
        summary.success = true;
        summary.message = result->message;
        summary.total_time_s = (now() - log_call->start_time).seconds();
        standard_logger_->writeSummary(*log_call, summary);
        standard_logger_->appendEvent(*log_call, "go_named_target", "action_result", true, result->message, summary.total_time_s);
      }
      goal_handle->succeed(result);
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoHomeActionServer>();
  node->initialize_moveit();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
