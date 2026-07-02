#include <memory>
#include <sstream>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_task_manager/action/checker_board.hpp"
#include "robot_task_manager/log_paths.hpp"
#include "robot_task_manager/moveit_executor.hpp"
#include "robot_task_manager/per_call_tcp_logger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class CheckerBoardActionServer : public rclcpp::Node
{
public:
  using CheckerBoard = robot_task_manager::action::CheckerBoard;
  using GoalHandleCheckerBoard = rclcpp_action::ServerGoalHandle<CheckerBoard>;

  CheckerBoardActionServer()
  : Node("move_checker_board_action_server")
  {
    planning_group_ = declare_parameter<std::string>("planning_group", "arm");
    base_frame_ = declare_parameter<std::string>("base_frame", "world");
    measurement_settle_time_s_ = declare_parameter<double>("measurement_settle_time_s", 2.0);

    enable_executor_logging_ = declare_parameter<bool>("enable_executor_logging", false);
    log_root_dir_            = declare_parameter<std::string>("log_root_dir", robot_task_manager::kDefaultLogRootDir);
    executor_log_dir_        = declare_parameter<std::string>(
      "executor_log_dir", robot_task_manager::executorLogBaseDir(log_root_dir_));
    executor_sample_rate_hz_ = declare_parameter<double>("executor_sample_rate_hz", 50.0);
    executor_base_frame_     = declare_parameter<std::string>("executor_base_frame", "base_link");
    executor_tcp_frame_      = declare_parameter<std::string>("executor_tcp_frame", "tcp_link");

    action_server_ = rclcpp_action::create_server<CheckerBoard>(
      this,
      "move_checker_board",
      std::bind(&CheckerBoardActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CheckerBoardActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&CheckerBoardActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "CheckerBoard action server ready");
  }

  void initialize_moveit()
  {
    executor_ = std::make_shared<robot_task_manager::MoveItExecutor>();
    executor_->initialize(shared_from_this(), planning_group_, base_frame_);

    try {
      tcp_log_tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      tcp_log_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tcp_log_tf_buffer_);
      tcp_logger_ = std::make_shared<robot_task_manager::PerCallTcpLogger>(
        shared_from_this(), tcp_log_tf_buffer_,
        robot_task_manager::executorActionLogDir(log_root_dir_, executor_log_dir_, "CheckerBoard"),
        executor_sample_rate_hz_,
        executor_base_frame_, executor_tcp_frame_, "checker_board", "/move_checker_board");
    } catch (const std::exception & e) {
      tcp_logger_.reset();
      RCLCPP_WARN(get_logger(), "CheckerBoard per-call TCP logger unavailable: %s", e.what());
    }
  }

private:
  std::string planning_group_;
  std::string base_frame_;
  double measurement_settle_time_s_{2.0};

  bool enable_executor_logging_{false};
  std::string log_root_dir_;
  std::string executor_log_dir_;
  double executor_sample_rate_hz_{50.0};
  std::string executor_base_frame_;
  std::string executor_tcp_frame_;
  std::shared_ptr<tf2_ros::Buffer> tcp_log_tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tcp_log_tf_listener_;
  std::shared_ptr<robot_task_manager::PerCallTcpLogger> tcp_logger_;

  std::shared_ptr<robot_task_manager::MoveItExecutor> executor_;
  rclcpp_action::Server<CheckerBoard>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
                                  const rclcpp_action::GoalUUID &,
                                  std::shared_ptr<const CheckerBoard::Goal> goal)
  {
    if (!std::isfinite(goal->step) || goal->step <= 0.0) {
      RCLCPP_WARN(get_logger(), "Reject goal because CheckerBoard step must be finite and > 0.0");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->velocity_scale <= 0.0 || goal->velocity_scale > 1.0) {
      RCLCPP_WARN(get_logger(), "Reject goal because velocity_scale must be in (0,1]");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!std::isfinite(measurement_settle_time_s_) || measurement_settle_time_s_ < 0.0) {
      RCLCPP_WARN(get_logger(), "Reject goal because measurement_settle_time_s must be finite and >= 0.0");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
                                        const std::shared_ptr<GoalHandleCheckerBoard> goal_handle)
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

  void handle_accepted(const std::shared_ptr<GoalHandleCheckerBoard> goal_handle)
  {
    std::thread(&CheckerBoardActionServer::execute, this, goal_handle).detach();
  }

  void execute(const std::shared_ptr<GoalHandleCheckerBoard> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Execute started");

    auto feedback = std::make_shared<CheckerBoard::Feedback>();
    auto result = std::make_shared<CheckerBoard::Result>();

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
      get_logger(), "[move_checker_board server] enable_tcp_log=%s",
      goal->enable_tcp_log ? "true" : "false");

    feedback->stage = goal->execute ?
      "CheckerBoard segmented motion starting" :
      "CheckerBoard segmented planning starting (plan-only)";
    feedback->progress = 0.0f;
    goal_handle->publish_feedback(feedback);

    // Per-call TCP CSV logging is opt-in (goal->enable_tcp_log). CheckerBoard
    // drives the arm through a grid of poses computed inside
    // MoveItExecutor::checkerBoard() — the server itself never sees the
    // individual target poses, only progress strings via feedback_cb. So
    // unlike the other loggers here, there is no per-stage "set pose": we
    // just tag the stage from the feedback text and keep sampling actual TCP
    // pose (set pose stays at whatever was seeded at call start — see
    // codex.md section 9's explicit "để set pose rỗng nhưng không crash"
    // allowance).
    std::shared_ptr<robot_task_manager::PerCallTcpLogger::Call> tcp_call;
    if (tcp_logger_ && goal->enable_tcp_log) {
      std::ostringstream meta;
      meta << "{\"step\":" << goal->step
           << ",\"velocity_scale\":" << goal->velocity_scale
           << ",\"execute\":" << (goal->execute ? "true" : "false") << "}";
      tcp_call = tcp_logger_->startCall(meta.str());
      if (tcp_call) {
        tcp_logger_->logEvent(tcp_call, "checker_board_start", "received", "CheckerBoard goal accepted");
        tcp_logger_->startSampling(tcp_call);
      }
    }

    std::string error_msg;
    bool ok = false;

    try {
      RCLCPP_INFO(this->get_logger(), "Calling executor_->CheckerBoard()");
      ok = executor_->checkerBoard(
        goal->step,
        error_msg,
        goal->velocity_scale,
        0.3,
        5.0,
        goal->execute,
        measurement_settle_time_s_,
        [this, goal_handle, tcp_call](const std::string & stage, float progress)
        {
          if (!goal_handle || goal_handle->is_canceling()) {
            return;
          }
          auto feedback = std::make_shared<CheckerBoard::Feedback>();
          feedback->stage = stage;
          feedback->progress = progress;
          goal_handle->publish_feedback(feedback);
          RCLCPP_INFO(
            this->get_logger(),
            "[CheckerBoard feedback] %s | %.1f%%",
            stage.c_str(),
            progress);
          if (tcp_logger_ && tcp_call) {
            tcp_logger_->setStage(tcp_call, stage);
            tcp_logger_->logEvent(tcp_call, stage, "progress", stage);
          }
        });
      RCLCPP_INFO(this->get_logger(), "Returned from CheckerBoard(), ok=%s", ok ? "true" : "false");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in CheckerBoard: %s", e.what());
      result->success = false;
      result->message = std::string("Exception: ") + e.what();
      if (tcp_logger_ && tcp_call) {
        tcp_logger_->finishCall(tcp_call, "aborted", false, result->message);
      }
      goal_handle->abort(result);
      return;
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown exception in CheckerBoard");
      result->success = false;
      result->message = "Unknown exception in CheckerBoard";
      if (tcp_logger_ && tcp_call) {
        tcp_logger_->finishCall(tcp_call, "aborted", false, result->message);
      }
      goal_handle->abort(result);
      return;
    }

    if (goal_handle->is_canceling()) {
      RCLCPP_WARN(this->get_logger(), "Goal canceled during execution");
      result->success = false;
      result->message = "Goal canceled";
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
      if (tcp_logger_ && tcp_call) {
        tcp_logger_->logEvent(tcp_call, "checker_board_end", "stage_failed", result->message);
        tcp_logger_->finishCall(tcp_call, "aborted", false, result->message);
      }
      goal_handle->abort(result);
      return;
    }

    feedback->stage = goal->execute ?
      "CheckerBoard segmented motion completed" :
      "CheckerBoard segmented plan validated (execution skipped)";
    feedback->progress = 100.0f;
    goal_handle->publish_feedback(feedback);

    result->success = true;
    result->message = goal->execute ?
      "CheckerBoard segmented motion completed successfully" :
      "CheckerBoard segmented planning success; execution skipped because execute=false";
    if (tcp_logger_ && tcp_call) {
      tcp_logger_->logEvent(tcp_call, "checker_board_end", "checker_board_end", result->message);
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
  auto node = std::make_shared<CheckerBoardActionServer>();
  node->initialize_moveit();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
