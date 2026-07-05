#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_task_manager/action/move_gripper.hpp"
#include "robot_task_manager/gripper_executor.hpp"
#include "robot_task_manager/log_paths.hpp"
#include "robot_task_manager/standard_action_logger.hpp"

class MoveGripperActionServer : public rclcpp::Node
{
public:
  using MoveGripper = robot_task_manager::action::MoveGripper;
  using GoalHandleMoveGripper = rclcpp_action::ServerGoalHandle<MoveGripper>;

  MoveGripperActionServer()
  : Node("move_gripper_action_server")
  {
    planning_group_ = declare_parameter<std::string>("planning_group", "gripper");
    base_frame_ = declare_parameter<std::string>("base_frame", "link_6");

    // Theo MoveGripper.action: goal chỉ có position.
    // Velocity/acceleration scale lấy bằng parameter nội bộ của server.
    default_velocity_scale_ = declare_parameter<double>("velocity_scale", 0.1);
    default_acceleration_scale_ = declare_parameter<double>("acceleration_scale", 0.5);
    enable_standard_logging_ = declare_parameter<bool>("enable_standard_logging", true);
    log_root_dir_ = declare_parameter<std::string>("log_root_dir", robot_task_manager::kDefaultLogRootDir);
    runtime_mode_ = declare_parameter<std::string>("runtime_mode", "mock");

    action_server_ = rclcpp_action::create_server<MoveGripper>(
      this,
      "move_gripper",
      std::bind(&MoveGripperActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveGripperActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MoveGripperActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "MoveGripper action server ready: /move_gripper");
  }

  void initialize_moveit()
  {
    gripper_executor_ = std::make_shared<robot_task_manager::GripperExecutor>();
    gripper_executor_->initialize(shared_from_this(), planning_group_, base_frame_);
    if (enable_standard_logging_) {
      standard_logger_ = std::make_shared<robot_task_manager::StandardActionLogger>(
        shared_from_this(), log_root_dir_, runtime_mode_, "MoveGripper",
        "", base_frame_, "", "", "", "baseline", "");
    }
  }

private:
  std::string planning_group_;
  std::string base_frame_;
  double default_velocity_scale_ = 0.1;
  double default_acceleration_scale_ = 0.5;
  bool enable_standard_logging_{true};
  std::string log_root_dir_;
  std::string runtime_mode_;

  std::shared_ptr<robot_task_manager::GripperExecutor> gripper_executor_;
  std::shared_ptr<robot_task_manager::StandardActionLogger> standard_logger_;
  rclcpp_action::Server<MoveGripper>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveGripper::Goal> goal)
  {
    // Range thật sự vẫn được GripperExecutor kiểm tra bằng gripper_min_open/gripper_max_open.
    // Ở đây chỉ reject các goal chắc chắn sai.
    if (!std::isfinite(goal->position) || goal->position < 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "Reject MoveGripper goal because position is invalid: %.6f",
        goal->position);
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (default_velocity_scale_ <= 0.0 || default_velocity_scale_ > 1.0 ||
        default_acceleration_scale_ <= 0.0 || default_acceleration_scale_ > 1.0)
    {
      RCLCPP_WARN(
        get_logger(),
        "Reject MoveGripper goal because velocity_scale/acceleration_scale parameter must be in (0, 1]");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveGripper>)
  {
    RCLCPP_WARN(get_logger(), "Cancel request received for MoveGripper");

    if (gripper_executor_) {
      gripper_executor_->stop();
    }

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveGripper> goal_handle)
  {
    std::thread(&MoveGripperActionServer::execute, this, goal_handle).detach();
  }

  void execute(const std::shared_ptr<GoalHandleMoveGripper> goal_handle)
  {
    auto feedback = std::make_shared<MoveGripper::Feedback>();
    auto result = std::make_shared<MoveGripper::Result>();

    const auto goal = goal_handle->get_goal();
    auto log_call = standard_logger_ ? standard_logger_->startCall() : nullptr;

    feedback->stage = goal->execute ? "Planning gripper motion" : "Planning gripper motion (plan-only)";
    feedback->progress = 30.0f;
    goal_handle->publish_feedback(feedback);

    std::string error_msg;
    const bool ok = gripper_executor_->moveToOpening(
      goal->position,
      error_msg,
      default_velocity_scale_,
      default_acceleration_scale_,
      goal->execute);

    if (goal_handle->is_canceling()) {
      if (gripper_executor_) {
        gripper_executor_->stop();
      }

      result->success = false;
      result->message = "MoveGripper action canceled";
      if (standard_logger_ && log_call) {
        robot_task_manager::StandardSummary summary;
        summary.execute_requested = goal->execute;
        summary.success = false;
        summary.failed_stage = "canceled";
        summary.failure_reason = "canceled";
        summary.message = result->message;
        summary.total_time_s = (now() - log_call->start_time).seconds();
        summary.extra_header = "target_opening_mm,actual_opening_mm,motion_time_s";
        summary.extra_values = robot_task_manager::standardFormatDouble(goal->position * 1000.0) + ",," +
          robot_task_manager::standardFormatDouble(summary.total_time_s);
        standard_logger_->writeSummary(*log_call, summary);
        standard_logger_->appendEvent(*log_call, "execute", "action_result", false, result->message, summary.total_time_s);
      }
      goal_handle->canceled(result);
      return;
    }

    if (!ok) {
      result->success = false;
      result->message = error_msg;
      if (standard_logger_ && log_call) {
        robot_task_manager::StandardSummary summary;
        summary.execute_requested = goal->execute;
        summary.success = false;
        summary.failed_stage = "move_to_opening";
        summary.failure_reason = "executor_failed";
        summary.message = result->message;
        summary.total_time_s = (now() - log_call->start_time).seconds();
        summary.extra_header = "target_opening_mm,actual_opening_mm,motion_time_s";
        summary.extra_values = robot_task_manager::standardFormatDouble(goal->position * 1000.0) + ",," +
          robot_task_manager::standardFormatDouble(summary.total_time_s);
        standard_logger_->writeSummary(*log_call, summary);
        standard_logger_->appendEvent(*log_call, "move_to_opening", "action_result", false, result->message, summary.total_time_s);
      }
      goal_handle->abort(result);
      return;
    }

    feedback->stage = goal->execute ? "Gripper position reached" : "Gripper plan validated (execution skipped)";
    feedback->progress = 100.0f;
    goal_handle->publish_feedback(feedback);

    result->success = true;
    result->message = goal->execute ?
      "Gripper moved successfully" :
      "MoveGripper planning success; execution skipped";
    if (standard_logger_ && log_call) {
      robot_task_manager::StandardSummary summary;
      summary.execute_requested = goal->execute;
      summary.success = true;
      summary.message = result->message;
      summary.total_time_s = (now() - log_call->start_time).seconds();
      summary.extra_header = "target_opening_mm,actual_opening_mm,motion_time_s";
      summary.extra_values = robot_task_manager::standardFormatDouble(goal->position * 1000.0) + ",," +
        robot_task_manager::standardFormatDouble(summary.total_time_s);
      standard_logger_->writeSummary(*log_call, summary);
      standard_logger_->appendEvent(*log_call, "move_to_opening", "action_result", true, result->message, summary.total_time_s);
    }
    goal_handle->succeed(result);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MoveGripperActionServer>();
  node->initialize_moveit();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
