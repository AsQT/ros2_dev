#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_task_manager/action/move_to_pose_cartesian.hpp"
#include "robot_task_manager/moveit_executor.hpp"

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
  }

private:
  std::string planning_group_;
  std::string base_frame_;

  std::shared_ptr<robot_task_manager::MoveItExecutor> executor_;
  rclcpp_action::Server<MoveToPoseCartesian>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveToPoseCartesian::Goal> goal)
  {
    if (goal->velocity_scale <= 0.0 || goal->velocity_scale > 1.0) {
      RCLCPP_WARN(get_logger(), "Reject goal because velocity_scale must be in (0,1]");
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

    feedback->stage = "Planning to pose";
    feedback->progress = 30.0f;
    goal_handle->publish_feedback(feedback);

    auto goal = goal_handle->get_goal();
    if (!goal) {
      RCLCPP_ERROR(this->get_logger(), "Goal is null");
      result->success = false;
      result->message = "Internal error: goal is null";
      goal_handle->abort(result);
      return;
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
        5.0);
      RCLCPP_INFO(this->get_logger(), "Returned from moveToPoseCartesian(), ok=%s", ok ? "true" : "false");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in moveToPoseCartesian: %s", e.what());
      result->success = false;
      result->message = std::string("Exception: ") + e.what();
      goal_handle->abort(result);
      return;
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown exception in moveToPoseCartesian");
      result->success = false;
      result->message = "Unknown exception in moveToPoseCartesian";
      goal_handle->abort(result);
      return;
    }

    if (goal_handle->is_canceling()) {
      RCLCPP_WARN(this->get_logger(), "Goal canceled during execution");
      result->success = false;
      result->message = "Goal canceled";
      goal_handle->canceled(result);
      return;
    }

    if (!ok) {
      result->success = false;
      result->message = error_msg.empty() ? "Cartesian motion failed" : error_msg;
      RCLCPP_ERROR(this->get_logger(), "Aborting goal: %s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    feedback->stage = "Pose reached";
    feedback->progress = 100.0f;
    goal_handle->publish_feedback(feedback);

    result->success = true;
    result->message = "Robot reached target pose successfully";

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
