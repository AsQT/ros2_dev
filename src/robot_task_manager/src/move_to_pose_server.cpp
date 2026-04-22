#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_task_manager/action/move_to_pose.hpp"
#include "robot_task_manager/moveit_executor.hpp"

class MoveToPoseActionServer : public rclcpp::Node
{
public:
  using MoveToPose = robot_task_manager::action::MoveToPose;
  using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;

  MoveToPoseActionServer(): Node("move_to_pose_action_server")
  {
    planning_group_ = declare_parameter<std::string>("planning_group", "arm");
    base_frame_ = declare_parameter<std::string>("base_frame", "world");

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
  }

private:
  std::string planning_group_;
  std::string base_frame_;

  std::shared_ptr<robot_task_manager::MoveItExecutor> executor_;
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
                                const rclcpp_action::GoalUUID &,
                                std::shared_ptr<const MoveToPose::Goal> goal)
  {
    if (goal->velocity_scale <= 0.0 || goal->velocity_scale > 1.0) {
      RCLCPP_WARN(get_logger(), "Reject goal because velocity_scale must be in (0,1]");
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

    feedback->stage = "Planning to pose";
    feedback->progress = 30.0f;
    goal_handle->publish_feedback(feedback);

    const auto goal = goal_handle->get_goal();

    std::string error_msg;
    const bool ok = executor_->moveToPose(
                                goal->target_pose,
                                error_msg,
                                goal->velocity_scale,
                                0.3,
                                5.0);

    if (!ok) {
      result->success = false;
      result->message = error_msg;
      goal_handle->abort(result);
      return;
    }

    feedback->stage = "Pose reached";
    feedback->progress = 100.0f;
    goal_handle->publish_feedback(feedback);

    result->success = true;
    result->message = "Robot reached target pose successfully";
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
