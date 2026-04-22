#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_task_manager/action/go_home.hpp"
#include "robot_task_manager/moveit_executor.hpp"

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

    action_server_ = rclcpp_action::create_server<GoHome>(
                                        this,
                                        "gohome",
                                        std::bind(&GoHomeActionServer::handle_goal, 
                                                  this, std::placeholders::_1, std::placeholders::_2),
                                        std::bind(&GoHomeActionServer::handle_cancel, 
                                                  this, std::placeholders::_1),
                                        std::bind(&GoHomeActionServer::handle_accepted, 
                                                  this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "GoHome action server ready");
  }

  void initialize_moveit()
  {
    executor_ = std::make_shared<robot_task_manager::MoveItExecutor>();
    executor_->initialize(shared_from_this(), planning_group_, base_frame_);
  }

private:
  std::string planning_group_;
  std::string home_target_;
  std::string base_frame_;

  std::shared_ptr<robot_task_manager::MoveItExecutor> executor_;
  rclcpp_action::Server<GoHome>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
                                const rclcpp_action::GoalUUID &,
                                std::shared_ptr<const GoHome::Goal> goal)
                    {
                      if (!goal->start) {
                        RCLCPP_WARN(get_logger(), "Reject goal because start=false");
                        return rclcpp_action::GoalResponse::REJECT;
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

      feedback->current_step = "Planning to home";
      feedback->progress = 30.0f;
      goal_handle->publish_feedback(feedback);

      std::string error_msg;
      const bool ok = executor_->goNamedTarget(home_target_, error_msg);

      if (!ok) {
        result->success = false;
        result->message = error_msg;
        goal_handle->abort(result);
        return;
      }

      feedback->current_step = "Home_reached";
      feedback->progress = 100.0f;
      goal_handle->publish_feedback(feedback);

      result->success = true;
      result->message = "Robot moved to home successfully";
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
