#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_task_manager/action/go_home.hpp"
#include "robot_task_manager/action/move_to_pose.hpp"
#include "robot_task_manager/action/move_to_pose_cartesian.hpp"
#include "robot_task_manager/action/checker_board.hpp"


class robotTaskManagerClient : public rclcpp::Node
{
public:
  using GoHome                          = robot_task_manager::action::GoHome;
  using GoHomeGoalHandle                = rclcpp_action::ClientGoalHandle<GoHome>;

  using MoveToPose                      = robot_task_manager::action::MoveToPose;
  using MoveToPoseGoalHandle            = rclcpp_action::ClientGoalHandle<MoveToPose>;

  using MoveToPoseCartesian             = robot_task_manager::action::MoveToPoseCartesian ;
  using MoveToPoseCartesianGoalHandle   = rclcpp_action::ClientGoalHandle<MoveToPoseCartesian>;
  
  using CheckerBoard                    = robot_task_manager::action::CheckerBoard ;
  using CheckerBoardGoalHandle          = rclcpp_action::ClientGoalHandle<CheckerBoard>;

  robotTaskManagerClient()
  : Node("task_manager_client")
  {
    declare_parameter<std::string>("task_name", "gohome");
    gohome_client_                  = rclcpp_action::create_client<GoHome>(this, "gohome");
    move_to_pose_client_            = rclcpp_action::create_client<MoveToPose>(this, "move_to_pose");
    move_to_pose_cartesian_client_  = rclcpp_action::create_client<MoveToPoseCartesian>(this, "move_to_pose_cartesian");
    move_checker_board_client_      = rclcpp_action::create_client<CheckerBoard>(this, "checker_board");

  }


  void run()
  {
    const auto task_name = get_parameter("task_name").as_string();

    if (task_name == "gohome") {
      send_gohome();
    } else if (task_name == "move_to_pose") {
      send_move_to_pose();
    } else if (task_name == "move_to_pose_cartesian") {
      send_move_to_pose_cartesian();
    } else if (task_name == "checker_board") {
      send_checker_board();
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown task_name: %s", task_name.c_str());
      rclcpp::shutdown();   } }

private:
  rclcpp_action::Client<GoHome>::SharedPtr gohome_client_;
  rclcpp_action::Client<MoveToPose>::SharedPtr move_to_pose_client_;
  rclcpp_action::Client<MoveToPoseCartesian>::SharedPtr move_to_pose_cartesian_client_;
  rclcpp_action::Client<CheckerBoard>::SharedPtr move_checker_board_client_;
  /*___________________________________________________________________________*/
  void send_gohome()
  {
    if (!gohome_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "GoHome server not available");
      rclcpp::shutdown();
      return;    }

    GoHome::Goal goal;
    goal.start = true;

    rclcpp_action::Client<GoHome>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const GoHomeGoalHandle::SharedPtr & handle)
      {
        if (!handle) {
          RCLCPP_ERROR(get_logger(), "GoHome goal rejected");
        } else {
          RCLCPP_INFO(get_logger(), "GoHome goal accepted");  }  };

    options.feedback_callback =
      [this](GoHomeGoalHandle::SharedPtr, const std::shared_ptr<const GoHome::Feedback> feedback)
      {
        RCLCPP_INFO(
          get_logger(),
          "[GoHome feedback] %s | %.1f%%",
          feedback->current_step.c_str(),
          feedback->progress);  };

    options.result_callback =
      [this](const GoHomeGoalHandle::WrappedResult & result)
      {
        RCLCPP_INFO(get_logger(), "GoHome result code = %d", static_cast<int>(result.code));
        if (result.result) {
          RCLCPP_INFO(get_logger(), "message: %s", result.result->message.c_str());
        }
        rclcpp::shutdown();  };
    gohome_client_->async_send_goal(goal, options);
  }
  /*___________________________________________________________________________*/
  void send_move_to_pose()
  {
    /* _____ Check action server _____*/
    if (!move_to_pose_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "MoveToPose server not available");
      rclcpp::shutdown();
      return;  }
    /* ___ setup giá trij ban đầu ____*/
    MoveToPose::Goal goal;
    goal.target_pose.position.x = 0.40;
    goal.target_pose.position.y = 0.10;
    goal.target_pose.position.z = 0.35;
    goal.target_pose.orientation.x = 0.0;
    goal.target_pose.orientation.y = 0.0;
    goal.target_pose.orientation.z = 0.0;
    goal.target_pose.orientation.w = 1.0;
    goal.velocity_scale = 0.5;

    rclcpp_action::Client<MoveToPose>::SendGoalOptions options;
    options.goal_response_callback = [this](const MoveToPoseGoalHandle::SharedPtr & handle)
      {
        if (!handle) {
          RCLCPP_ERROR(get_logger(), "MoveToPose goal rejected");
        } else {
          RCLCPP_INFO(get_logger(), "MoveToPose goal accepted");  }  };

    options.feedback_callback =    [this](MoveToPoseGoalHandle::SharedPtr, 
            const std::shared_ptr<const MoveToPose::Feedback> feedback)
      {
        RCLCPP_INFO(
          get_logger(),
          "[MoveToPose feedback] %s | %.1f%%",
          feedback->stage.c_str(),
          feedback->progress);   };

    options.result_callback =    [this](const MoveToPoseGoalHandle::WrappedResult & result)
      {
        RCLCPP_INFO(get_logger(), "MoveToPose result code = %d", static_cast<int>(result.code));
        if (result.result) {
          RCLCPP_INFO(get_logger(), "message: %s", result.result->message.c_str());
        }
        rclcpp::shutdown(); };

    move_to_pose_client_->async_send_goal(goal, options);
  }
  /*___________________________________________________________________________*/
  void send_move_to_pose_cartesian()
  {
    if (!move_to_pose_cartesian_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "MoveToPoseCartesian server not available");
      rclcpp::shutdown();
      return;  }

    MoveToPoseCartesian::Goal goal;
    goal.target_pose.position.x = 0.40;
    goal.target_pose.position.y = 0.10;
    goal.target_pose.position.z = 0.35;
    goal.target_pose.orientation.x = 0.0;
    goal.target_pose.orientation.y = 0.0;
    goal.target_pose.orientation.z = 0.0;
    goal.target_pose.orientation.w = 1.0;
    goal.velocity_scale = 0.5;

    rclcpp_action::Client<MoveToPoseCartesian>::SendGoalOptions options;
    options.goal_response_callback =   [this](const MoveToPoseCartesianGoalHandle::SharedPtr & handle)
      {
        if (!handle) {
          RCLCPP_ERROR(get_logger(), "MoveToPoseCartesian  goal rejected");
        } else {
          RCLCPP_INFO(get_logger(), "MoveToPoseCartesian  goal accepted");  }  };

    options.feedback_callback =   [this](MoveToPoseCartesianGoalHandle::SharedPtr, 
            const std::shared_ptr<const MoveToPoseCartesian::Feedback> feedback)
      {
        RCLCPP_INFO(
          get_logger(),
          "[MoveToPoseCartesian  feedback] %s | %.1f%%",
          feedback->stage.c_str(),
          feedback->progress);   };

    options.result_callback = [this](const MoveToPoseCartesianGoalHandle::WrappedResult & result)
      {
        RCLCPP_INFO(get_logger(), "MoveToPoseCartesian  result code = %d", static_cast<int>(result.code));
        if (result.result) {
          RCLCPP_INFO(get_logger(), "message: %s", result.result->message.c_str());
        }
        rclcpp::shutdown(); };
    move_to_pose_cartesian_client_->async_send_goal(goal, options);
  }

  /*___________________________________________________________________________*/
  void send_checker_board()
  {
    if (!move_checker_board_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "CheckerBoard server not available");
      rclcpp::shutdown();
      return;  }

    CheckerBoard::Goal goal;
    goal.step = 0.40;
    goal.velocity_scale = 0.5;

    rclcpp_action::Client<CheckerBoard>::SendGoalOptions options;
    options.goal_response_callback = [this](const CheckerBoardGoalHandle::SharedPtr & handle)
      {
        if (!handle) {
          RCLCPP_ERROR(get_logger(), "CheckerBoard  goal rejected");
        } else {
          RCLCPP_INFO(get_logger(), "CheckerBoard  goal accepted");  }  };

    options.feedback_callback =   [this](CheckerBoardGoalHandle::SharedPtr, 
                                    const std::shared_ptr<const CheckerBoard::Feedback> feedback)
      {
        RCLCPP_INFO(
          get_logger(),
          "[CheckerBoard  feedback] %s | %.1f%%",
          feedback->stage.c_str(),
          feedback->progress);   };

    options.result_callback =   [this](const CheckerBoardGoalHandle::WrappedResult & result)
      {
        RCLCPP_INFO(get_logger(), "CheckerBoard  result code = %d", static_cast<int>(result.code));
        if (result.result) {
          RCLCPP_INFO(get_logger(), "message: %s", result.result->message.c_str());
        }
        rclcpp::shutdown(); };

    move_checker_board_client_->async_send_goal(goal, options);
  }

};
/*___________________________________________________________________________*/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robotTaskManagerClient>();
  node->run();
  rclcpp::spin(node);
  return 0;
}
