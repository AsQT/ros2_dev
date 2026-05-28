#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose.hpp"

#include "robot_task_manager/action/pick_place.hpp"
#include "robot_task_manager/action/move_to_pose.hpp"
#include "robot_task_manager/action/move_to_pose_cartesian.hpp"
#include "robot_task_manager/action/move_gripper.hpp"

using namespace std::chrono_literals;

class PickPlaceActionServer : public rclcpp::Node
{
public:
  using PickPlace = robot_task_manager::action::PickPlace;
  using PickPlaceGoalHandle = rclcpp_action::ServerGoalHandle<PickPlace>;

  using MoveToPose = robot_task_manager::action::MoveToPose;
  using MoveToPoseGoalHandle = rclcpp_action::ClientGoalHandle<MoveToPose>;

  using MoveToPoseCartesian = robot_task_manager::action::MoveToPoseCartesian;
  using MoveToPoseCartesianGoalHandle =
    rclcpp_action::ClientGoalHandle<MoveToPoseCartesian>;

  using MoveGripper = robot_task_manager::action::MoveGripper;
  using MoveGripperGoalHandle = rclcpp_action::ClientGoalHandle<MoveGripper>;

  PickPlaceActionServer()
  : Node("pickplace_action_server")
  {
    approach_height_ = declare_parameter<double>("approach_height", 0.10);
    open_gripper_position_ = declare_parameter<double>("open_gripper_position", 0.048);
    server_wait_timeout_s_ = declare_parameter<double>("server_wait_timeout_s", 5.0);
    action_result_timeout_s_ = declare_parameter<double>("action_result_timeout_s", 90.0);

    move_to_pose_client_ =
      rclcpp_action::create_client<MoveToPose>(this, "move_to_pose");

    move_to_pose_cartesian_client_ =
      rclcpp_action::create_client<MoveToPoseCartesian>(this, "move_to_pose_cartesian");

    move_gripper_client_ =
      rclcpp_action::create_client<MoveGripper>(this, "move_gripper");

    action_server_ = rclcpp_action::create_server<PickPlace>(
      this,
      "pickplace",
      std::bind(&PickPlaceActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PickPlaceActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&PickPlaceActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "PickPlace action server ready: /pickplace");
  }

private:
  double approach_height_ = 0.10;
  double open_gripper_position_ = 0.048;
  double server_wait_timeout_s_ = 5.0;
  double action_result_timeout_s_ = 90.0;

  rclcpp_action::Server<PickPlace>::SharedPtr action_server_;

  rclcpp_action::Client<MoveToPose>::SharedPtr move_to_pose_client_;
  rclcpp_action::Client<MoveToPoseCartesian>::SharedPtr move_to_pose_cartesian_client_;
  rclcpp_action::Client<MoveGripper>::SharedPtr move_gripper_client_;

  std::mutex active_goal_mutex_;

  MoveToPoseGoalHandle::SharedPtr active_move_to_pose_goal_;
  MoveToPoseCartesianGoalHandle::SharedPtr active_move_to_pose_cartesian_goal_;
  MoveGripperGoalHandle::SharedPtr active_move_gripper_goal_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const PickPlace::Goal> goal)
  {
    if (!std::isfinite(goal->velocity_scale) ||
        goal->velocity_scale <= 0.0 ||
        goal->velocity_scale > 1.0)
    {
      RCLCPP_WARN(get_logger(), "Reject PickPlace goal: velocity_scale must be in (0, 1]");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (!std::isfinite(goal->gripper) || goal->gripper < 0.0) {
      RCLCPP_WARN(get_logger(), "Reject PickPlace goal: gripper invalid");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<PickPlaceGoalHandle>)
  {
    RCLCPP_WARN(get_logger(), "PickPlace cancel requested");

    std::lock_guard<std::mutex> lock(active_goal_mutex_);

    if (active_move_to_pose_goal_) {
      move_to_pose_client_->async_cancel_goal(active_move_to_pose_goal_);
    }

    if (active_move_to_pose_cartesian_goal_) {
      move_to_pose_cartesian_client_->async_cancel_goal(active_move_to_pose_cartesian_goal_);
    }

    if (active_move_gripper_goal_) {
      move_gripper_client_->async_cancel_goal(active_move_gripper_goal_);
    }

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<PickPlaceGoalHandle> goal_handle)
  {
    std::thread(&PickPlaceActionServer::execute, this, goal_handle).detach();
  }

  void publish_feedback(
    const std::shared_ptr<PickPlaceGoalHandle> & goal_handle,
    const std::string & stage,
    float progress)
  {
    auto feedback = std::make_shared<PickPlace::Feedback>();
    feedback->stage = stage;
    feedback->progress = progress;

    goal_handle->publish_feedback(feedback);

    RCLCPP_INFO(
      get_logger(),
      "[PickPlace] %s | %.1f%%",
      stage.c_str(),
      progress);
  }

  bool check_cancel(
    const std::shared_ptr<PickPlaceGoalHandle> & goal_handle,
    const std::shared_ptr<PickPlace::Result> & result)
  {
    if (!goal_handle->is_canceling()) {
      return false;
    }

    result->success = false;
    result->message = "PickPlace canceled";
    goal_handle->canceled(result);

    RCLCPP_WARN(get_logger(), "PickPlace canceled");
    return true;
  }

  void clear_active_goals()
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    active_move_to_pose_goal_.reset();
    active_move_to_pose_cartesian_goal_.reset();
    active_move_gripper_goal_.reset();
  }

  void abort_goal(
    const std::shared_ptr<PickPlaceGoalHandle> & goal_handle,
    const std::shared_ptr<PickPlace::Result> & result,
    const std::string & message)
  {
    clear_active_goals();

    result->success = false;
    result->message = message;

    RCLCPP_ERROR(get_logger(), "PickPlace failed: %s", message.c_str());

    goal_handle->abort(result);
  }

  bool wait_for_sub_action_servers(std::string & error_msg)
  {
    auto timeout =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(server_wait_timeout_s_));

    if (!move_gripper_client_->wait_for_action_server(timeout)) {
      error_msg = "MoveGripper server not available: /move_gripper";
      return false;
    }

    if (!move_to_pose_client_->wait_for_action_server(timeout)) {
      error_msg = "MoveToPose server not available: /move_to_pose";
      return false;
    }

    if (!move_to_pose_cartesian_client_->wait_for_action_server(timeout)) {
      error_msg = "MoveToPoseCartesian server not available: /move_to_pose_cartesian";
      return false;
    }

    return true;
  }

  bool call_move_gripper(
    double position,
    std::string & error_msg)
  {
    MoveGripper::Goal goal;
    goal.position = position;

    auto result_timeout =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(action_result_timeout_s_));

    auto goal_handle_future = move_gripper_client_->async_send_goal(goal);

    if (goal_handle_future.wait_for(result_timeout) != std::future_status::ready) {
      error_msg = "Timeout while sending MoveGripper goal";
      return false;
    }

    auto goal_handle = goal_handle_future.get();

    if (!goal_handle) {
      error_msg = "MoveGripper goal rejected";
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(active_goal_mutex_);
      active_move_gripper_goal_ = goal_handle;
    }

    auto result_future = move_gripper_client_->async_get_result(goal_handle);

    if (result_future.wait_for(result_timeout) != std::future_status::ready) {
      error_msg = "Timeout while waiting MoveGripper result";
      return false;
    }

    auto wrapped_result = result_future.get();

    {
      std::lock_guard<std::mutex> lock(active_goal_mutex_);
      active_move_gripper_goal_.reset();
    }

    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      error_msg =
        "MoveGripper failed, result code = " +
        std::to_string(static_cast<int>(wrapped_result.code));
      return false;
    }

    if (!wrapped_result.result) {
      error_msg = "MoveGripper result is null";
      return false;
    }

    if (!wrapped_result.result->success) {
      error_msg = wrapped_result.result->message;
      return false;
    }

    return true;
  }

  bool call_move_to_pose(
    const geometry_msgs::msg::Pose & target_pose,
    double velocity_scale,
    std::string & error_msg)
  {
    MoveToPose::Goal goal;
    goal.target_pose = target_pose;
    goal.velocity_scale = velocity_scale;

    auto result_timeout =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(action_result_timeout_s_));

    auto goal_handle_future = move_to_pose_client_->async_send_goal(goal);

    if (goal_handle_future.wait_for(result_timeout) != std::future_status::ready) {
      error_msg = "Timeout while sending MoveToPose goal";
      return false;
    }

    auto goal_handle = goal_handle_future.get();

    if (!goal_handle) {
      error_msg = "MoveToPose goal rejected";
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(active_goal_mutex_);
      active_move_to_pose_goal_ = goal_handle;
    }

    auto result_future = move_to_pose_client_->async_get_result(goal_handle);

    if (result_future.wait_for(result_timeout) != std::future_status::ready) {
      error_msg = "Timeout while waiting MoveToPose result";
      return false;
    }

    auto wrapped_result = result_future.get();

    {
      std::lock_guard<std::mutex> lock(active_goal_mutex_);
      active_move_to_pose_goal_.reset();
    }

    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      error_msg =
        "MoveToPose failed, result code = " +
        std::to_string(static_cast<int>(wrapped_result.code));
      return false;
    }

    if (!wrapped_result.result) {
      error_msg = "MoveToPose result is null";
      return false;
    }

    if (!wrapped_result.result->success) {
      error_msg = wrapped_result.result->message;
      return false;
    }

    return true;
  }

  bool call_move_to_pose_cartesian(
    const geometry_msgs::msg::Pose & target_pose,
    double velocity_scale,
    std::string & error_msg)
  {
    MoveToPoseCartesian::Goal goal;
    goal.target_pose = target_pose;
    goal.velocity_scale = velocity_scale;

    auto result_timeout =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(action_result_timeout_s_));

    auto goal_handle_future =
      move_to_pose_cartesian_client_->async_send_goal(goal);

    if (goal_handle_future.wait_for(result_timeout) != std::future_status::ready) {
      error_msg = "Timeout while sending MoveToPoseCartesian goal";
      return false;
    }

    auto goal_handle = goal_handle_future.get();

    if (!goal_handle) {
      error_msg = "MoveToPoseCartesian goal rejected";
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(active_goal_mutex_);
      active_move_to_pose_cartesian_goal_ = goal_handle;
    }

    auto result_future =
      move_to_pose_cartesian_client_->async_get_result(goal_handle);

    if (result_future.wait_for(result_timeout) != std::future_status::ready) {
      error_msg = "Timeout while waiting MoveToPoseCartesian result";
      return false;
    }

    auto wrapped_result = result_future.get();

    {
      std::lock_guard<std::mutex> lock(active_goal_mutex_);
      active_move_to_pose_cartesian_goal_.reset();
    }

    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      error_msg =
        "MoveToPoseCartesian failed, result code = " +
        std::to_string(static_cast<int>(wrapped_result.code));
      return false;
    }

    if (!wrapped_result.result) {
      error_msg = "MoveToPoseCartesian result is null";
      return false;
    }

    if (!wrapped_result.result->success) {
      error_msg = wrapped_result.result->message;
      return false;
    }

    return true;
  }

  void execute(
    const std::shared_ptr<PickPlaceGoalHandle> goal_handle)
  {
    auto result = std::make_shared<PickPlace::Result>();

    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "goal_handle is null");
      return;
    }

    auto goal = goal_handle->get_goal();

    if (!goal) {
      result->success = false;
      result->message = "Goal is null";
      goal_handle->abort(result);
      return;
    }

    std::string error_msg;

    publish_feedback(goal_handle, "Waiting for sub action servers", 2.0f);

    if (!wait_for_sub_action_servers(error_msg)) {
      abort_goal(goal_handle, result, error_msg);
      return;
    }

    if (check_cancel(goal_handle, result)) {
      return;
    }

    geometry_msgs::msg::Pose pick_approach = goal->pose_pick;
    pick_approach.position.z += approach_height_;

    geometry_msgs::msg::Pose place_approach = goal->pose_place;
    place_approach.position.z += approach_height_;

    RCLCPP_INFO(
      get_logger(),
      "PickPlace start | pick=(%.3f %.3f %.3f) | place=(%.3f %.3f %.3f) | gripper=%.4f | vel=%.2f",
      goal->pose_pick.position.x,
      goal->pose_pick.position.y,
      goal->pose_pick.position.z,
      goal->pose_place.position.x,
      goal->pose_place.position.y,
      goal->pose_place.position.z,
      goal->gripper,
      goal->velocity_scale);

    // 1. Open gripper
    publish_feedback(goal_handle, "Open gripper", 5.0f);

    if (!call_move_gripper(open_gripper_position_, error_msg)) {
      abort_goal(goal_handle, result, "Open gripper failed: " + error_msg);
      return;
    }

    if (check_cancel(goal_handle, result)) {
      return;
    }

    // 2. MoveToPose to pick approach
    publish_feedback(goal_handle, "Move to pick approach", 15.0f);

    if (!call_move_to_pose(pick_approach, goal->velocity_scale, error_msg)) {
      abort_goal(goal_handle, result, "Move to pick approach failed: " + error_msg);
      return;
    }

    if (check_cancel(goal_handle, result)) {
      return;
    }

    // 3. Cartesian down to pick
    publish_feedback(goal_handle, "Cartesian down to pick", 30.0f);

    if (!call_move_to_pose_cartesian(goal->pose_pick, goal->velocity_scale, error_msg)) {
      abort_goal(goal_handle, result, "Cartesian down to pick failed: " + error_msg);
      return;
    }

    if (check_cancel(goal_handle, result)) {
      return;
    }

    // 4. Close gripper
    publish_feedback(goal_handle, "Close gripper", 45.0f);

    if (!call_move_gripper(goal->gripper, error_msg)) {
      abort_goal(goal_handle, result, "Close gripper failed: " + error_msg);
      return;
    }

    if (check_cancel(goal_handle, result)) {
      return;
    }

    // 5. Cartesian lift from pick
    publish_feedback(goal_handle, "Cartesian lift from pick", 55.0f);

    if (!call_move_to_pose_cartesian(pick_approach, goal->velocity_scale, error_msg)) {
      abort_goal(goal_handle, result, "Cartesian lift from pick failed: " + error_msg);
      return;
    }

    if (check_cancel(goal_handle, result)) {
      return;
    }

    // 6. MoveToPose to place approach
    publish_feedback(goal_handle, "Move to place approach", 68.0f);

    if (!call_move_to_pose(place_approach, goal->velocity_scale, error_msg)) {
      abort_goal(goal_handle, result, "Move to place approach failed: " + error_msg);
      return;
    }

    if (check_cancel(goal_handle, result)) {
      return;
    }

    // 7. Cartesian down to place
    publish_feedback(goal_handle, "Cartesian down to place", 80.0f);

    if (!call_move_to_pose_cartesian(goal->pose_place, goal->velocity_scale, error_msg)) {
      abort_goal(goal_handle, result, "Cartesian down to place failed: " + error_msg);
      return;
    }

    if (check_cancel(goal_handle, result)) {
      return;
    }

    // 8. Open gripper to release
    publish_feedback(goal_handle, "Open gripper to release", 90.0f);

    if (!call_move_gripper(open_gripper_position_, error_msg)) {
      abort_goal(goal_handle, result, "Release object failed: " + error_msg);
      return;
    }

    if (check_cancel(goal_handle, result)) {
      return;
    }

    // 9. Cartesian lift from place
    publish_feedback(goal_handle, "Cartesian lift from place", 97.0f);

    if (!call_move_to_pose_cartesian(place_approach, goal->velocity_scale, error_msg)) {
      abort_goal(goal_handle, result, "Cartesian lift from place failed: " + error_msg);
      return;
    }

    publish_feedback(goal_handle, "PickPlace completed", 100.0f);

    result->success = true;
    result->message = "PickPlace completed successfully";

    clear_active_goals();

    goal_handle->succeed(result);

    RCLCPP_INFO(get_logger(), "PickPlace completed successfully");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PickPlaceActionServer>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}