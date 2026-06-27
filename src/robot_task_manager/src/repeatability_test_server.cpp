#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "robot_task_manager/action/move_to_pose.hpp"
#include "robot_task_manager/action/move_to_pose_cartesian.hpp"
#include "robot_task_manager/action/repeatability_test.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class RepeatabilityTestActionServer : public rclcpp::Node
{
public:
  using RepeatabilityTest = robot_task_manager::action::RepeatabilityTest;
  using GoalHandle = rclcpp_action::ServerGoalHandle<RepeatabilityTest>;

  using MoveToPose = robot_task_manager::action::MoveToPose;
  using MoveToPoseGoalHandle = rclcpp_action::ClientGoalHandle<MoveToPose>;

  using MoveToPoseCartesian = robot_task_manager::action::MoveToPoseCartesian;
  using MoveToPoseCartesianGoalHandle =
    rclcpp_action::ClientGoalHandle<MoveToPoseCartesian>;

  RepeatabilityTestActionServer()
  : Node("repeatability_test_action_server")
  {
    server_wait_timeout_s_ = declare_parameter<double>("server_wait_timeout_s", 5.0);
    action_result_timeout_s_ = declare_parameter<double>("action_result_timeout_s", 120.0);
    measurement_settle_time_s_ = declare_parameter<double>("measurement_settle_time_s", 2.0);
    fast_velocity_scale_ = declare_parameter<double>("fast_velocity_scale", 0.1);
    axis_y_tool_yaw_offset_rad_ =
      declare_parameter<double>("axis_y_tool_yaw_offset_rad", kDefaultAxisYToolYawOffsetRad);
    if (!std::isfinite(fast_velocity_scale_) ||
      fast_velocity_scale_ <= 0.0 ||
      fast_velocity_scale_ > 1.0)
    {
      RCLCPP_WARN(
        get_logger(),
        "Invalid fast_velocity_scale=%.3f, clamping to 0.1 ",
        fast_velocity_scale_);
      fast_velocity_scale_ = 0.1;
    }
    if (!std::isfinite(axis_y_tool_yaw_offset_rad_)) {
      RCLCPP_WARN(
        get_logger(),
        "Invalid axis_y_tool_yaw_offset_rad, using %.16f",
        kDefaultAxisYToolYawOffsetRad);
      axis_y_tool_yaw_offset_rad_ = kDefaultAxisYToolYawOffsetRad;
    }

    move_to_pose_client_ =
      rclcpp_action::create_client<MoveToPose>(this, "move_to_pose");

    move_to_pose_cartesian_client_ =
      rclcpp_action::create_client<MoveToPoseCartesian>(this, "move_to_pose_cartesian");

    action_server_ = rclcpp_action::create_server<RepeatabilityTest>(
      this,
      "repeatability_test",
      std::bind(&RepeatabilityTestActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RepeatabilityTestActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&RepeatabilityTestActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "RepeatabilityTest action server ready: /repeatability_test");
  }

private:
  static constexpr double kDefaultAxisYToolYawOffsetRad = -1.5707963267948966;

  double server_wait_timeout_s_ = 5.0;
  double action_result_timeout_s_ = 120.0;
  double measurement_settle_time_s_ = 2.0;
  double fast_velocity_scale_ = 0.1;
  double axis_y_tool_yaw_offset_rad_ = kDefaultAxisYToolYawOffsetRad;

  rclcpp_action::Server<RepeatabilityTest>::SharedPtr action_server_;
  rclcpp_action::Client<MoveToPose>::SharedPtr move_to_pose_client_;
  rclcpp_action::Client<MoveToPoseCartesian>::SharedPtr move_to_pose_cartesian_client_;

  std::mutex active_goal_mutex_;
  MoveToPoseGoalHandle::SharedPtr active_move_to_pose_goal_;
  MoveToPoseCartesianGoalHandle::SharedPtr active_move_to_pose_cartesian_goal_;

  static bool is_pose_valid(const geometry_msgs::msg::PoseStamped & pose)
  {
    const auto & p = pose.pose.position;
    const auto & q = pose.pose.orientation;
    const double q_norm =
      q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;

    return
      std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
      std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
      q_norm > 1e-12;
  }

  geometry_msgs::msg::Quaternion rotate_tool_yaw(
    const geometry_msgs::msg::Quaternion & input,
    double yaw_offset_rad) const
  {
    tf2::Quaternion q_input;
    tf2::fromMsg(input, q_input);

    tf2::Quaternion q_yaw_offset;
    q_yaw_offset.setRPY(0.0, 0.0, yaw_offset_rad);

    tf2::Quaternion q_result = q_input * q_yaw_offset;
    q_result.normalize();
    return tf2::toMsg(q_result);
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const RepeatabilityTest::Goal> goal)
  {
    std::string error_msg;
    if (!validate_goal(goal, error_msg)) {
      RCLCPP_WARN(get_logger(), "Reject RepeatabilityTest goal: %s", error_msg.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  bool validate_goal(
    const std::shared_ptr<const RepeatabilityTest::Goal> & goal,
    std::string & error_msg) const
  {
    if (!goal) {
      error_msg = "goal is null";
      return false;
    }

    if (goal->repeat_count <= 0) {
      error_msg = "repeat_count must be > 0";
      return false;
    }

    if (goal->axis != RepeatabilityTest::Goal::AXIS_X &&
      goal->axis != RepeatabilityTest::Goal::AXIS_Y &&
      goal->axis != RepeatabilityTest::Goal::AXIS_Z)
    {
      error_msg = "axis must be AXIS_X(0), AXIS_Y(1), or AXIS_Z(2)";
      return false;
    }

    if (!std::isfinite(goal->meas_offset) || goal->meas_offset == 0.0) {
      error_msg = "meas_offset must be finite and non-zero";
      return false;
    }

    if (!std::isfinite(goal->velocity_scale) ||
      goal->velocity_scale <= 0.0 ||
      goal->velocity_scale > 1.0)
    {
      error_msg = "velocity_scale must be in (0, 1]";
      return false;
    }

    if (!is_pose_valid(goal->retract_pose)) {
      error_msg = "retract_pose is invalid";
      return false;
    }

    if (!is_pose_valid(goal->disturb_pose_1)) {
      error_msg = "disturb_pose_1 is invalid";
      return false;
    }

    return true;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_WARN(get_logger(), "RepeatabilityTest cancel requested");

    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    if (active_move_to_pose_goal_) {
      move_to_pose_client_->async_cancel_goal(active_move_to_pose_goal_);
    }
    if (active_move_to_pose_cartesian_goal_) {
      move_to_pose_cartesian_client_->async_cancel_goal(active_move_to_pose_cartesian_goal_);
    }

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread(&RepeatabilityTestActionServer::execute, this, goal_handle).detach();
  }

  void publish_feedback(
    const std::shared_ptr<GoalHandle> & goal_handle,
    int32_t current_index,
    const std::string & current_step)
  {
    auto feedback = std::make_shared<RepeatabilityTest::Feedback>();
    feedback->current_index = current_index;
    feedback->current_step = current_step;
    goal_handle->publish_feedback(feedback);

    RCLCPP_INFO(
      get_logger(),
      "[RepeatabilityTest] loop=%d | %s",
      current_index,
      current_step.c_str());
  }

  void clear_active_goals()
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    active_move_to_pose_goal_.reset();
    active_move_to_pose_cartesian_goal_.reset();
  }

  bool check_cancel(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::shared_ptr<RepeatabilityTest::Result> & result,
    int32_t completed_count)
  {
    if (!goal_handle->is_canceling()) {
      return false;
    }

    clear_active_goals();
    result->success = false;
    result->message = "RepeatabilityTest canceled";
    result->completed_count = completed_count;
    goal_handle->canceled(result);

    RCLCPP_WARN(get_logger(), "RepeatabilityTest canceled");
    return true;
  }

  bool sleep_with_cancel(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::shared_ptr<RepeatabilityTest::Result> & result,
    int32_t completed_count)
  {
    const auto sleep_step = 100ms;
    auto remaining =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(measurement_settle_time_s_));

    while (remaining > 0ns) {
      if (check_cancel(goal_handle, result, completed_count)) {
        return false;
      }
      const auto chunk = std::min(remaining, std::chrono::duration_cast<std::chrono::nanoseconds>(sleep_step));
      std::this_thread::sleep_for(chunk);
      remaining -= chunk;
    }

    return true;
  }

  void abort_goal(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::shared_ptr<RepeatabilityTest::Result> & result,
    const std::string & message,
    int32_t completed_count)
  {
    clear_active_goals();

    result->success = false;
    result->message = message;
    result->completed_count = completed_count;

    RCLCPP_ERROR(get_logger(), "RepeatabilityTest failed: %s", message.c_str());
    goal_handle->abort(result);
  }

  bool wait_for_sub_action_servers(std::string & error_msg)
  {
    auto timeout =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(server_wait_timeout_s_));

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

  bool call_move_to_pose(
    const geometry_msgs::msg::Pose & target_pose,
    double velocity_scale,
    bool execute,
    std::string & error_msg)
  {
    MoveToPose::Goal goal;
    goal.target_pose = target_pose;
    goal.velocity_scale = velocity_scale;
    goal.execute = execute;

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
    bool execute,
    std::string & error_msg)
  {
    MoveToPoseCartesian::Goal goal;
    goal.target_pose = target_pose;
    goal.velocity_scale = velocity_scale;
    goal.execute = execute;

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

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto result = std::make_shared<RepeatabilityTest::Result>();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "goal_handle is null");
      return;
    }

    const auto goal = goal_handle->get_goal();
    std::string error_msg;
    int32_t completed_count = 0;
    const bool execute_motion = goal->execute;

    publish_feedback(
      goal_handle,
      0,
      execute_motion ? "Waiting for sub action servers" : "Waiting for sub action servers (plan-only)");
    if (!wait_for_sub_action_servers(error_msg)) {
      abort_goal(goal_handle, result, error_msg, completed_count);
      return;
    }

    if (check_cancel(goal_handle, result, completed_count)) {
      return;
    }

    geometry_msgs::msg::Pose retract_pose = goal->retract_pose.pose;
    geometry_msgs::msg::Pose disturb_pose_1 = goal->disturb_pose_1.pose;
    geometry_msgs::msg::Pose working_retract_pose = retract_pose;
    geometry_msgs::msg::Pose working_disturb_pose_1 = disturb_pose_1;

    if (goal->axis == RepeatabilityTest::Goal::AXIS_Y) {
      const auto rotated_orientation =
        rotate_tool_yaw(retract_pose.orientation, axis_y_tool_yaw_offset_rad_);
      working_retract_pose.orientation = rotated_orientation;
      working_disturb_pose_1.orientation = rotated_orientation;
    }

    geometry_msgs::msg::Pose meas_pose = working_retract_pose;

    if (goal->axis == RepeatabilityTest::Goal::AXIS_X) {
      meas_pose.position.x = working_retract_pose.position.x + goal->meas_offset;
    } else if (goal->axis == RepeatabilityTest::Goal::AXIS_Y) {
      meas_pose.position.y = working_retract_pose.position.y + goal->meas_offset;
    } else {
      meas_pose.position.z = working_retract_pose.position.z - goal->meas_offset;
    }

    RCLCPP_INFO(
      get_logger(),
      "RepeatabilityTest start | mode=%s | axis=%u | offset=%.4f | repeats=%d | meas_vel=%.2f | fast_vel=%.2f | axis_y_yaw_offset=%.6f",
      execute_motion ? "execute" : "plan-only",
      goal->axis,
      goal->meas_offset,
      goal->repeat_count,
      goal->velocity_scale,
      fast_velocity_scale_,
      axis_y_tool_yaw_offset_rad_);
    RCLCPP_INFO(
      get_logger(),
      "RepeatabilityTest working_retract_pose | position=(%.4f, %.4f, %.4f) | orientation=(%.7f, %.7f, %.7f, %.7f)",
      working_retract_pose.position.x,
      working_retract_pose.position.y,
      working_retract_pose.position.z,
      working_retract_pose.orientation.x,
      working_retract_pose.orientation.y,
      working_retract_pose.orientation.z,
      working_retract_pose.orientation.w);
    RCLCPP_INFO(
      get_logger(),
      "RepeatabilityTest meas_pose | position=(%.4f, %.4f, %.4f) | orientation=(%.7f, %.7f, %.7f, %.7f)",
      meas_pose.position.x,
      meas_pose.position.y,
      meas_pose.position.z,
      meas_pose.orientation.x,
      meas_pose.orientation.y,
      meas_pose.orientation.z,
      meas_pose.orientation.w);
    RCLCPP_INFO(
      get_logger(),
      "RepeatabilityTest working_disturb_pose_1 | position=(%.4f, %.4f, %.4f) | orientation=(%.7f, %.7f, %.7f, %.7f)",
      working_disturb_pose_1.position.x,
      working_disturb_pose_1.position.y,
      working_disturb_pose_1.position.z,
      working_disturb_pose_1.orientation.x,
      working_disturb_pose_1.orientation.y,
      working_disturb_pose_1.orientation.z,
      working_disturb_pose_1.orientation.w);

    publish_feedback(
      goal_handle,
      0,
      execute_motion ? "MoveToPose to working_retract_pose" : "Plan MoveToPose to working_retract_pose (execution skipped)");
    if (!call_move_to_pose(working_retract_pose, fast_velocity_scale_, execute_motion, error_msg)) {
      abort_goal(goal_handle, result, "Initial MoveToPose to working_retract_pose failed: " + error_msg, completed_count);
      return;
    }

    for (int32_t i = 1; i <= goal->repeat_count; ++i) {
      if (check_cancel(goal_handle, result, completed_count)) {
        return;
      }

      publish_feedback(
        goal_handle,
        i,
        execute_motion ? "Cartesian to meas_pose" : "Plan Cartesian to meas_pose (execution skipped)");
      if (!call_move_to_pose_cartesian(meas_pose, goal->velocity_scale, execute_motion, error_msg)) {
        abort_goal(goal_handle, result, fail_message("Cartesian to meas_pose", i, error_msg), completed_count);
        return;
      }

      publish_feedback(goal_handle, i, execute_motion ? "Wait at meas_pose" : "Skip measurement settle wait (plan-only)");
      if (execute_motion) {
        if (!sleep_with_cancel(goal_handle, result, completed_count)) {
          return;
        }
      }

      publish_feedback(
        goal_handle,
        i,
        execute_motion ? "Cartesian back to working_retract_pose" : "Plan Cartesian back to working_retract_pose (execution skipped)");
      if (!call_move_to_pose_cartesian(working_retract_pose, fast_velocity_scale_, execute_motion, error_msg)) {
        abort_goal(goal_handle, result, fail_message("Cartesian back to working_retract_pose", i, error_msg), completed_count);
        return;
      }

      publish_feedback(
        goal_handle,
        i,
        execute_motion ? "MoveToPose to working_disturb_pose_1" : "Plan MoveToPose to working_disturb_pose_1 (execution skipped)");
      if (!call_move_to_pose(working_disturb_pose_1, fast_velocity_scale_, execute_motion, error_msg)) {
        abort_goal(goal_handle, result, fail_message("MoveToPose to working_disturb_pose_1", i, error_msg), completed_count);
        return;
      }

      publish_feedback(
        goal_handle,
        i,
        execute_motion ? "MoveToPose back to working_retract_pose" : "Plan MoveToPose back to working_retract_pose (execution skipped)");
      if (!call_move_to_pose(working_retract_pose, fast_velocity_scale_, execute_motion, error_msg)) {
        abort_goal(goal_handle, result, fail_message("MoveToPose back to working_retract_pose", i, error_msg), completed_count);
        return;
      }

      completed_count = i;
    }

    clear_active_goals();
    publish_feedback(
      goal_handle,
      goal->repeat_count,
      execute_motion ? "RepeatabilityTest completed" : "RepeatabilityTest planning completed (execution skipped)");

    result->success = true;
    result->message = execute_motion ?
      "RepeatabilityTest completed successfully" :
      "RepeatabilityTest planning success; execution skipped";
    result->completed_count = completed_count;
    goal_handle->succeed(result);

    RCLCPP_INFO(get_logger(), "RepeatabilityTest completed successfully");
  }

  static std::string fail_message(
    const std::string & step,
    int32_t loop_index,
    const std::string & detail)
  {
    return step + " failed at loop " + std::to_string(loop_index) + ": " + detail;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RepeatabilityTestActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
