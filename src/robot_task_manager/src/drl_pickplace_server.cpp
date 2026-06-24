#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "robot_task_manager/action/drl_pick_place.hpp"
#include "robot_task_manager/action/move_gripper.hpp"
#include "robot_task_manager/action/move_to_pose_cartesian.hpp"

using namespace std::chrono_literals;

namespace
{

double position_error(
  const geometry_msgs::msg::Pose & a,
  const geometry_msgs::msg::Pose & b)
{
  const double dx = a.position.x - b.position.x;
  const double dy = a.position.y - b.position.y;
  const double dz = a.position.z - b.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double orientation_error_rad(
  const geometry_msgs::msg::Quaternion & a,
  const geometry_msgs::msg::Quaternion & b)
{
  tf2::Quaternion qa(a.x, a.y, a.z, a.w);
  tf2::Quaternion qb(b.x, b.y, b.z, b.w);
  if (qa.length2() <= 1e-12 || qb.length2() <= 1e-12) {
    return M_PI;
  }
  qa.normalize();
  qb.normalize();
  const double dot = std::abs(qa.dot(qb));
  return 2.0 * std::acos(std::clamp(dot, -1.0, 1.0));
}

bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) &&
         std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) &&
         std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) &&
         std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

}  // namespace

class DrlPickPlaceActionServer : public rclcpp::Node
{
public:
  using DrlPickPlace = robot_task_manager::action::DrlPickPlace;
  using GoalHandle = rclcpp_action::ServerGoalHandle<DrlPickPlace>;
  using MoveGripper = robot_task_manager::action::MoveGripper;
  using MoveGripperGoalHandle = rclcpp_action::ClientGoalHandle<MoveGripper>;
  using MoveToPoseCartesian = robot_task_manager::action::MoveToPoseCartesian;
  using CartesianGoalHandle = rclcpp_action::ClientGoalHandle<MoveToPoseCartesian>;

  DrlPickPlaceActionServer()
  : Node("drl_pickplace_action_server"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    planning_frame_ = declare_parameter<std::string>("planning_frame", "base_link");
    ee_link_ = declare_parameter<std::string>("ee_link", "tcp_link");
    position_tolerance_m_ = declare_parameter<double>("position_tolerance_m", 0.01);
    orientation_tolerance_rad_ = declare_parameter<double>("orientation_tolerance_rad", 0.10);
    sub_action_timeout_sec_ = declare_parameter<double>("sub_action_timeout_sec", 60.0);
    drl_timeout_sec_ = declare_parameter<double>("drl_timeout_sec", 120.0);
    drl_trajectory_endpoint_tolerance_m_ =
      declare_parameter<double>("drl_trajectory_endpoint_tolerance_m", 0.015);
    drl_plan_attempts_ = declare_parameter<int>("drl_plan_attempts", 3);
    gripper_open_width_m_ = declare_parameter<double>("gripper_open_width_m", 0.05);
    gripper_default_close_width_m_ = declare_parameter<double>("gripper_default_close_width_m", 0.028);
    pick_approach_height_m_ = declare_parameter<double>("pick_approach_height_m", 0.05);
    cartesian_velocity_scale_ = declare_parameter<double>("cartesian_velocity_scale", 0.3);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 2.0);
    planner_node_name_ = declare_parameter<std::string>(
      "planner_node_name", "/drl_unified_planner_node");

    move_gripper_client_ =
      rclcpp_action::create_client<MoveGripper>(this, "move_gripper");
    cartesian_client_ =
      rclcpp_action::create_client<MoveToPoseCartesian>(this, "move_to_pose_cartesian");

    set_planner_params_client_ =
      create_client<rcl_interfaces::srv::SetParameters>(planner_node_name_ + "/set_parameters");
    drl_plan_client_ = create_client<std_srvs::srv::Trigger>("/drl/plan");
    drl_clear_client_ = create_client<std_srvs::srv::Trigger>("/drl/clear_trajectory");
    drl_execute_client_ = create_client<std_srvs::srv::Trigger>("/drl/execute_forward");
    drl_status_client_ = create_client<std_srvs::srv::Trigger>("/drl/get_execution_status");

    auto qos = rclcpp::QoS(1).reliable().transient_local();
    trajectory_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/drl/forward_trajectory_poses",
      qos,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        latest_trajectory_ = *msg;
        trajectory_seq_++;
      });

    action_server_ = rclcpp_action::create_server<DrlPickPlace>(
      this,
      "drl_pickplace",
      std::bind(&DrlPickPlaceActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DrlPickPlaceActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&DrlPickPlaceActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "DrlPickPlace action server ready: /drl_pickplace");
  }

private:
  std::string planning_frame_;
  std::string ee_link_;
  std::string planner_node_name_;
  double position_tolerance_m_{0.01};
  double orientation_tolerance_rad_{0.10};
  double sub_action_timeout_sec_{60.0};
  double drl_timeout_sec_{120.0};
  double drl_trajectory_endpoint_tolerance_m_{0.015};
  int drl_plan_attempts_{3};
  double gripper_open_width_m_{0.05};
  double gripper_default_close_width_m_{0.028};
  double pick_approach_height_m_{0.05};
  double cartesian_velocity_scale_{0.3};
  double tf_timeout_sec_{2.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp_action::Server<DrlPickPlace>::SharedPtr action_server_;
  rclcpp_action::Client<MoveGripper>::SharedPtr move_gripper_client_;
  rclcpp_action::Client<MoveToPoseCartesian>::SharedPtr cartesian_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_planner_params_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr drl_plan_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr drl_clear_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr drl_execute_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr drl_status_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr trajectory_sub_;

  std::mutex active_goal_mutex_;
  MoveGripperGoalHandle::SharedPtr active_gripper_goal_;
  CartesianGoalHandle::SharedPtr active_cartesian_goal_;

  std::mutex trajectory_mutex_;
  geometry_msgs::msg::PoseArray latest_trajectory_;
  uint64_t trajectory_seq_{0};

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const DrlPickPlace::Goal> goal)
  {
    if (!finite_pose(goal->target_pick.pose) || !finite_pose(goal->target_place.pose)) {
      RCLCPP_WARN(get_logger(), "Reject DrlPickPlace goal: non-finite pose");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_WARN(get_logger(), "DrlPickPlace cancel requested");
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    if (active_gripper_goal_) {
      move_gripper_client_->async_cancel_goal(active_gripper_goal_);
    }
    if (active_cartesian_goal_) {
      cartesian_client_->async_cancel_goal(active_cartesian_goal_);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread(&DrlPickPlaceActionServer::execute, this, goal_handle).detach();
  }

  geometry_msgs::msg::PoseStamped current_pose()
  {
    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = now();
    out.header.frame_id = planning_frame_;
    try {
      const auto tf = tf_buffer_.lookupTransform(
        planning_frame_,
        ee_link_,
        tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_sec_));
      out.pose.position.x = tf.transform.translation.x;
      out.pose.position.y = tf.transform.translation.y;
      out.pose.position.z = tf.transform.translation.z;
      out.pose.orientation = tf.transform.rotation;
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Current EE pose TF unavailable: %s", e.what());
      out.pose.orientation.w = 1.0;
    }
    return out;
  }

  void publish_feedback(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::string & stage,
    float progress)
  {
    auto feedback = std::make_shared<DrlPickPlace::Feedback>();
    feedback->current_stage = stage;
    feedback->progress = progress;
    feedback->current_pose = current_pose();
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(get_logger(), "[DrlPickPlace] %s | %.1f%%", stage.c_str(), progress);
  }

  bool check_cancel(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::shared_ptr<DrlPickPlace::Result> & result,
    const std::string & stage)
  {
    if (!goal_handle->is_canceling()) {
      return false;
    }
    result->success = false;
    result->message = "DrlPickPlace canceled";
    result->failed_stage = stage;
    clear_active_goals();
    goal_handle->canceled(result);
    return true;
  }

  void clear_active_goals()
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    active_gripper_goal_.reset();
    active_cartesian_goal_.reset();
  }

  void abort_goal(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::shared_ptr<DrlPickPlace::Result> & result,
    const std::string & stage,
    const std::string & message)
  {
    clear_active_goals();
    result->success = false;
    result->message = message;
    result->failed_stage = stage;
    RCLCPP_ERROR(get_logger(), "DrlPickPlace failed at %s: %s", stage.c_str(), message.c_str());
    goal_handle->abort(result);
  }

  geometry_msgs::msg::PoseStamped transform_to_planning_frame(
    const geometry_msgs::msg::PoseStamped & input)
  {
    geometry_msgs::msg::PoseStamped src = input;
    if (src.header.frame_id.empty()) {
      src.header.frame_id = planning_frame_;
    }
    if (src.header.frame_id == planning_frame_) {
      src.header.stamp = now();
      return src;
    }
    geometry_msgs::msg::PoseStamped out;
    tf_buffer_.transform(
      src,
      out,
      planning_frame_,
      tf2::durationFromSec(tf_timeout_sec_));
    return out;
  }

  bool wait_for_servers(std::string & error_msg)
  {
    auto action_timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(sub_action_timeout_sec_));
    if (!move_gripper_client_->wait_for_action_server(action_timeout)) {
      error_msg = "MoveGripper server not available: /move_gripper";
      return false;
    }
    if (!cartesian_client_->wait_for_action_server(action_timeout)) {
      error_msg = "MoveToPoseCartesian server not available: /move_to_pose_cartesian";
      return false;
    }
    if (!set_planner_params_client_->wait_for_service(std::chrono::seconds(5))) {
      error_msg = "Planner parameter service not available: " + planner_node_name_ + "/set_parameters";
      return false;
    }
    for (const auto & item : std::vector<std::pair<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr>>{
        {"/drl/plan", drl_plan_client_},
        {"/drl/clear_trajectory", drl_clear_client_},
        {"/drl/execute_forward", drl_execute_client_},
        {"/drl/get_execution_status", drl_status_client_},
      })
    {
      if (!item.second->wait_for_service(std::chrono::seconds(5))) {
        error_msg = "DRL service not available: " + item.first;
        return false;
      }
    }
    return true;
  }

  bool call_move_gripper(double width_m, std::string & error_msg)
  {
    MoveGripper::Goal goal;
    goal.position = width_m;
    auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(sub_action_timeout_sec_));
    auto goal_future = move_gripper_client_->async_send_goal(goal);
    if (goal_future.wait_for(timeout) != std::future_status::ready) {
      error_msg = "Timeout sending MoveGripper goal";
      return false;
    }
    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      error_msg = "MoveGripper goal rejected";
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(active_goal_mutex_);
      active_gripper_goal_ = goal_handle;
    }
    auto result_future = move_gripper_client_->async_get_result(goal_handle);
    if (result_future.wait_for(timeout) != std::future_status::ready) {
      error_msg = "Timeout waiting MoveGripper result";
      return false;
    }
    auto wrapped = result_future.get();
    {
      std::lock_guard<std::mutex> lock(active_goal_mutex_);
      active_gripper_goal_.reset();
    }
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result) {
      error_msg = "MoveGripper failed, result code=" +
        std::to_string(static_cast<int>(wrapped.code));
      return false;
    }
    if (!wrapped.result->success) {
      error_msg = wrapped.result->message;
      return false;
    }
    return true;
  }

  bool call_cartesian(
    const geometry_msgs::msg::Pose & target,
    std::string & error_msg)
  {
    MoveToPoseCartesian::Goal goal;
    goal.target_pose = target;
    goal.velocity_scale = cartesian_velocity_scale_;
    auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(sub_action_timeout_sec_));
    auto goal_future = cartesian_client_->async_send_goal(goal);
    if (goal_future.wait_for(timeout) != std::future_status::ready) {
      error_msg = "Timeout sending MoveToPoseCartesian goal";
      return false;
    }
    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      error_msg = "MoveToPoseCartesian goal rejected";
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(active_goal_mutex_);
      active_cartesian_goal_ = goal_handle;
    }
    auto result_future = cartesian_client_->async_get_result(goal_handle);
    if (result_future.wait_for(timeout) != std::future_status::ready) {
      error_msg = "Timeout waiting MoveToPoseCartesian result";
      return false;
    }
    auto wrapped = result_future.get();
    {
      std::lock_guard<std::mutex> lock(active_goal_mutex_);
      active_cartesian_goal_.reset();
    }
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result) {
      error_msg = "MoveToPoseCartesian failed, result code=" +
        std::to_string(static_cast<int>(wrapped.code));
      return false;
    }
    if (!wrapped.result->success) {
      error_msg = wrapped.result->message;
      return false;
    }
    return true;
  }

  rcl_interfaces::msg::Parameter make_double_array_param(
    const std::string & name,
    const std::vector<double> & values)
  {
    rcl_interfaces::msg::Parameter param;
    param.name = name;
    param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
    param.value.double_array_value = values;
    return param;
  }

  rcl_interfaces::msg::Parameter make_bool_param(
    const std::string & name,
    bool value)
  {
    rcl_interfaces::msg::Parameter param;
    param.name = name;
    param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    param.value.bool_value = value;
    return param;
  }

  bool set_drl_target(
    const geometry_msgs::msg::Pose & target,
    bool preposition_before_plan,
    std::string & error_msg)
  {
    auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    req->parameters.push_back(make_double_array_param(
      "manual_default_target",
      {target.position.x, target.position.y, target.position.z}));
    req->parameters.push_back(make_bool_param("preposition_before_plan", preposition_before_plan));
    req->parameters.push_back(make_bool_param("update_start_tcp_from_tf_before_plan", true));
    req->parameters.push_back(make_bool_param("auto_execute_after_plan", false));

    auto future = set_planner_params_client_->async_send_request(req);
    if (future.wait_for(5s) != std::future_status::ready) {
      error_msg = "Timeout setting DRL planner target parameters";
      return false;
    }
    auto resp = future.get();
    if (!resp) {
      error_msg = "DRL planner parameter service returned no response";
      return false;
    }
    for (size_t i = 0; i < resp->results.size(); ++i) {
      if (!resp->results[i].successful) {
        error_msg = "Failed setting DRL planner parameter " +
          req->parameters[i].name + ": " + resp->results[i].reason;
        return false;
      }
    }
    return true;
  }

  bool call_trigger(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
    const std::string & name,
    std::string & response_message,
    double timeout_sec)
  {
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(req);
    auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(timeout_sec));
    if (future.wait_for(timeout) != std::future_status::ready) {
      response_message = "Timeout calling " + name;
      return false;
    }
    auto resp = future.get();
    if (!resp) {
      response_message = name + " returned no response";
      return false;
    }
    response_message = resp->message;
    return resp->success;
  }

  bool wait_for_planned_trajectory(
    uint64_t seq_before,
    const geometry_msgs::msg::Pose & target,
    std::string & error_msg)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(drl_timeout_sec_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        if (trajectory_seq_ > seq_before && !latest_trajectory_.poses.empty()) {
          const auto & final_pose = latest_trajectory_.poses.back();
          const double err = position_error(final_pose, target);
          const double allowed = std::max(
            position_tolerance_m_,
            drl_trajectory_endpoint_tolerance_m_);
          if (err <= allowed) {
            RCLCPP_INFO(
              get_logger(),
              "DRL trajectory ready: waypoints=%zu final_position_error=%.5f allowed=%.5f",
              latest_trajectory_.poses.size(),
              err,
              allowed);
            return true;
          }
          error_msg = "DRL trajectory final waypoint error too large: " +
            std::to_string(err);
          return false;
        }
      }
      std::this_thread::sleep_for(100ms);
    }
    error_msg = "Timed out waiting for DRL trajectory";
    return false;
  }

  bool wait_for_drl_execution(std::string & error_msg)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(drl_timeout_sec_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      std::string status_msg;
      const bool idle = call_trigger(
        drl_status_client_,
        "/drl/get_execution_status",
        status_msg,
        2.0);
      if (idle) {
        if (status_msg.find("SUCCEEDED") != std::string::npos) {
          RCLCPP_INFO(get_logger(), "DRL execution completed: %s", status_msg.c_str());
          return true;
        }
        if (status_msg.find("FAILED") != std::string::npos) {
          error_msg = "DRL execution failed: " + status_msg;
          return false;
        }
      }
      std::this_thread::sleep_for(200ms);
    }
    error_msg = "Timed out waiting for DRL execution";
    return false;
  }

  bool call_drl_plan_and_execute(
    const geometry_msgs::msg::Pose & target,
    bool preposition_before_plan,
    std::string & error_msg)
  {
    const int attempts = std::max(1, drl_plan_attempts_);
    std::string last_error;
    for (int attempt = 1; attempt <= attempts; ++attempt) {
      RCLCPP_INFO(
        get_logger(),
        "DRL plan attempt %d/%d target=(%.4f %.4f %.4f) preposition=%s",
        attempt,
        attempts,
        target.position.x,
        target.position.y,
        target.position.z,
        preposition_before_plan ? "true" : "false");

      if (!set_drl_target(target, preposition_before_plan, last_error)) {
        error_msg = last_error;
        return false;
      }

      uint64_t seq_before = 0;
      {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        seq_before = trajectory_seq_;
      }

      std::string msg;
      if (!call_trigger(drl_clear_client_, "/drl/clear_trajectory", msg, 5.0)) {
        last_error = "Clear DRL trajectory failed: " + msg;
      } else if (!call_trigger(drl_plan_client_, "/drl/plan", msg, 5.0)) {
        last_error = "Start DRL planning failed: " + msg;
      } else if (!wait_for_planned_trajectory(seq_before, target, last_error)) {
        // last_error is already populated.
      } else if (!call_trigger(drl_execute_client_, "/drl/execute_forward", msg, 5.0)) {
        last_error = "Start DRL execution failed: " + msg;
      } else if (!wait_for_drl_execution(last_error)) {
        // last_error is already populated.
      } else {
        return true;
      }

      RCLCPP_WARN(
        get_logger(),
        "DRL plan attempt %d/%d failed: %s",
        attempt,
        attempts,
        last_error.c_str());
      if (attempt < attempts) {
        std::this_thread::sleep_for(500ms);
      }
    }
    error_msg = last_error.empty() ? "DRL planning failed" : last_error;
    return false;
  }

  bool verify_pose(
    const std::string & label,
    const geometry_msgs::msg::Pose & target,
    std::string & error_msg)
  {
    const auto actual = current_pose();
    const double pos_err = position_error(actual.pose, target);
    const double ori_err = orientation_error_rad(actual.pose.orientation, target.orientation);
    RCLCPP_INFO(
      get_logger(),
      "%s pose check | requested=(%.4f %.4f %.4f) actual=(%.4f %.4f %.4f) pos_err=%.5f ori_err=%.5f",
      label.c_str(),
      target.position.x,
      target.position.y,
      target.position.z,
      actual.pose.position.x,
      actual.pose.position.y,
      actual.pose.position.z,
      pos_err,
      ori_err);
    if (pos_err > position_tolerance_m_) {
      error_msg = label + " position tolerance failed: " + std::to_string(pos_err);
      return false;
    }
    if (ori_err > orientation_tolerance_rad_) {
      error_msg = label + " orientation tolerance failed: " + std::to_string(ori_err);
      return false;
    }
    return true;
  }

  double sanitize_close_width(double requested) const
  {
    if (!std::isfinite(requested) || requested <= 0.0) {
      return gripper_default_close_width_m_;
    }
    return std::clamp(requested, 0.0, gripper_open_width_m_);
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto result = std::make_shared<DrlPickPlace::Result>();
    const auto goal = goal_handle->get_goal();
    std::string error_msg;

    publish_feedback(goal_handle, "VALIDATE_GOAL", 1.0f);
    geometry_msgs::msg::PoseStamped target_pick;
    geometry_msgs::msg::PoseStamped target_place;
    try {
      target_pick = transform_to_planning_frame(goal->target_pick);
      target_place = transform_to_planning_frame(goal->target_place);
    } catch (const std::exception & e) {
      abort_goal(goal_handle, result, "VALIDATE_GOAL", std::string("Goal transform failed: ") + e.what());
      return;
    }
    const double close_width_m = sanitize_close_width(goal->gripper_close_width_m);

    publish_feedback(goal_handle, "WAIT_FOR_SERVERS", 3.0f);
    if (!wait_for_servers(error_msg)) {
      abort_goal(goal_handle, result, "WAIT_FOR_SERVERS", error_msg);
      return;
    }
    if (check_cancel(goal_handle, result, "WAIT_FOR_SERVERS")) {
      return;
    }

    auto pre_pick = target_pick.pose;
    pre_pick.position.z += pick_approach_height_m_;
    auto lift_pose = pre_pick;

    publish_feedback(goal_handle, "OPEN_GRIPPER", 8.0f);
    if (!call_move_gripper(gripper_open_width_m_, error_msg)) {
      abort_goal(goal_handle, result, "OPEN_GRIPPER", error_msg);
      return;
    }
    if (check_cancel(goal_handle, result, "OPEN_GRIPPER")) {
      return;
    }

    publish_feedback(goal_handle, "PLAN_TO_PRE_PICK", 22.0f);
    if (!call_drl_plan_and_execute(pre_pick, true, error_msg) ||
        !verify_pose("PLAN_TO_PRE_PICK", pre_pick, error_msg))
    {
      abort_goal(goal_handle, result, "PLAN_TO_PRE_PICK", error_msg);
      return;
    }
    if (check_cancel(goal_handle, result, "PLAN_TO_PRE_PICK")) {
      return;
    }

    publish_feedback(goal_handle, "DESCEND_TO_PICK", 38.0f);
    if (!call_cartesian(target_pick.pose, error_msg) ||
        !verify_pose("DESCEND_TO_PICK", target_pick.pose, error_msg))
    {
      abort_goal(goal_handle, result, "DESCEND_TO_PICK", error_msg);
      return;
    }
    if (check_cancel(goal_handle, result, "DESCEND_TO_PICK")) {
      return;
    }

    publish_feedback(goal_handle, "CLOSE_GRIPPER", 50.0f);
    if (!call_move_gripper(close_width_m, error_msg)) {
      abort_goal(goal_handle, result, "CLOSE_GRIPPER", error_msg);
      return;
    }
    if (check_cancel(goal_handle, result, "CLOSE_GRIPPER")) {
      return;
    }

    publish_feedback(goal_handle, "LIFT_FROM_PICK", 62.0f);
    if (!call_cartesian(lift_pose, error_msg) ||
        !verify_pose("LIFT_FROM_PICK", lift_pose, error_msg))
    {
      abort_goal(goal_handle, result, "LIFT_FROM_PICK", error_msg);
      return;
    }
    if (check_cancel(goal_handle, result, "LIFT_FROM_PICK")) {
      return;
    }

    publish_feedback(goal_handle, "PLAN_TO_PLACE", 82.0f);
    if (!call_drl_plan_and_execute(target_place.pose, false, error_msg) ||
        !verify_pose("PLAN_TO_PLACE", target_place.pose, error_msg))
    {
      abort_goal(goal_handle, result, "PLAN_TO_PLACE", error_msg);
      return;
    }
    if (check_cancel(goal_handle, result, "PLAN_TO_PLACE")) {
      return;
    }

    publish_feedback(goal_handle, "OPEN_GRIPPER_AT_PLACE", 96.0f);
    if (!call_move_gripper(gripper_open_width_m_, error_msg)) {
      abort_goal(goal_handle, result, "OPEN_GRIPPER_AT_PLACE", error_msg);
      return;
    }
    if (check_cancel(goal_handle, result, "OPEN_GRIPPER_AT_PLACE")) {
      return;
    }

    publish_feedback(goal_handle, "DONE", 100.0f);
    result->success = true;
    result->message = "DrlPickPlace completed successfully";
    result->failed_stage = "";
    clear_active_goals();
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "DrlPickPlace completed successfully");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DrlPickPlaceActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
