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
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "robot_task_manager/action/move_pose_rl.hpp"

using namespace std::chrono_literals;

namespace
{

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

bool valid_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(norm2) && norm2 > 1e-12;
}

double position_error(
  const geometry_msgs::msg::Pose & a,
  const geometry_msgs::msg::Pose & b)
{
  const double dx = a.position.x - b.position.x;
  const double dy = a.position.y - b.position.y;
  const double dz = a.position.z - b.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

class MovePoseRlActionServer : public rclcpp::Node
{
public:
  using MovePoseRl = robot_task_manager::action::MovePoseRl;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MovePoseRl>;

  MovePoseRlActionServer()
  : Node("move_pose_rl_action_server"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    planning_frame_ = declare_parameter<std::string>("planning_frame", "base_link");
    ee_link_ = declare_parameter<std::string>("ee_link", "tcp_link");
    position_tolerance_m_ = declare_parameter<double>("position_tolerance_m", 0.01);
    orientation_tolerance_rad_ = declare_parameter<double>("orientation_tolerance_rad", 0.10);
    drl_timeout_sec_ = declare_parameter<double>("drl_timeout_sec", 120.0);
    drl_trajectory_endpoint_tolerance_m_ =
      declare_parameter<double>("drl_trajectory_endpoint_tolerance_m", 0.015);
    drl_plan_attempts_ = declare_parameter<int>("drl_plan_attempts", 3);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 2.0);
    sub_action_timeout_sec_ = declare_parameter<double>("sub_action_timeout_sec", 60.0);
    planner_node_name_ = declare_parameter<std::string>(
      "planner_node_name", "/drl_unified_planner_node");

    set_planner_params_client_ =
      create_client<rcl_interfaces::srv::SetParameters>(planner_node_name_ + "/set_parameters");
    drl_plan_client_ = create_client<std_srvs::srv::Trigger>("/drl/plan");
    drl_clear_client_ = create_client<std_srvs::srv::Trigger>("/drl/clear_trajectory");
    drl_execute_client_ = create_client<std_srvs::srv::Trigger>("/drl/execute_forward");
    drl_status_client_ = create_client<std_srvs::srv::Trigger>("/drl/get_execution_status");

    auto trajectory_qos = rclcpp::QoS(1).reliable().transient_local();
    trajectory_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/drl/forward_trajectory_poses",
      trajectory_qos,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        latest_trajectory_ = *msg;
        trajectory_seq_++;
      });

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!msg->name.empty() && !msg->position.empty()) {
          std::lock_guard<std::mutex> lock(joint_state_mutex_);
          received_joint_state_ = true;
        }
      });

    action_server_ = rclcpp_action::create_server<MovePoseRl>(
      this,
      "move_pose_rl",
      std::bind(&MovePoseRlActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MovePoseRlActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MovePoseRlActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "MovePoseRl action server ready: /move_pose_rl");
  }

private:
  std::string planning_frame_;
  std::string ee_link_;
  std::string planner_node_name_;
  double position_tolerance_m_{0.01};
  double orientation_tolerance_rad_{0.10};
  double drl_timeout_sec_{120.0};
  double drl_trajectory_endpoint_tolerance_m_{0.015};
  int drl_plan_attempts_{3};
  double tf_timeout_sec_{2.0};
  double sub_action_timeout_sec_{60.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp_action::Server<MovePoseRl>::SharedPtr action_server_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_planner_params_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr drl_plan_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr drl_clear_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr drl_execute_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr drl_status_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  std::mutex trajectory_mutex_;
  geometry_msgs::msg::PoseArray latest_trajectory_;
  uint64_t trajectory_seq_{0};

  std::mutex joint_state_mutex_;
  bool received_joint_state_{false};

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MovePoseRl::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_WARN(get_logger(), "MovePoseRl cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread(&MovePoseRlActionServer::execute, this, goal_handle).detach();
  }

  bool current_pose(geometry_msgs::msg::PoseStamped & out, std::string & error_msg)
  {
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
      return true;
    } catch (const std::exception & e) {
      error_msg = "Failed to get current TCP pose from TF " + planning_frame_ +
        " <- " + ee_link_ + ": " + e.what();
      return false;
    }
  }

  bool wait_for_joint_state(std::string & error_msg)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(tf_timeout_sec_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        if (received_joint_state_) {
          return true;
        }
      }
      std::this_thread::sleep_for(50ms);
    }
    error_msg =
      "Failed to receive /joint_states before DRL planning. "
      "Refusing to plan from zero/default state.";
    return false;
  }

  void publish_feedback(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::string & stage,
    float progress)
  {
    auto feedback = std::make_shared<MovePoseRl::Feedback>();
    feedback->current_stage = stage;
    feedback->progress = progress;
    std::string pose_error;
    current_pose(feedback->current_pose, pose_error);
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(get_logger(), "[MovePoseRl] %s | %.1f%%", stage.c_str(), progress);
  }

  void abort_goal(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::shared_ptr<MovePoseRl::Result> & result,
    const std::string & stage,
    const std::string & message)
  {
    result->success = false;
    result->message = message;
    result->failed_stage = stage;
    RCLCPP_ERROR(get_logger(), "MovePoseRl failed at %s: %s", stage.c_str(), message.c_str());
    goal_handle->abort(result);
  }

  bool validate_goal(const MovePoseRl::Goal & goal, std::string & error_msg)
  {
    if (!std::isfinite(goal.velocity_scale) ||
      goal.velocity_scale <= 0.0 ||
      goal.velocity_scale > 1.0)
    {
      error_msg = "velocity_scale must be finite and in (0, 1]";
      return false;
    }
    if (!finite_pose(goal.target_pose)) {
      error_msg = "target_pose contains non-finite values";
      return false;
    }
    if (!valid_quaternion(goal.target_pose.orientation)) {
      error_msg = "target_pose orientation quaternion is invalid";
      return false;
    }
    return true;
  }

  bool wait_for_services(bool execute_motion, std::string & error_msg)
  {
    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(sub_action_timeout_sec_));

    const auto wait_for_set_parameters =
      [this, timeout, &error_msg](const std::string & service_name) {
        RCLCPP_INFO(
          get_logger(),
          "[MovePoseRl] checking service: %s ...",
          service_name.c_str());
        if (!set_planner_params_client_->wait_for_service(timeout)) {
          error_msg = "missing " + service_name;
          RCLCPP_ERROR(
            get_logger(),
            "[MovePoseRl] checking service: %s ... MISSING",
            service_name.c_str());
          return false;
        }
        RCLCPP_INFO(
          get_logger(),
          "[MovePoseRl] checking service: %s ... OK",
          service_name.c_str());
        return true;
      };

    const auto wait_for_trigger =
      [this, timeout, &error_msg](
        const std::string & service_name,
        const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client) {
        RCLCPP_INFO(
          get_logger(),
          "[MovePoseRl] checking service: %s ...",
          service_name.c_str());
        if (!client->wait_for_service(timeout)) {
          error_msg = "missing " + service_name;
          RCLCPP_ERROR(
            get_logger(),
            "[MovePoseRl] checking service: %s ... MISSING",
            service_name.c_str());
          return false;
        }
        RCLCPP_INFO(
          get_logger(),
          "[MovePoseRl] checking service: %s ... OK",
          service_name.c_str());
        return true;
      };

    const std::string parameter_service = planner_node_name_ + "/set_parameters";
    if (!wait_for_set_parameters(parameter_service)) {
      return false;
    }

    const std::vector<std::pair<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr>>
      required_services = execute_motion ?
      std::vector<std::pair<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr>>{
        {"/drl/plan", drl_plan_client_},
        {"/drl/clear_trajectory", drl_clear_client_},
        {"/drl/execute_forward", drl_execute_client_},
        {"/drl/get_execution_status", drl_status_client_},
      } :
      std::vector<std::pair<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr>>{
        {"/drl/plan", drl_plan_client_},
        {"/drl/clear_trajectory", drl_clear_client_},
      };

    for (const auto & item : required_services) {
      if (!wait_for_trigger(item.first, item.second)) {
        return false;
      }
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
    std::string & error_msg)
  {
    auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    req->parameters.push_back(make_double_array_param(
      "manual_default_target",
      {target.position.x, target.position.y, target.position.z}));
    req->parameters.push_back(make_bool_param("preposition_before_plan", false));
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
    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
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
          const double pos_err = position_error(final_pose, target);
          const double allowed_position = std::max(
            position_tolerance_m_,
            drl_trajectory_endpoint_tolerance_m_);
          if (pos_err > allowed_position) {
            error_msg = "DRL trajectory final waypoint position error too large: " +
              std::to_string(pos_err);
            return false;
          }
          RCLCPP_INFO(
            get_logger(),
            "DRL planner target is position-only; endpoint orientation is not enforced");
          RCLCPP_INFO(
            get_logger(),
            "DRL trajectory ready: waypoints=%zu final_position_error=%.5f",
            latest_trajectory_.poses.size(),
            pos_err);
          return true;
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
      const bool ok = call_trigger(
        drl_status_client_,
        "/drl/get_execution_status",
        status_msg,
        2.0);
      if (ok) {
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

  bool plan_with_drl(
    const geometry_msgs::msg::Pose & target,
    std::string & failed_stage,
    std::string & error_msg)
  {
    const int attempts = std::max(1, drl_plan_attempts_);
    std::string last_error;
    for (int attempt = 1; attempt <= attempts; ++attempt) {
      RCLCPP_INFO(
        get_logger(),
        "MovePoseRl DRL plan attempt %d/%d target=(%.4f %.4f %.4f)",
        attempt,
        attempts,
        target.position.x,
        target.position.y,
        target.position.z);

      if (!set_drl_target(target, last_error)) {
        failed_stage = "drl_plan";
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
        failed_stage = "drl_plan";
      } else if (!call_trigger(drl_plan_client_, "/drl/plan", msg, 5.0)) {
        last_error = "Start DRL planning failed: " + msg;
        failed_stage = "drl_plan";
      } else if (!wait_for_planned_trajectory(seq_before, target, last_error)) {
        failed_stage = "endpoint_check";
      } else {
        return true;
      }

      RCLCPP_WARN(
        get_logger(),
        "MovePoseRl DRL plan attempt %d/%d failed: %s",
        attempt,
        attempts,
        last_error.c_str());
      if (attempt < attempts) {
        std::this_thread::sleep_for(500ms);
      }
    }
    error_msg = last_error.empty() ? "DRL planning failed" : last_error;
    if (failed_stage.empty()) {
      failed_stage = "drl_plan";
    }
    return false;
  }

  bool verify_final_pose(
    const geometry_msgs::msg::Pose & target,
    std::string & error_msg)
  {
    geometry_msgs::msg::PoseStamped actual;
    if (!current_pose(actual, error_msg)) {
      return false;
    }
    const double pos_err = position_error(actual.pose, target);
    if (pos_err > position_tolerance_m_) {
      error_msg = "Final TCP position tolerance failed: " + std::to_string(pos_err);
      return false;
    }
    RCLCPP_INFO(
      get_logger(),
      "DRL execution final pose check is position-only because planner target has no orientation input");
    return true;
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto result = std::make_shared<MovePoseRl::Result>();
    const auto goal = goal_handle->get_goal();
    const bool execute_motion = goal->execute;
    std::string error_msg;

    publish_feedback(goal_handle, "validate_goal", 5.0f);
    if (!validate_goal(*goal, error_msg)) {
      abort_goal(goal_handle, result, "validate_goal", error_msg);
      return;
    }

    publish_feedback(goal_handle, "get_current_pose", 15.0f);
    if (!wait_for_joint_state(error_msg)) {
      abort_goal(goal_handle, result, "get_current_pose", error_msg);
      return;
    }
    geometry_msgs::msg::PoseStamped start_pose;
    if (!current_pose(start_pose, error_msg)) {
      abort_goal(goal_handle, result, "get_current_pose", error_msg);
      return;
    }

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.stamp = now();
    target_pose.header.frame_id = planning_frame_;
    target_pose.pose = goal->target_pose;

    publish_feedback(goal_handle, "wait_for_drl_services", 25.0f);
    if (!wait_for_services(execute_motion, error_msg)) {
      abort_goal(goal_handle, result, "drl_plan", error_msg);
      return;
    }

    publish_feedback(goal_handle, "drl_plan", 45.0f);
    std::string failed_stage;
    if (!plan_with_drl(target_pose.pose, failed_stage, error_msg)) {
      abort_goal(goal_handle, result, failed_stage, error_msg);
      return;
    }

    if (!execute_motion) {
      publish_feedback(goal_handle, "done_plan_only", 100.0f);
      result->success = true;
      result->message = "DRL plan succeeded; execution skipped because execute=false";
      result->failed_stage = "";
      goal_handle->succeed(result);
      return;
    }

    publish_feedback(goal_handle, "execute_forward", 70.0f);
    std::string msg;
    if (!call_trigger(drl_execute_client_, "/drl/execute_forward", msg, 5.0)) {
      abort_goal(goal_handle, result, "execute_forward", "Start DRL execution failed: " + msg);
      return;
    }

    publish_feedback(goal_handle, "execution_status", 85.0f);
    if (!wait_for_drl_execution(error_msg)) {
      abort_goal(goal_handle, result, "execution_status", error_msg);
      return;
    }
    if (!verify_final_pose(target_pose.pose, error_msg)) {
      abort_goal(goal_handle, result, "execution_status", error_msg);
      return;
    }

    publish_feedback(goal_handle, "done", 100.0f);
    result->success = true;
    result->message = "MovePoseRl completed successfully";
    result->failed_stage = "";
    goal_handle->succeed(result);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MovePoseRlActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
