#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "robot_task_manager/action/move_pose_rl.hpp"
#include "robot_task_manager/action_metrics_logger.hpp"
#include "robot_task_manager/log_paths.hpp"
#include "robot_task_manager/rl_obstacle_input.hpp"
#include "robot_task_executor/executor_experiment_logger.hpp"
#include "robot_vision_pipeline_msgs/msg/box_array.hpp"

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

std::array<double, 3> rpy_deg_from_quat(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  if (quat.length2() <= 1e-12) {
    return {0.0, 0.0, 0.0};
  }
  quat.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  constexpr double kRadToDeg = 180.0 / M_PI;
  return {roll * kRadToDeg, pitch * kRadToDeg, yaw * kRadToDeg};
}

std::string format_values(const std::vector<double> & values)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6) << "[";
  for (size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      oss << ", ";
    }
    oss << values[i];
  }
  oss << "]";
  return oss.str();
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

bool finite_vector3(const geometry_msgs::msg::Vector3 & v)
{
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// "Full size" obstacle: every axis must be a real, strictly positive extent
// (same rule as MoveTargetRl/MoveToPoseObstacle — codex2.md
// "move_pose_rl_obstacle_avoidance").
bool positive_size(const geometry_msgs::msg::Vector3 & v)
{
  return finite_vector3(v) && v.x > 1e-6 && v.y > 1e-6 && v.z > 1e-6;
}

// Shortest distance from `point` to the segment [seg_a, seg_b] — used to
// pick the box detection that best represents the obstacle actually
// standing between the current TCP and the target (same rule as
// MoveTargetRl/MoveToPoseObstacle's obstacle selection).
double point_to_segment_distance(
  const geometry_msgs::msg::Point & point,
  const geometry_msgs::msg::Point & seg_a,
  const geometry_msgs::msg::Point & seg_b)
{
  const double abx = seg_b.x - seg_a.x;
  const double aby = seg_b.y - seg_a.y;
  const double abz = seg_b.z - seg_a.z;
  const double apx = point.x - seg_a.x;
  const double apy = point.y - seg_a.y;
  const double apz = point.z - seg_a.z;
  const double ab_len2 = abx * abx + aby * aby + abz * abz;
  double t = 0.0;
  if (ab_len2 > 1e-12) {
    t = (apx * abx + apy * aby + apz * abz) / ab_len2;
    t = std::clamp(t, 0.0, 1.0);
  }
  const double cx = seg_a.x + t * abx;
  const double cy = seg_a.y + t * aby;
  const double cz = seg_a.z + t * abz;
  const double dx = point.x - cx;
  const double dy = point.y - cy;
  const double dz = point.z - cz;
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

    // codex2.md "move_pose_rl_obstacle_avoidance": MovePoseRl.action has no
    // obstacle-related goal fields (unlike MoveTargetRl/MoveToPoseObstacle),
    // so this is always best-effort — use a fresh /vision/box_objects
    // detection when available, plan without an obstacle when not. No
    // "require_obstacle" here since the interface has nothing to fail on.
    obstacle_class_ = declare_parameter<std::string>("obstacle_class", "box");
    vision_timeout_sec_ = declare_parameter<double>("vision_timeout_sec", 1.0);
    box_objects_topic_ = declare_parameter<std::string>(
      "box_objects_topic", "/vision/box_objects");
    rl_obstacle_safety_margin_m_ =
      declare_parameter<double>("rl_obstacle_safety_margin_m", 0.03);

    enable_executor_logging_ = declare_parameter<bool>("enable_executor_logging", false);
    log_root_dir_            = declare_parameter<std::string>("log_root_dir", robot_task_manager::kDefaultLogRootDir);
    runtime_mode_            = declare_parameter<std::string>("runtime_mode", "mock");
    executor_log_dir_        = declare_parameter<std::string>(
      "executor_log_dir", robot_task_manager::executorLogBaseDir(log_root_dir_));
    executor_sample_rate_hz_ = declare_parameter<double>("executor_sample_rate_hz", 50.0);
    executor_base_frame_     = declare_parameter<std::string>("executor_base_frame", "base_link");
    executor_tcp_frame_      = declare_parameter<std::string>("executor_tcp_frame", "tcp_link");

    set_planner_params_client_ =
      create_client<rcl_interfaces::srv::SetParameters>(planner_node_name_ + "/set_parameters");
    drl_plan_client_ = create_client<std_srvs::srv::Trigger>("/drl/plan");
    drl_clear_client_ = create_client<std_srvs::srv::Trigger>("/drl/clear_trajectory");
    drl_execute_client_ = create_client<std_srvs::srv::Trigger>("/drl/execute_forward");
    drl_status_client_ = create_client<std_srvs::srv::Trigger>("/drl/get_execution_status");
    drl_planning_status_client_ = create_client<std_srvs::srv::Trigger>("/drl/get_planning_status");
    // Stop path (codex.md section 7.6): RL robot motion actually runs in the
    // cartesian executor (task_executor_node / robot_drl_executor_node) via
    // /move_cartesian_pose_sequence, so a real Stop must halt it there — not
    // just cancel this wrapper action.
    cartesian_stop_client_ = create_client<std_srvs::srv::Trigger>("/move_cartesian_stop");

    auto trajectory_qos = rclcpp::QoS(1).reliable().transient_local();
    trajectory_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/drl/forward_trajectory_poses",
      trajectory_qos,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        latest_trajectory_ = *msg;
        trajectory_seq_++;
      });
    observation_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/drl/last_plan_observation_15d",
      trajectory_qos,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() < 30) {
          RCLCPP_WARN(
            get_logger(),
            "Ignoring /drl/last_plan_observation_15d with %zu values; expected 30",
            msg->data.size());
          return;
        }
        std::lock_guard<std::mutex> lock(observation_mutex_);
        latest_raw_observation_.assign(msg->data.begin(), msg->data.begin() + 15);
        latest_model_observation_.assign(msg->data.begin() + 15, msg->data.begin() + 30);
        observation_seq_++;
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

    // Vision detections are published BEST_EFFORT/VOLATILE by
    // pixel_to_base_mapper_node (see robot_vision_pipeline config
    // pixel_to_base_mapper.yaml) — match QoS so the subscription is
    // compatible, same as MoveTargetRl/MoveToPoseObstacle.
    auto vision_qos = rclcpp::QoS(1).best_effort().durability_volatile();
    box_sub_ = create_subscription<robot_vision_pipeline_msgs::msg::BoxArray>(
      box_objects_topic_,
      vision_qos,
      [this](const robot_vision_pipeline_msgs::msg::BoxArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(vision_mutex_);
        latest_box_ = *msg;
        latest_box_stamp_ = now();
        have_box_ = true;
      });

    metrics_logger_ = std::make_shared<robot_task_manager::ActionMetricsLogger>(
      robot_task_manager::actionMetricsLogDir(log_root_dir_, runtime_mode_, "MovePoseRl"), get_logger());
    declare_parameter<bool>("use_mock", true);
    declare_parameter<std::string>("hardware_plugin", "unknown");
    declare_parameter<bool>("enable_log_plots", true);
    robot_task_manager::applyLogProvenanceFromParams(this, metrics_logger_);

    action_server_ = rclcpp_action::create_server<MovePoseRl>(
      this,
      "move_pose_rl",
      std::bind(&MovePoseRlActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MovePoseRlActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MovePoseRlActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "MovePoseRl action server ready: /move_pose_rl");
  }

  // Called once after make_shared(), before spin() — mirrors
  // MoveItExecutor::initializeLogging()'s pattern, needed because
  // shared_from_this() cannot be used inside the constructor.
  void initialize_logging()
  {
    if (!enable_executor_logging_) {
      return;
    }
    try {
      log_tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      log_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*log_tf_buffer_);
      logger_ = std::make_shared<robot_task_executor::ExecutorExperimentLogger>(
        shared_from_this(), log_tf_buffer_,
        robot_task_manager::executorActionLogDir(log_root_dir_, executor_log_dir_, runtime_mode_, "MovePoseRl"),
        executor_sample_rate_hz_,
        executor_base_frame_, executor_tcp_frame_);
    } catch (const std::exception & e) {
      logger_.reset();
      RCLCPP_WARN(get_logger(), "MovePoseRl CSV logger unavailable: %s", e.what());
    }
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
  std::string obstacle_class_;
  double vision_timeout_sec_{1.0};
  std::string box_objects_topic_;
  double rl_obstacle_safety_margin_m_{0.03};

  bool enable_executor_logging_{false};
  std::string log_root_dir_;
  std::string runtime_mode_;
  std::string executor_log_dir_;
  double executor_sample_rate_hz_{50.0};
  std::string executor_base_frame_;
  std::string executor_tcp_frame_;
  std::shared_ptr<tf2_ros::Buffer> log_tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> log_tf_listener_;
  std::shared_ptr<robot_task_executor::ExecutorExperimentLogger> logger_;
  uint64_t action_call_id_{0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp_action::Server<MovePoseRl>::SharedPtr action_server_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_planner_params_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr drl_plan_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr drl_clear_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr drl_execute_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr drl_status_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr drl_planning_status_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cartesian_stop_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr observation_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<robot_vision_pipeline_msgs::msg::BoxArray>::SharedPtr box_sub_;

  std::atomic<bool> cancel_requested_{false};

  std::mutex trajectory_mutex_;
  geometry_msgs::msg::PoseArray latest_trajectory_;
  uint64_t trajectory_seq_{0};

  std::mutex observation_mutex_;
  std::vector<double> latest_raw_observation_;
  std::vector<double> latest_model_observation_;
  uint64_t observation_seq_{0};

  std::mutex joint_state_mutex_;
  bool received_joint_state_{false};

  std::mutex vision_mutex_;
  robot_vision_pipeline_msgs::msg::BoxArray latest_box_;
  rclcpp::Time latest_box_stamp_;
  bool have_box_{false};

  std::mutex goal_active_mutex_;
  bool goal_active_{false};

  // Per-call metrics state — safe as plain members for the same reason
  // action_call_id_ above already is: goal_active_ guarantees only one
  // execute() runs at a time per node instance.
  std::shared_ptr<robot_task_manager::ActionMetricsLogger> metrics_logger_;
  std::shared_ptr<robot_task_manager::ActionMetricsRow> metrics_row_;
  rclcpp::Time goal_start_time_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MovePoseRl::Goal>)
  {
    if (logger_) {
      action_call_id_ = logger_->log_lifecycle_event(
        "/move_pose_rl", "action_goal_received", "handle_goal", "received", "");
    }
    // Reject overlapping goals: a second goal accepted while the first is
    // still driving the shared DRL planner (/drl/clear_trajectory,
    // /drl/execute_forward) would race with it.
    std::lock_guard<std::mutex> lock(goal_active_mutex_);
    if (goal_active_) {
      RCLCPP_WARN(get_logger(), "Reject MovePoseRl goal: another goal is already active");
      if (logger_) {
        logger_->log_lifecycle_event(
          "/move_pose_rl", "action_goal_rejected", "handle_goal", "rejected",
          "another goal is already active", "", action_call_id_);
      }
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (logger_) {
      logger_->log_lifecycle_event(
        "/move_pose_rl", "action_goal_accepted", "handle_goal", "accepted", "",
        "", action_call_id_);
    }
    goal_active_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_WARN(get_logger(), "MovePoseRl cancel requested");
    // Real stop (codex.md section 7): don't just accept the cancel — also
    // halt the actual robot motion, which is running in the cartesian
    // executor via /move_cartesian_pose_sequence. Set the flag so execute()'s
    // wait loops bail out, and fire /move_cartesian_stop best-effort.
    cancel_requested_.store(true);
    request_cartesian_stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void request_cartesian_stop()
  {
    if (!cartesian_stop_client_->service_is_ready()) {
      RCLCPP_WARN(
        get_logger(),
        "MovePoseRl: /move_cartesian_stop not available; cannot halt cartesian motion");
      return;
    }
    RCLCPP_WARN(get_logger(), "MovePoseRl: calling /move_cartesian_stop to halt robot motion");
    cartesian_stop_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
  }

  void release_goal_slot()
  {
    std::lock_guard<std::mutex> lock(goal_active_mutex_);
    goal_active_ = false;
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

  bool transform_point_to_planning_frame(
    const geometry_msgs::msg::Point & point_in,
    const std::string & source_frame,
    geometry_msgs::msg::Point & point_out,
    std::string & error_msg)
  {
    if (source_frame.empty() || source_frame == planning_frame_) {
      point_out = point_in;
      return true;
    }
    geometry_msgs::msg::PointStamped stamped_in;
    stamped_in.header.frame_id = source_frame;
    stamped_in.header.stamp = builtin_interfaces::msg::Time();
    stamped_in.point = point_in;
    try {
      const auto stamped_out = tf_buffer_.transform(
        stamped_in, planning_frame_, tf2::durationFromSec(tf_timeout_sec_));
      point_out = stamped_out.point;
      return true;
    } catch (const std::exception & e) {
      error_msg = "TF transform " + source_frame + " -> " + planning_frame_ +
        " failed: " + e.what();
      return false;
    }
  }

  struct ObstacleResolution
  {
    bool ok = false;
    bool has_obstacle = false;
    std::string source;  // "vision" or "none"
    geometry_msgs::msg::Point center_base;
    geometry_msgs::msg::Vector3 size;
  };

  // codex2.md "move_pose_rl_obstacle_avoidance": best-effort vision obstacle
  // for MovePoseRl — MovePoseRl.action has no obstacle-related goal fields
  // (unlike MoveTargetRl/MoveToPoseObstacle), so there is nothing to "fail"
  // on when no box is visible; it just plans without one. This mirrors
  // MoveTargetRlActionServer::resolve_obstacle()'s vision branch exactly
  // (same selection rule: box closest to the current-TCP -> target segment).
  ObstacleResolution resolve_obstacle(
    const geometry_msgs::msg::Point & current_tcp_base,
    const geometry_msgs::msg::Point & target_base)
  {
    ObstacleResolution result;

    robot_vision_pipeline_msgs::msg::BoxArray box_snapshot;
    rclcpp::Time box_stamp;
    bool have_box;
    {
      std::lock_guard<std::mutex> lock(vision_mutex_);
      box_snapshot = latest_box_;
      box_stamp = latest_box_stamp_;
      have_box = have_box_;
    }

    const bool fresh = have_box && (now() - box_stamp).seconds() <= vision_timeout_sec_;
    if (!fresh) {
      result.ok = true;
      result.has_obstacle = false;
      result.source = "none";
      return result;
    }

    // Shared selection logic (robot_task_manager/rl_obstacle_input.hpp) so
    // MovePoseRL and PickPlaceRL feed the DRL policy an obstacle resolved the
    // exact same way.
    const robot_task_manager::RlObstacleInput shared = robot_task_manager::resolveRlObstacleInput(
      box_snapshot, obstacle_class_, current_tcp_base, target_base,
      [this](
        const geometry_msgs::msg::Point & in, const std::string & frame_in,
        geometry_msgs::msg::Point & out) {
        std::string err;
        return transform_point_to_planning_frame(in, frame_in, out, err);
      });

    result.ok = true;
    result.has_obstacle = shared.has_obstacle;
    result.source = shared.source;
    result.center_base = shared.center_base;
    result.size = shared.size;
    return result;
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

  // Common tail shared by abort_goal()/finish_success(): writes the
  // opt-in metrics CSV row exactly once, regardless of outcome.
  void log_metrics_and_release(bool success, const std::string & stage, const std::string & message)
  {
    if (metrics_row_) {
      metrics_row_->success = success;
      metrics_row_->action_result_success = success;
      metrics_row_->failed_stage = success ? "" : stage;
      metrics_row_->message = message;
      metrics_row_->total_action_time_s = (now() - goal_start_time_).seconds();
      metrics_logger_->finish(metrics_row_);
      metrics_row_.reset();
    }
    release_goal_slot();
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
    if (logger_) {
      logger_->log_lifecycle_event(
        "/move_pose_rl", "action_stage_failed", stage, "failed", message, "", action_call_id_);
      logger_->log_lifecycle_event(
        "/move_pose_rl", "action_result", stage, "aborted", message, "", action_call_id_);
    }
    log_metrics_and_release(false, stage, message);
    if (goal_handle->is_canceling() || cancel_requested_.load()) {
      result->failed_stage = "canceled";
      result->message = message.empty() ? "MovePoseRl canceled" : message;
      goal_handle->canceled(result);
      return;
    }
    goal_handle->abort(result);
  }

  // Twin of abort_goal() for the two success exit points (plan-only-early
  // and final "done"), giving the metrics logger a single success
  // chokepoint to match abort_goal()'s failure chokepoint.
  void finish_success(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::shared_ptr<MovePoseRl::Result> & result,
    const std::string & stage,
    const std::string & message)
  {
    result->success = true;
    result->message = message;
    result->failed_stage = "";
    if (logger_) {
      logger_->log_lifecycle_event(
        "/move_pose_rl", "action_succeeded", stage, "succeeded", message, "", action_call_id_);
      logger_->log_lifecycle_event(
        "/move_pose_rl", "action_result", stage, "succeeded", message, "", action_call_id_);
    }
    log_metrics_and_release(true, stage, message);
    goal_handle->succeed(result);
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
        {"/drl/get_planning_status", drl_planning_status_client_},
      } :
      std::vector<std::pair<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr>>{
        {"/drl/plan", drl_plan_client_},
        {"/drl/clear_trajectory", drl_clear_client_},
        {"/drl/get_planning_status", drl_planning_status_client_},
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
    bool has_obstacle,
    const geometry_msgs::msg::Point & obstacle_center,
    const geometry_msgs::msg::Vector3 & obstacle_size,
    std::string & error_msg)
  {
    auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    req->parameters.push_back(make_double_array_param(
      "manual_default_target",
      {target.position.x, target.position.y, target.position.z}));
    req->parameters.push_back(make_double_array_param(
      "manual_default_obstacle_center",
      {obstacle_center.x, obstacle_center.y, obstacle_center.z}));
    req->parameters.push_back(make_double_array_param(
      "manual_default_obstacle_size",
      {obstacle_size.x, obstacle_size.y, obstacle_size.z}));
    // manual_allow_skip_obstacle=false forces has_obstacle=true downstream in
    // drl_unified_planner_node._get_manual_default_input() whenever we
    // actually resolved a fresh vision obstacle (codex2.md
    // "move_pose_rl_obstacle_avoidance" — this is the exact wiring
    // MovePoseRl was missing; MoveTargetRl already does this).
    req->parameters.push_back(make_bool_param("manual_allow_skip_obstacle", !has_obstacle));
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
      if (cancel_requested_.load()) {
        request_cartesian_stop();
        error_msg = "DRL planning canceled by user Stop";
        return false;
      }
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

      // Poll /drl/get_planning_status so a failed /drl/plan (e.g. goal outside
      // the trained workspace) is detected within ~1s instead of only after
      // drl_timeout_sec_, since a failed plan never publishes
      // /drl/forward_trajectory_poses.
      std::string status_msg;
      const bool idle = call_trigger(
        drl_planning_status_client_,
        "/drl/get_planning_status",
        status_msg,
        2.0);
      if (idle && status_msg.rfind("FAILED", 0) == 0) {
        error_msg = "DRL planning failed: " + status_msg;
        return false;
      }

      std::this_thread::sleep_for(100ms);
    }
    error_msg = "Timed out waiting for DRL trajectory";
    return false;
  }

  bool copy_observation_after(
    uint64_t seq_before,
    std::vector<double> & raw_observation,
    std::vector<double> & model_observation,
    double timeout_sec = 2.0)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_sec);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(observation_mutex_);
        if (observation_seq_ > seq_before &&
          latest_raw_observation_.size() == 15 &&
          latest_model_observation_.size() == 15)
        {
          raw_observation = latest_raw_observation_;
          model_observation = latest_model_observation_;
          return true;
        }
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

  bool wait_for_drl_execution(std::string & error_msg)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(drl_timeout_sec_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (cancel_requested_.load()) {
        // Stop was requested: /move_cartesian_stop has been fired, so stop
        // waiting and report the cancel instead of driving to completion.
        request_cartesian_stop();
        error_msg = "DRL execution canceled by user Stop";
        return false;
      }
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
    const ObstacleResolution & obstacle,
    std::string & failed_stage,
    std::string & error_msg,
    std::vector<double> & raw_observation,
    std::vector<double> & model_observation)
  {
    const int attempts = std::max(1, drl_plan_attempts_);
    std::string last_error;
    for (int attempt = 1; attempt <= attempts; ++attempt) {
      if (cancel_requested_.load()) {
        failed_stage = "canceled";
        error_msg = "DRL planning canceled by user Stop";
        return false;
      }
      RCLCPP_INFO(
        get_logger(),
        "MovePoseRl DRL plan attempt %d/%d target=(%.4f %.4f %.4f)",
        attempt,
        attempts,
        target.position.x,
        target.position.y,
        target.position.z);

      if (!set_drl_target(
          target, obstacle.has_obstacle, obstacle.center_base, obstacle.size, last_error))
      {
        failed_stage = "drl_plan";
        error_msg = last_error;
        return false;
      }

      uint64_t seq_before = 0;
      {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        seq_before = trajectory_seq_;
      }
      uint64_t observation_seq_before = 0;
      {
        std::lock_guard<std::mutex> lock(observation_mutex_);
        observation_seq_before = observation_seq_;
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
        if (!copy_observation_after(observation_seq_before, raw_observation, model_observation)) {
          RCLCPP_WARN(
            get_logger(),
            "DRL trajectory was ready but no fresh /drl/last_plan_observation_15d arrived");
        }
        return true;
      }

      RCLCPP_WARN(
        get_logger(),
        "MovePoseRl DRL plan attempt %d/%d failed: %s",
        attempt,
        attempts,
        last_error.c_str());
      if (cancel_requested_.load()) {
        failed_stage = "canceled";
        error_msg = last_error.empty() ? "DRL planning canceled by user Stop" : last_error;
        return false;
      }
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
    goal_start_time_ = now();
    cancel_requested_.store(false);

    RCLCPP_INFO(
      get_logger(), "[move_pose_rl server] enable_metrics_log=%s",
      goal->enable_metrics_log ? "true" : "false");

    metrics_row_.reset();
    if (goal->enable_metrics_log) {
      metrics_row_ = metrics_logger_->startCall(
        "MovePoseRl", "rl", robot_task_manager::goalUuidHex(goal_handle->get_goal_id()));
      if (metrics_row_) {
        metrics_row_->execute_requested = execute_motion;
        metrics_row_->target_x = goal->target_pose.position.x;
        metrics_row_->target_y = goal->target_pose.position.y;
        metrics_row_->target_z = goal->target_pose.position.z;
      }
    }

    if (logger_) {
      logger_->log_lifecycle_event(
        "/move_pose_rl", "action_start", "execute", "started", "", "", action_call_id_);
    }

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
    if (metrics_row_) {
      metrics_row_->start_x = start_pose.pose.position.x;
      metrics_row_->start_y = start_pose.pose.position.y;
      metrics_row_->start_z = start_pose.pose.position.z;
      metrics_row_->start_source = "tf_lookup";
    }
    // codex2.md "move_pose_rl_current_tcp_start": make the actual start pose
    // used for this goal impossible to miss in the launch terminal — this is
    // a fresh TF lookup (planning_frame_ <- ee_link_) taken right here, at
    // goal-execution time, never a cached/default/last-plan value.
    RCLCPP_INFO(
      get_logger(),
      "[move_pose_rl] current TCP start pose: frame=%s position=(%.4f, %.4f, %.4f) "
      "orientation=(%.4f, %.4f, %.4f, %.4f)",
      planning_frame_.c_str(),
      start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z,
      start_pose.pose.orientation.x, start_pose.pose.orientation.y,
      start_pose.pose.orientation.z, start_pose.pose.orientation.w);

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.stamp = now();
    target_pose.header.frame_id = planning_frame_;
    target_pose.pose = goal->target_pose;
    RCLCPP_INFO(
      get_logger(),
      "[move_pose_rl] target pose: frame=%s position=(%.4f, %.4f, %.4f) "
      "orientation=(%.4f, %.4f, %.4f, %.4f)",
      planning_frame_.c_str(),
      target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z,
      target_pose.pose.orientation.x, target_pose.pose.orientation.y,
      target_pose.pose.orientation.z, target_pose.pose.orientation.w);
    const auto target_rpy_deg = rpy_deg_from_quat(target_pose.pose.orientation);
    RCLCPP_INFO(
      get_logger(),
      "[MovePoseRL] current_tcp=(%.4f, %.4f, %.4f)",
      start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z);
    RCLCPP_INFO(
      get_logger(),
      "[MovePoseRL] target_after_offset=(%.4f, %.4f, %.4f)",
      target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
    RCLCPP_INFO(
      get_logger(),
      "[MovePoseRL] target_rpy_deg=(%.2f, %.2f, %.2f)",
      target_rpy_deg[0], target_rpy_deg[1], target_rpy_deg[2]);

    // codex2.md "move_pose_rl_obstacle_avoidance": resolve a fresh vision
    // obstacle (best-effort — MovePoseRl.action has no obstacle goal fields
    // to fail on) and feed it into the DRL planner. Previously this server
    // never set manual_default_obstacle_center/size at all, so MovePoseRl
    // silently planned with whatever obstacle params some *other* RL action
    // (MoveTargetRl/DrlPickPlace) happened to leave on the shared planner
    // node — not real obstacle avoidance for this call.
    const ObstacleResolution obstacle = resolve_obstacle(
      start_pose.pose.position, target_pose.pose.position);
    if (obstacle.has_obstacle) {
      RCLCPP_INFO(
        get_logger(),
        "[move_pose_rl] use obstacle source=%s", box_objects_topic_.c_str());
      RCLCPP_INFO(
        get_logger(),
        "[move_pose_rl] selected obstacle center=(%.4f, %.4f, %.4f)",
        obstacle.center_base.x, obstacle.center_base.y, obstacle.center_base.z);
      RCLCPP_INFO(
        get_logger(),
        "[move_pose_rl] selected obstacle size=(%.4f, %.4f, %.4f)",
        obstacle.size.x, obstacle.size.y, obstacle.size.z);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "[move_pose_rl] no obstacle from %s; planning without obstacle avoidance",
        box_objects_topic_.c_str());
    }
    if (metrics_row_) {
      metrics_row_->has_obstacle = obstacle.has_obstacle;
      metrics_row_->obstacle_source = obstacle.source;
      metrics_row_->obstacle_class = obstacle_class_;
      metrics_row_->obstacle_frame = planning_frame_;
      metrics_row_->safety_margin_m = rl_obstacle_safety_margin_m_;
      if (obstacle.has_obstacle) {
        metrics_row_->obstacle_center_x = obstacle.center_base.x;
        metrics_row_->obstacle_center_y = obstacle.center_base.y;
        metrics_row_->obstacle_center_z = obstacle.center_base.z;
        metrics_row_->obstacle_size_x = obstacle.size.x;
        metrics_row_->obstacle_size_y = obstacle.size.y;
        metrics_row_->obstacle_size_z = obstacle.size.z;
      }
    }

    publish_feedback(goal_handle, "wait_for_drl_services", 25.0f);
    if (!wait_for_services(execute_motion, error_msg)) {
      abort_goal(goal_handle, result, "drl_plan", error_msg);
      return;
    }

    publish_feedback(goal_handle, "drl_plan", 45.0f);
    std::string failed_stage;
    std::vector<double> raw_observation;
    std::vector<double> model_observation;
    const auto drl_plan_start = std::chrono::steady_clock::now();
    const bool planned = plan_with_drl(
      target_pose.pose, obstacle, failed_stage, error_msg, raw_observation, model_observation);
    if (raw_observation.size() == 15) {
      RCLCPP_INFO(
        get_logger(),
        "[MovePoseRL] rl_observation_15d=%s",
        format_values(raw_observation).c_str());
    }

    std::vector<geometry_msgs::msg::Point> waypoints;
    std::vector<geometry_msgs::msg::Pose> planned_poses;
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      planned_poses = latest_trajectory_.poses;
      waypoints.reserve(latest_trajectory_.poses.size());
      for (const auto & pose : latest_trajectory_.poses) {
        waypoints.push_back(pose.position);
      }
    }

    // codex2.md "move_pose_rl_current_tcp_start" acceptance criteria: prove
    // the RL trajectory actually starts at the current TCP we just read,
    // not some other pose. Logged whenever a trajectory came back, whether
    // or not enable_metrics_log is set, so this is visible on every run.
    if (planned && !waypoints.empty()) {
      const auto & first_point = waypoints.front();
      const double distance_start_to_first_point = std::sqrt(
        std::pow(first_point.x - start_pose.pose.position.x, 2) +
        std::pow(first_point.y - start_pose.pose.position.y, 2) +
        std::pow(first_point.z - start_pose.pose.position.z, 2));
      RCLCPP_INFO(
        get_logger(),
        "[move_pose_rl] first trajectory point: position=(%.4f, %.4f, %.4f)",
        first_point.x, first_point.y, first_point.z);
      RCLCPP_INFO(
        get_logger(),
        "[move_pose_rl] distance_start_to_first_point=%.4f",
        distance_start_to_first_point);
      if (distance_start_to_first_point >= 0.02) {
        RCLCPP_WARN(
          get_logger(),
          "[move_pose_rl] distance_start_to_first_point=%.4f >= 0.02 m — RL trajectory "
          "does not start close to the current TCP pose; investigate.",
          distance_start_to_first_point);
      }
      if (metrics_row_) {
        metrics_row_->first_point_x = first_point.x;
        metrics_row_->first_point_y = first_point.y;
        metrics_row_->first_point_z = first_point.z;
        metrics_row_->distance_start_to_first_point = distance_start_to_first_point;
      }
    }

    // codex2.md "move_pose_rl_obstacle_avoidance" section 9/13: waypoint-level
    // clearance against the resolved obstacle AABB, logged whenever a
    // trajectory came back — this is only a coarse (waypoint-spaced) check
    // for a runtime log line; the report's Case A/B verification uses a
    // properly segment-sampled check offline against the same data.
    // Gated on `planned` (not just non-empty waypoints): /drl/forward_trajectory_poses
    // is TRANSIENT_LOCAL, so a failed plan_with_drl() call can still leave
    // latest_trajectory_ holding a *previous* successful call's trajectory —
    // computing clearance against that stale data would be misleading.
    if (planned && !waypoints.empty()) {
      robot_task_manager::AabbObstacle aabb;
      aabb.center = obstacle.center_base;
      aabb.size = obstacle.size;
      aabb.has_obstacle = obstacle.has_obstacle;
      const auto traj_metrics = robot_task_manager::computeTrajectoryMetrics(
        waypoints, start_pose.pose.position, target_pose.pose.position,
        obstacle.has_obstacle ? &aabb : nullptr, rl_obstacle_safety_margin_m_);
      if (obstacle.has_obstacle) {
        RCLCPP_INFO(
          get_logger(),
          "[move_pose_rl] min_distance_to_obstacle=%.4f (safety_margin=%.3f, "
          "collision_or_inside_count=%.0f)",
          traj_metrics.min_clearance_m, rl_obstacle_safety_margin_m_,
          traj_metrics.collision_or_inside_obstacle_count);
        if (traj_metrics.clearance_ok.has_value() && !*traj_metrics.clearance_ok) {
          RCLCPP_WARN(
            get_logger(),
            "[move_pose_rl] trajectory violates obstacle safety margin: "
            "min_clearance_with_margin_m=%.4f",
            traj_metrics.min_clearance_with_margin_m);
        }
      }
      if (metrics_row_) {
        metrics_row_->path_length_m = traj_metrics.path_length_m;
        metrics_row_->straight_line_distance_m = traj_metrics.straight_line_distance_m;
        metrics_row_->path_efficiency = traj_metrics.path_efficiency;
        metrics_row_->min_clearance_m = traj_metrics.min_clearance_m;
        metrics_row_->average_clearance_m = traj_metrics.average_clearance_m;
        metrics_row_->start_clearance_m = traj_metrics.start_clearance_m;
        metrics_row_->target_clearance_m = traj_metrics.target_clearance_m;
        metrics_row_->clearance_violation_count = traj_metrics.clearance_violation_count;
        metrics_row_->collision_or_inside_obstacle_count =
          traj_metrics.collision_or_inside_obstacle_count;
        metrics_row_->min_clearance_with_margin_m = traj_metrics.min_clearance_with_margin_m;
        metrics_row_->clearance_ok = traj_metrics.clearance_ok;
        if (raw_observation.size() == 15 && model_observation.size() == 15) {
          metrics_logger_->writeRlInput15d(
            *metrics_row_, "rl_observation.csv", "plan", raw_observation,
            model_observation, "drl_unified_planner_node:first_raw_observation");
        }
        metrics_logger_->writePlanningTrajectory(
          *metrics_row_, "rl_planning_path.csv", "plan", planned_poses,
          target_pose.pose.position, obstacle.has_obstacle ? &aabb : nullptr,
          false);
      }
    }

    if (metrics_row_) {
      metrics_row_->drl_plan_time_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - drl_plan_start).count();
      metrics_row_->planning_success = planned;
      metrics_row_->trajectory_points = static_cast<double>(waypoints.size());
    }
    if (!planned) {
      abort_goal(goal_handle, result, failed_stage, error_msg);
      return;
    }

    if (!execute_motion) {
      publish_feedback(goal_handle, "done_plan_only", 100.0f);
      finish_success(
        goal_handle, result, "done_plan_only",
        "DRL plan succeeded; execution skipped because execute=false");
      return;
    }

    if (goal_handle->is_canceling() || cancel_requested_.load()) {
      request_cartesian_stop();
      abort_goal(
        goal_handle, result, "canceled",
        "DRL execution canceled before execute_forward by user Stop");
      return;
    }

    publish_feedback(goal_handle, "execute_forward", 70.0f);
    robot_task_manager::AabbObstacle exec_aabb;
    exec_aabb.center = obstacle.center_base;
    exec_aabb.size = obstacle.size;
    exec_aabb.has_obstacle = obstacle.has_obstacle;
    auto tcp_sampler = metrics_logger_->startTcpExecutionSampling(
      metrics_row_, "execute_forward", target_pose.pose, planned_poses, {},
      obstacle.has_obstacle ? &exec_aabb : nullptr,
      [this](geometry_msgs::msg::PoseStamped & out, std::string & err) {
        return current_pose(out, err);
      },
      executor_sample_rate_hz_);
    std::string msg;
    if (!call_trigger(drl_execute_client_, "/drl/execute_forward", msg, 5.0)) {
      metrics_logger_->stopTcpExecutionSampling(tcp_sampler);
      abort_goal(goal_handle, result, "execute_forward", "Start DRL execution failed: " + msg);
      return;
    }

    publish_feedback(goal_handle, "execution_status", 85.0f);
    const auto drl_exec_wait_start = std::chrono::steady_clock::now();
    const bool exec_ok = wait_for_drl_execution(error_msg);
    if (metrics_row_) {
      metrics_row_->drl_execution_wait_time_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - drl_exec_wait_start).count();
      metrics_row_->execution_time_s = metrics_row_->drl_execution_wait_time_s;
      metrics_row_->execution_success = exec_ok;
    }
    metrics_logger_->stopTcpExecutionSampling(tcp_sampler);
    if (!exec_ok) {
      abort_goal(goal_handle, result, "execution_status", error_msg);
      return;
    }
    if (!verify_final_pose(target_pose.pose, error_msg)) {
      abort_goal(goal_handle, result, "execution_status", error_msg);
      return;
    }

    publish_feedback(goal_handle, "done", 100.0f);
    if (metrics_row_) {
      geometry_msgs::msg::PoseStamped final_pose;
      std::string final_error;
      if (current_pose(final_pose, final_error)) {
        metrics_row_->final_x = final_pose.pose.position.x;
        metrics_row_->final_y = final_pose.pose.position.y;
        metrics_row_->final_z = final_pose.pose.position.z;
        metrics_row_->final_position_error_m = position_error(final_pose.pose, target_pose.pose);
      }
    }
    finish_success(goal_handle, result, "done", "MovePoseRl completed successfully");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MovePoseRlActionServer>();
  node->initialize_logging();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
