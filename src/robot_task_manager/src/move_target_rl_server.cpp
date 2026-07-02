#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
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
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "robot_task_manager/action/move_target_rl.hpp"
#include "robot_task_manager/action_metrics_logger.hpp"
#include "robot_task_manager/log_paths.hpp"
#include "robot_task_executor/executor_experiment_logger.hpp"
#include "robot_vision_pipeline_msgs/msg/box_array.hpp"
#include "robot_vision_pipeline_msgs/msg/wood_array.hpp"

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

bool finite_point(const geometry_msgs::msg::Point & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

bool finite_vector3(const geometry_msgs::msg::Vector3 & v)
{
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool positive_size(const geometry_msgs::msg::Vector3 & v)
{
  // "Full size" obstacle: every axis must be a real, positive extent —
  // an all-zero or partially-zero size is treated as "no usable size",
  // matching codex.md's ban on ever sending [0,0,0] to the planner when
  // an obstacle was actually detected.
  return finite_vector3(v) && v.x > 1e-6 && v.y > 1e-6 && v.z > 1e-6;
}

double position_error(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Shortest distance from `point` to the segment [seg_a, seg_b], used to pick
// the box detection that best represents the obstacle actually standing
// between the current TCP and the wood target (codex.md 9: "box gần đoạn
// thẳng current TCP -> target nhất").
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

class MoveTargetRlActionServer : public rclcpp::Node
{
public:
  using MoveTargetRl = robot_task_manager::action::MoveTargetRl;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MoveTargetRl>;

  MoveTargetRlActionServer()
  : Node("move_target_rl_action_server"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    planning_frame_ = declare_parameter<std::string>("planning_frame", "base_link");
    ee_link_ = declare_parameter<std::string>("ee_link", "tcp_link");
    target_class_ = declare_parameter<std::string>("target_class", "wood");
    obstacle_class_ = declare_parameter<std::string>("obstacle_class", "box");
    vision_timeout_sec_ = declare_parameter<double>("vision_timeout_sec", 1.0);
    default_box_size_ = declare_parameter<std::vector<double>>(
      "default_box_size", std::vector<double>{0.10, 0.10, 0.10});
    target_position_tolerance_m_ =
      declare_parameter<double>("target_position_tolerance_m", 0.02);
    drl_planner_node_name_ = declare_parameter<std::string>(
      "drl_planner_node_name", "/drl_unified_planner_node");
    wood_objects_topic_ = declare_parameter<std::string>(
      "wood_objects_topic", "/vision/wood_objects");
    box_objects_topic_ = declare_parameter<std::string>(
      "box_objects_topic", "/vision/box_objects");

    position_tolerance_m_ = declare_parameter<double>("position_tolerance_m", 0.01);
    drl_timeout_sec_ = declare_parameter<double>("drl_timeout_sec", 120.0);
    drl_trajectory_endpoint_tolerance_m_ =
      declare_parameter<double>("drl_trajectory_endpoint_tolerance_m", 0.015);
    drl_plan_attempts_ = declare_parameter<int>("drl_plan_attempts", 3);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 2.0);
    sub_action_timeout_sec_ = declare_parameter<double>("sub_action_timeout_sec", 60.0);

    workspace_min_base_ = declare_parameter<std::vector<double>>(
      "workspace_min_base", std::vector<double>{0.20, -0.25, 0.0});
    workspace_max_base_ = declare_parameter<std::vector<double>>(
      "workspace_max_base", std::vector<double>{0.60, 0.25, 0.50});

    enable_executor_logging_ = declare_parameter<bool>("enable_executor_logging", false);
    log_root_dir_            = declare_parameter<std::string>("log_root_dir", robot_task_manager::kDefaultLogRootDir);
    executor_log_dir_        = declare_parameter<std::string>(
      "executor_log_dir", robot_task_manager::executorLogBaseDir(log_root_dir_));
    executor_sample_rate_hz_ = declare_parameter<double>("executor_sample_rate_hz", 50.0);
    executor_base_frame_     = declare_parameter<std::string>("executor_base_frame", "base_link");
    executor_tcp_frame_      = declare_parameter<std::string>("executor_tcp_frame", "tcp_link");

    set_planner_params_client_ =
      create_client<rcl_interfaces::srv::SetParameters>(drl_planner_node_name_ + "/set_parameters");
    drl_plan_client_ = create_client<std_srvs::srv::Trigger>("/drl/plan");
    drl_clear_client_ = create_client<std_srvs::srv::Trigger>("/drl/clear_trajectory");
    drl_execute_client_ = create_client<std_srvs::srv::Trigger>("/drl/execute_forward");
    drl_status_client_ = create_client<std_srvs::srv::Trigger>("/drl/get_execution_status");
    drl_planning_status_client_ = create_client<std_srvs::srv::Trigger>("/drl/get_planning_status");
    // Stop path (codex.md 7.6): RL robot motion runs in the cartesian executor
    // via /move_cartesian_pose_sequence; a real Stop must halt it there.
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

    // Vision detections are published BEST_EFFORT/VOLATILE by
    // pixel_to_base_mapper_node (see robot_vision_pipeline config
    // pixel_to_base_mapper.yaml) — the subscription QoS must be compatible.
    auto vision_qos = rclcpp::QoS(1).best_effort().durability_volatile();
    wood_sub_ = create_subscription<robot_vision_pipeline_msgs::msg::WoodArray>(
      wood_objects_topic_,
      vision_qos,
      [this](const robot_vision_pipeline_msgs::msg::WoodArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(vision_mutex_);
        latest_wood_ = *msg;
        latest_wood_stamp_ = now();
        have_wood_ = true;
      });
    box_sub_ = create_subscription<robot_vision_pipeline_msgs::msg::BoxArray>(
      box_objects_topic_,
      vision_qos,
      [this](const robot_vision_pipeline_msgs::msg::BoxArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(vision_mutex_);
        latest_box_ = *msg;
        latest_box_stamp_ = now();
        have_box_ = true;
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

    metrics_logger_ = std::make_shared<robot_task_manager::ActionMetricsLogger>(
      robot_task_manager::actionMetricsLogDir(log_root_dir_, "MoveTargetRl"), get_logger());

    action_server_ = rclcpp_action::create_server<MoveTargetRl>(
      this,
      "move_target_rl",
      std::bind(&MoveTargetRlActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveTargetRlActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MoveTargetRlActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "MoveTargetRl action server ready: /move_target_rl | target_class=%s obstacle_class=%s "
      "wood_topic=%s box_topic=%s",
      target_class_.c_str(), obstacle_class_.c_str(),
      wood_objects_topic_.c_str(), box_objects_topic_.c_str());
  }

  // Mirrors MoveItExecutor::initializeLogging()/MovePoseRl's pattern: called
  // once after make_shared(), before spin(), because shared_from_this()
  // cannot be used inside the constructor.
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
        robot_task_manager::executorActionLogDir(log_root_dir_, executor_log_dir_, "MoveTargetRl"),
        executor_sample_rate_hz_,
        executor_base_frame_, executor_tcp_frame_);
    } catch (const std::exception & e) {
      logger_.reset();
      RCLCPP_WARN(get_logger(), "MoveTargetRl CSV logger unavailable: %s", e.what());
    }
  }

private:
  // ---------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------
  std::string planning_frame_;
  std::string ee_link_;
  std::string target_class_;
  std::string obstacle_class_;
  double vision_timeout_sec_{1.0};
  std::vector<double> default_box_size_;
  double target_position_tolerance_m_{0.02};
  std::string drl_planner_node_name_;
  std::string wood_objects_topic_;
  std::string box_objects_topic_;

  double position_tolerance_m_{0.01};
  double drl_timeout_sec_{120.0};
  double drl_trajectory_endpoint_tolerance_m_{0.015};
  int drl_plan_attempts_{3};
  double tf_timeout_sec_{2.0};
  double sub_action_timeout_sec_{60.0};

  std::vector<double> workspace_min_base_;
  std::vector<double> workspace_max_base_;

  bool enable_executor_logging_{false};
  std::string log_root_dir_;
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

  rclcpp_action::Server<MoveTargetRl>::SharedPtr action_server_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_planner_params_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr drl_plan_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr drl_clear_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr drl_execute_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr drl_status_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr drl_planning_status_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cartesian_stop_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<robot_vision_pipeline_msgs::msg::WoodArray>::SharedPtr wood_sub_;
  rclcpp::Subscription<robot_vision_pipeline_msgs::msg::BoxArray>::SharedPtr box_sub_;

  std::atomic<bool> cancel_requested_{false};

  std::mutex trajectory_mutex_;
  geometry_msgs::msg::PoseArray latest_trajectory_;
  uint64_t trajectory_seq_{0};

  // VisionResultCache: latest detection batch per class, plus arrival time
  // used for the staleness check (vision_timeout_sec_). robot_vision_pipeline
  // already batches every currently-visible object of a class into one
  // WoodArray/BoxArray message, so caching "latest message" is sufficient —
  // no need to merge across messages.
  std::mutex vision_mutex_;
  robot_vision_pipeline_msgs::msg::WoodArray latest_wood_;
  rclcpp::Time latest_wood_stamp_;
  bool have_wood_{false};
  robot_vision_pipeline_msgs::msg::BoxArray latest_box_;
  rclcpp::Time latest_box_stamp_;
  bool have_box_{false};

  std::mutex joint_state_mutex_;
  bool received_joint_state_{false};

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
    std::shared_ptr<const MoveTargetRl::Goal>)
  {
    if (logger_) {
      action_call_id_ = logger_->log_lifecycle_event(
        "/move_target_rl", "action_goal_received", "handle_goal", "received", "");
    }
    // Reject overlapping goals: a second goal accepted while the first is
    // still driving the shared DRL planner (/drl/clear_trajectory,
    // /drl/execute_forward) would race with it — same guard as MovePoseRl.
    std::lock_guard<std::mutex> lock(goal_active_mutex_);
    if (goal_active_) {
      RCLCPP_WARN(get_logger(), "Reject MoveTargetRl goal: another goal is already active");
      if (logger_) {
        logger_->log_lifecycle_event(
          "/move_target_rl", "action_goal_rejected", "handle_goal", "rejected",
          "another goal is already active", "", action_call_id_);
      }
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (logger_) {
      logger_->log_lifecycle_event(
        "/move_target_rl", "action_goal_accepted", "handle_goal", "accepted", "",
        "", action_call_id_);
    }
    goal_active_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_WARN(get_logger(), "MoveTargetRl cancel requested");
    // Real stop (codex.md section 7): halt the actual robot motion running in
    // the cartesian executor, not just accept the wrapper-action cancel.
    cancel_requested_.store(true);
    request_cartesian_stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void request_cartesian_stop()
  {
    if (!cartesian_stop_client_->service_is_ready()) {
      RCLCPP_WARN(
        get_logger(),
        "MoveTargetRl: /move_cartesian_stop not available; cannot halt cartesian motion");
      return;
    }
    RCLCPP_WARN(get_logger(), "MoveTargetRl: calling /move_cartesian_stop to halt robot motion");
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
    std::thread(&MoveTargetRlActionServer::execute, this, goal_handle).detach();
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

  // Transforms `pose_in` (already stamped with its source frame_id) into
  // planning_frame_ using TF2. If the message is already in planning_frame_
  // this is a no-op copy — no manual offset is ever added (codex.md 10:
  // "Không được cộng offset thủ công nếu đã có TF").  Returns false (with
  // error_msg filled) if the TF is unavailable — callers must map that to
  // failed_stage="tf_transform_failed".
  bool transform_to_planning_frame(
    const geometry_msgs::msg::PoseStamped & pose_in,
    geometry_msgs::msg::PoseStamped & pose_out,
    std::string & error_msg)
  {
    if (pose_in.header.frame_id.empty() || pose_in.header.frame_id == planning_frame_) {
      pose_out = pose_in;
      pose_out.header.frame_id = planning_frame_;
      return true;
    }
    try {
      pose_out = tf_buffer_.transform(
        pose_in, planning_frame_, tf2::durationFromSec(tf_timeout_sec_));
      return true;
    } catch (const std::exception & e) {
      error_msg = "TF transform " + pose_in.header.frame_id + " -> " + planning_frame_ +
        " failed: " + e.what();
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

  // -----------------------------------------------------------------------
  // Vision target/obstacle resolution
  // -----------------------------------------------------------------------

  struct TargetResolution
  {
    bool ok = false;
    std::string source;  // "vision" or "fallback"
    geometry_msgs::msg::PoseStamped pose_base;  // already in planning_frame_
    std::string failed_stage;
    std::string error_msg;
  };

  struct ObstacleResolution
  {
    bool ok = false;
    bool has_obstacle = false;
    std::string source;  // "vision", "fallback", or "none"
    geometry_msgs::msg::PoseStamped pose_base;  // center, in planning_frame_
    geometry_msgs::msg::Vector3 size;
    std::string failed_stage;
    std::string error_msg;
  };

  TargetResolution resolve_target(const MoveTargetRl::Goal & goal)
  {
    TargetResolution result;
    if (goal.use_fallback_target) {
      if (!finite_pose(goal.fallback_target_pose)) {
        result.failed_stage = "validate_goal";
        result.error_msg = "fallback_target_pose contains non-finite values";
        return result;
      }
      result.pose_base.header.frame_id = planning_frame_;
      result.pose_base.header.stamp = now();
      result.pose_base.pose = goal.fallback_target_pose;
      result.source = "fallback";
      result.ok = true;
      return result;
    }

    const std::string target_class =
      goal.target_class.empty() ? target_class_ : goal.target_class;

    robot_vision_pipeline_msgs::msg::WoodArray wood_snapshot;
    rclcpp::Time wood_stamp;
    bool have_wood;
    {
      std::lock_guard<std::mutex> lock(vision_mutex_);
      wood_snapshot = latest_wood_;
      wood_stamp = latest_wood_stamp_;
      have_wood = have_wood_;
    }

    if (!have_wood ||
      (now() - wood_stamp).seconds() > vision_timeout_sec_)
    {
      result.failed_stage = "vision_target_3d_unavailable";
      result.error_msg = "No fresh detection on " + wood_objects_topic_ +
        " (class=" + target_class + ", timeout=" + std::to_string(vision_timeout_sec_) + "s)";
      return result;
    }

    // Pick the matching-class detection with the highest confidence
    // (codex.md 9: "Chọn detection class wood có confidence cao nhất").
    const robot_vision_pipeline_msgs::msg::Wood * best = nullptr;
    for (const auto & wood : wood_snapshot.woods) {
      if (wood.class_name != target_class) {
        continue;
      }
      if (!finite_pose(wood.pose)) {
        continue;
      }
      if (best == nullptr || wood.confidence > best->confidence) {
        best = &wood;
      }
    }
    if (best == nullptr) {
      result.failed_stage = "vision_target_3d_unavailable";
      result.error_msg = "No valid Wood detection with class_name='" + target_class +
        "' and finite pose in latest " + wood_objects_topic_ + " message";
      return result;
    }

    geometry_msgs::msg::PoseStamped wood_pose_in;
    wood_pose_in.header = wood_snapshot.header;
    wood_pose_in.pose = best->pose;

    geometry_msgs::msg::PoseStamped wood_pose_base;
    if (!transform_to_planning_frame(wood_pose_in, wood_pose_base, result.error_msg)) {
      result.failed_stage = "tf_transform_failed";
      return result;
    }

    result.pose_base = wood_pose_base;
    result.source = "vision";
    result.ok = true;
    return result;
  }

  ObstacleResolution resolve_obstacle(
    const MoveTargetRl::Goal & goal,
    const geometry_msgs::msg::Point & current_tcp_base,
    const geometry_msgs::msg::Point & target_base)
  {
    ObstacleResolution result;

    if (goal.use_fallback_obstacle) {
      if (!finite_point(goal.fallback_obstacle_center) ||
        !finite_vector3(goal.fallback_obstacle_size))
      {
        result.failed_stage = "validate_goal";
        result.error_msg = "fallback_obstacle_center/size contains non-finite values";
        return result;
      }
      if (!positive_size(goal.fallback_obstacle_size)) {
        if (goal.require_obstacle) {
          result.failed_stage = "vision_obstacle_size_unavailable";
          result.error_msg = "fallback_obstacle_size is not a valid positive full size "
            "and require_obstacle=true";
          return result;
        }
        result.ok = true;
        result.has_obstacle = false;
        result.source = "none";
        return result;
      }
      result.pose_base.header.frame_id = planning_frame_;
      result.pose_base.header.stamp = now();
      result.pose_base.pose.position = goal.fallback_obstacle_center;
      result.pose_base.pose.orientation.w = 1.0;
      result.size = goal.fallback_obstacle_size;
      result.source = "fallback";
      result.has_obstacle = true;
      result.ok = true;
      return result;
    }

    const std::string obstacle_class =
      goal.obstacle_class.empty() ? obstacle_class_ : goal.obstacle_class;

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
      if (goal.require_obstacle) {
        result.failed_stage = "vision_obstacle_3d_unavailable";
        result.error_msg = "No fresh detection on " + box_objects_topic_ +
          " (class=" + obstacle_class + ", timeout=" + std::to_string(vision_timeout_sec_) + "s)";
        return result;
      }
      result.ok = true;
      result.has_obstacle = false;
      result.source = "none";
      return result;
    }

    // Transform every matching-class box candidate to planning_frame_ first,
    // then pick the one closest to the current-TCP -> target segment
    // (codex.md 9: "Chọn detection class box gần đoạn thẳng current TCP ->
    // target nhất"). If there is only one candidate, it is used directly.
    struct Candidate
    {
      geometry_msgs::msg::Point center_base;
      geometry_msgs::msg::Vector3 size;
    };
    std::vector<Candidate> candidates;
    std::string transform_error;
    bool any_transform_failed = false;
    for (const auto & box : box_snapshot.boxes) {
      if (box.class_name != obstacle_class) {
        continue;
      }
      if (!finite_pose(box.pose)) {
        continue;
      }
      geometry_msgs::msg::Point center_base;
      if (!transform_point_to_planning_frame(
          box.pose.position, box_snapshot.header.frame_id, center_base, transform_error))
      {
        any_transform_failed = true;
        continue;
      }
      Candidate c;
      c.center_base = center_base;
      c.size = box.size;
      if (!positive_size(c.size)) {
        // Vision gave a pose but no usable size — fall back to the
        // configured default_box_size for this candidate (codex.md 5.2).
        if (default_box_size_.size() == 3 &&
          default_box_size_[0] > 1e-6 && default_box_size_[1] > 1e-6 && default_box_size_[2] > 1e-6)
        {
          c.size.x = default_box_size_[0];
          c.size.y = default_box_size_[1];
          c.size.z = default_box_size_[2];
        } else {
          continue;
        }
      }
      candidates.push_back(c);
    }

    if (candidates.empty()) {
      if (goal.require_obstacle) {
        if (any_transform_failed) {
          result.failed_stage = "tf_transform_failed";
          result.error_msg = transform_error;
        } else {
          result.failed_stage = "vision_obstacle_size_unavailable";
          result.error_msg = "No Box detection with class_name='" + obstacle_class +
            "', finite pose, and a usable size (vision or default_box_size fallback)";
        }
        return result;
      }
      result.ok = true;
      result.has_obstacle = false;
      result.source = "none";
      return result;
    }

    const Candidate * chosen = &candidates.front();
    if (candidates.size() > 1) {
      double best_dist = point_to_segment_distance(
        candidates.front().center_base, current_tcp_base, target_base);
      for (size_t i = 1; i < candidates.size(); ++i) {
        const double d = point_to_segment_distance(
          candidates[i].center_base, current_tcp_base, target_base);
        if (d < best_dist) {
          best_dist = d;
          chosen = &candidates[i];
        }
      }
    }

    result.pose_base.header.frame_id = planning_frame_;
    result.pose_base.header.stamp = now();
    result.pose_base.pose.position = chosen->center_base;
    result.pose_base.pose.orientation.w = 1.0;
    result.size = chosen->size;
    result.source = "vision";
    result.has_obstacle = true;
    result.ok = true;
    return result;
  }

  bool validate_target_workspace(const geometry_msgs::msg::Point & target, std::string & error_msg)
  {
    if (workspace_min_base_.size() != 3 || workspace_max_base_.size() != 3) {
      return true;
    }
    const bool inside =
      target.x >= workspace_min_base_[0] && target.x <= workspace_max_base_[0] &&
      target.y >= workspace_min_base_[1] && target.y <= workspace_max_base_[1] &&
      target.z >= workspace_min_base_[2] && target.z <= workspace_max_base_[2];
    if (!inside) {
      error_msg = "target position (" + std::to_string(target.x) + ", " +
        std::to_string(target.y) + ", " + std::to_string(target.z) +
        ") is outside workspace_min_base/workspace_max_base";
      return false;
    }
    return true;
  }

  void publish_feedback(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::string & stage,
    float progress,
    const geometry_msgs::msg::PoseStamped & target_pose,
    const geometry_msgs::msg::PoseStamped & obstacle_pose)
  {
    auto feedback = std::make_shared<MoveTargetRl::Feedback>();
    feedback->current_stage = stage;
    feedback->progress = progress;
    std::string pose_error;
    current_pose(feedback->current_pose, pose_error);
    feedback->target_pose = target_pose;
    feedback->obstacle_pose = obstacle_pose;
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(get_logger(), "[MoveTargetRl] %s | %.1f%%", stage.c_str(), progress);
  }

  // Common tail shared by abort_goal()/finish_success(): writes the
  // opt-in metrics CSV row exactly once, regardless of outcome, so
  // "success=false must still log" holds without duplicating this logic
  // at both call sites.
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
    const std::shared_ptr<MoveTargetRl::Result> & result,
    const std::string & stage,
    const std::string & message)
  {
    result->success = false;
    result->message = message;
    result->failed_stage = stage;
    RCLCPP_ERROR(get_logger(), "MoveTargetRl failed at %s: %s", stage.c_str(), message.c_str());
    if (logger_) {
      logger_->log_lifecycle_event(
        "/move_target_rl", "action_stage_failed", stage, "failed", message, "", action_call_id_);
      logger_->log_lifecycle_event(
        "/move_target_rl", "action_result", stage, "aborted", message, "", action_call_id_);
    }
    log_metrics_and_release(false, stage, message);
    if (goal_handle->is_canceling() || cancel_requested_.load()) {
      result->failed_stage = "canceled";
      result->message = message.empty() ? "MoveTargetRl canceled" : message;
      goal_handle->canceled(result);
      return;
    }
    goal_handle->abort(result);
  }

  // Twin of abort_goal() for the two success exit points (plan-only-early
  // and final "done") so both route through one place, giving the metrics
  // logger a single success chokepoint to match abort_goal()'s failure
  // chokepoint.
  void finish_success(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::shared_ptr<MoveTargetRl::Result> & result,
    const std::string & stage,
    const std::string & message)
  {
    result->success = true;
    result->message = message;
    result->failed_stage = "";
    if (logger_) {
      logger_->log_lifecycle_event(
        "/move_target_rl", "action_succeeded", stage, "succeeded", message, "", action_call_id_);
      logger_->log_lifecycle_event(
        "/move_target_rl", "action_result", stage, "succeeded", message, "", action_call_id_);
    }
    log_metrics_and_release(true, stage, message);
    goal_handle->succeed(result);
  }

  bool validate_goal(const MoveTargetRl::Goal & goal, std::string & error_msg)
  {
    if (!std::isfinite(goal.velocity_scale) ||
      goal.velocity_scale <= 0.0 ||
      goal.velocity_scale > 1.0)
    {
      error_msg = "velocity_scale must be finite and in (0, 1]";
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
        if (!set_planner_params_client_->wait_for_service(timeout)) {
          error_msg = "missing " + service_name;
          return false;
        }
        return true;
      };
    const auto wait_for_trigger =
      [timeout, &error_msg](
        const std::string & service_name,
        const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client) {
        if (!client->wait_for_service(timeout)) {
          error_msg = "missing " + service_name;
          return false;
        }
        return true;
      };

    const std::string parameter_service = drl_planner_node_name_ + "/set_parameters";
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

  bool set_drl_target_and_obstacle(
    const geometry_msgs::msg::Point & target,
    bool has_obstacle,
    const geometry_msgs::msg::Point & obstacle_center,
    const geometry_msgs::msg::Vector3 & obstacle_size,
    std::string & error_msg)
  {
    auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    req->parameters.push_back(make_double_array_param(
      "manual_default_target", {target.x, target.y, target.z}));
    req->parameters.push_back(make_double_array_param(
      "manual_default_obstacle_center",
      {obstacle_center.x, obstacle_center.y, obstacle_center.z}));
    req->parameters.push_back(make_double_array_param(
      "manual_default_obstacle_size",
      {obstacle_size.x, obstacle_size.y, obstacle_size.z}));
    // manual_allow_skip_obstacle=false forces has_obstacle=true downstream
    // in drl_unified_planner_node._get_manual_default_input() whenever we
    // actually resolved an obstacle — see Reports/rl_move_pose_rl_mode_input_audit_report.md
    // section 6 for why MovePoseRl never gets obstacle avoidance today.
    req->parameters.push_back(make_bool_param("manual_allow_skip_obstacle", !has_obstacle));
    req->parameters.push_back(make_bool_param("preposition_before_plan", false));
    req->parameters.push_back(make_bool_param("update_start_tcp_from_tf_before_plan", true));
    req->parameters.push_back(make_bool_param("auto_execute_after_plan", false));

    auto future = set_planner_params_client_->async_send_request(req);
    if (future.wait_for(5s) != std::future_status::ready) {
      error_msg = "Timeout setting DRL planner target/obstacle parameters";
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
    const geometry_msgs::msg::Point & target,
    uint32_t & trajectory_points,
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
          const double pos_err = position_error(final_pose.position, target);
          const double allowed_position = std::max(
            position_tolerance_m_,
            drl_trajectory_endpoint_tolerance_m_);
          if (pos_err > allowed_position) {
            error_msg = "DRL trajectory final waypoint position error too large: " +
              std::to_string(pos_err);
            return false;
          }
          trajectory_points = static_cast<uint32_t>(latest_trajectory_.poses.size());
          RCLCPP_INFO(
            get_logger(),
            "DRL trajectory ready: waypoints=%u final_position_error=%.5f",
            trajectory_points,
            pos_err);
          return true;
        }
      }

      // Poll /drl/get_planning_status so a failed /drl/plan (e.g. target or
      // obstacle rejected, workspace/IK failure) is detected within ~1s
      // instead of only after drl_timeout_sec_.
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

  bool wait_for_drl_execution(std::string & error_msg)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(drl_timeout_sec_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (cancel_requested_.load()) {
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
    const geometry_msgs::msg::Point & target,
    bool has_obstacle,
    const geometry_msgs::msg::Point & obstacle_center,
    const geometry_msgs::msg::Vector3 & obstacle_size,
    uint32_t & trajectory_points,
    std::string & failed_stage,
    std::string & error_msg)
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
        "MoveTargetRl DRL plan attempt %d/%d target=(%.4f %.4f %.4f) has_obstacle=%s "
        "obstacle_center=(%.4f %.4f %.4f) obstacle_size=(%.4f %.4f %.4f)",
        attempt, attempts,
        target.x, target.y, target.z,
        has_obstacle ? "true" : "false",
        obstacle_center.x, obstacle_center.y, obstacle_center.z,
        obstacle_size.x, obstacle_size.y, obstacle_size.z);

      if (!set_drl_target_and_obstacle(
          target, has_obstacle, obstacle_center, obstacle_size, last_error))
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

      std::string msg;
      if (!call_trigger(drl_clear_client_, "/drl/clear_trajectory", msg, 5.0)) {
        last_error = "Clear DRL trajectory failed: " + msg;
        failed_stage = "drl_plan";
      } else if (!call_trigger(drl_plan_client_, "/drl/plan", msg, 5.0)) {
        last_error = "Start DRL planning failed: " + msg;
        failed_stage = "drl_plan";
      } else if (!wait_for_planned_trajectory(seq_before, target, trajectory_points, last_error)) {
        failed_stage = "endpoint_check";
      } else {
        return true;
      }

      RCLCPP_WARN(
        get_logger(),
        "MoveTargetRl DRL plan attempt %d/%d failed: %s",
        attempt, attempts, last_error.c_str());
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
    const geometry_msgs::msg::Point & target,
    std::string & error_msg)
  {
    geometry_msgs::msg::PoseStamped actual;
    if (!current_pose(actual, error_msg)) {
      return false;
    }
    const double pos_err = position_error(actual.pose.position, target);
    if (pos_err > position_tolerance_m_) {
      error_msg = "Final TCP position tolerance failed: " + std::to_string(pos_err);
      return false;
    }
    return true;
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto result = std::make_shared<MoveTargetRl::Result>();
    const auto goal = goal_handle->get_goal();
    const bool execute_motion = goal->execute;
    std::string error_msg;
    goal_start_time_ = now();
    cancel_requested_.store(false);

    RCLCPP_INFO(
      get_logger(), "[move_target_rl server] enable_metrics_log=%s",
      goal->enable_metrics_log ? "true" : "false");

    metrics_row_.reset();
    if (goal->enable_metrics_log) {
      metrics_row_ = metrics_logger_->startCall(
        "MoveTargetRl", "rl", robot_task_manager::goalUuidHex(goal_handle->get_goal_id()));
      if (metrics_row_) {
        metrics_row_->execute_requested = execute_motion;
        if (default_box_size_.size() == 3) {
          metrics_row_->manual_default_obstacle_size_x = default_box_size_[0];
          metrics_row_->manual_default_obstacle_size_y = default_box_size_[1];
          metrics_row_->manual_default_obstacle_size_z = default_box_size_[2];
        }
        metrics_row_->workspace_min_x = workspace_min_base_.size() == 3 ? workspace_min_base_[0] :
          std::numeric_limits<double>::quiet_NaN();
        metrics_row_->workspace_min_y = workspace_min_base_.size() == 3 ? workspace_min_base_[1] :
          std::numeric_limits<double>::quiet_NaN();
        metrics_row_->workspace_min_z = workspace_min_base_.size() == 3 ? workspace_min_base_[2] :
          std::numeric_limits<double>::quiet_NaN();
        metrics_row_->workspace_max_x = workspace_max_base_.size() == 3 ? workspace_max_base_[0] :
          std::numeric_limits<double>::quiet_NaN();
        metrics_row_->workspace_max_y = workspace_max_base_.size() == 3 ? workspace_max_base_[1] :
          std::numeric_limits<double>::quiet_NaN();
        metrics_row_->workspace_max_z = workspace_max_base_.size() == 3 ? workspace_max_base_[2] :
          std::numeric_limits<double>::quiet_NaN();
      }
    }

    geometry_msgs::msg::PoseStamped empty_pose;
    empty_pose.header.frame_id = planning_frame_;

    if (logger_) {
      logger_->log_lifecycle_event(
        "/move_target_rl", "action_start", "execute", "started", "", "", action_call_id_);
    }

    publish_feedback(goal_handle, "validate_goal", 5.0f, empty_pose, empty_pose);
    if (!validate_goal(*goal, error_msg)) {
      abort_goal(goal_handle, result, "validate_goal", error_msg);
      return;
    }

    publish_feedback(goal_handle, "get_current_pose", 10.0f, empty_pose, empty_pose);
    if (!wait_for_joint_state(error_msg)) {
      abort_goal(goal_handle, result, "get_current_pose", error_msg);
      return;
    }
    geometry_msgs::msg::PoseStamped start_pose;
    if (!current_pose(start_pose, error_msg)) {
      abort_goal(goal_handle, result, "get_current_pose", error_msg);
      return;
    }
    RCLCPP_INFO(
      get_logger(), "MoveTargetRl current_tcp_base=(%.4f, %.4f, %.4f)",
      start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z);
    if (metrics_row_) {
      metrics_row_->start_x = start_pose.pose.position.x;
      metrics_row_->start_y = start_pose.pose.position.y;
      metrics_row_->start_z = start_pose.pose.position.z;
    }

    publish_feedback(goal_handle, "resolve_target", 20.0f, empty_pose, empty_pose);
    const TargetResolution target_res = resolve_target(*goal);
    if (!target_res.ok) {
      abort_goal(goal_handle, result, target_res.failed_stage, target_res.error_msg);
      return;
    }
    result->target_pose = target_res.pose_base;
    if (metrics_row_) {
      metrics_row_->target_x = target_res.pose_base.pose.position.x;
      metrics_row_->target_y = target_res.pose_base.pose.position.y;
      metrics_row_->target_z = target_res.pose_base.pose.position.z;
      metrics_row_->source = target_res.source;
    }
    RCLCPP_INFO(
      get_logger(), "MoveTargetRl target_wood_base=(%.4f, %.4f, %.4f) source=%s",
      target_res.pose_base.pose.position.x, target_res.pose_base.pose.position.y,
      target_res.pose_base.pose.position.z, target_res.source.c_str());

    if (!validate_target_workspace(target_res.pose_base.pose.position, error_msg)) {
      abort_goal(goal_handle, result, "validate_goal", error_msg);
      return;
    }

    publish_feedback(goal_handle, "resolve_obstacle", 30.0f, target_res.pose_base, empty_pose);
    const ObstacleResolution obstacle_res = resolve_obstacle(
      *goal, start_pose.pose.position, target_res.pose_base.pose.position);
    if (!obstacle_res.ok) {
      abort_goal(goal_handle, result, obstacle_res.failed_stage, obstacle_res.error_msg);
      return;
    }
    if (goal->require_obstacle && !obstacle_res.has_obstacle) {
      abort_goal(
        goal_handle, result, "vision_obstacle_3d_unavailable",
        "require_obstacle=true but no obstacle could be resolved");
      return;
    }
    result->obstacle_pose = obstacle_res.pose_base;
    result->obstacle_size = obstacle_res.size;
    if (metrics_row_) {
      metrics_row_->has_obstacle = obstacle_res.has_obstacle;
      metrics_row_->rl_has_obstacle = obstacle_res.has_obstacle;
      metrics_row_->obstacle_source = obstacle_res.source;
      metrics_row_->obstacle_center_x = obstacle_res.pose_base.pose.position.x;
      metrics_row_->obstacle_center_y = obstacle_res.pose_base.pose.position.y;
      metrics_row_->obstacle_center_z = obstacle_res.pose_base.pose.position.z;
      metrics_row_->obstacle_size_x = obstacle_res.size.x;
      metrics_row_->obstacle_size_y = obstacle_res.size.y;
      metrics_row_->obstacle_size_z = obstacle_res.size.z;
    }
    RCLCPP_INFO(
      get_logger(),
      "MoveTargetRl obstacle_box_center_base=(%.4f, %.4f, %.4f) obstacle_box_size=(%.4f, %.4f, %.4f) "
      "has_obstacle=%s source=%s",
      obstacle_res.pose_base.pose.position.x, obstacle_res.pose_base.pose.position.y,
      obstacle_res.pose_base.pose.position.z,
      obstacle_res.size.x, obstacle_res.size.y, obstacle_res.size.z,
      obstacle_res.has_obstacle ? "true" : "false", obstacle_res.source.c_str());

    publish_feedback(
      goal_handle, "wait_for_drl_services", 35.0f, target_res.pose_base, obstacle_res.pose_base);
    if (!wait_for_services(execute_motion, error_msg)) {
      abort_goal(goal_handle, result, "drl_plan", error_msg);
      return;
    }

    publish_feedback(
      goal_handle, "drl_plan", 45.0f, target_res.pose_base, obstacle_res.pose_base);
    std::string failed_stage;
    uint32_t trajectory_points = 0;
    const auto drl_plan_start = std::chrono::steady_clock::now();
    const bool planned = plan_with_drl(
      target_res.pose_base.pose.position,
      obstacle_res.has_obstacle,
      obstacle_res.pose_base.pose.position,
      obstacle_res.size,
      trajectory_points,
      failed_stage,
      error_msg);
    if (metrics_row_) {
      metrics_row_->drl_plan_time_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - drl_plan_start).count();
      metrics_row_->planning_success = planned;
    }
    result->trajectory_points = trajectory_points;
    if (metrics_row_) {
      metrics_row_->trajectory_points = trajectory_points;
      std::vector<geometry_msgs::msg::Point> waypoints;
      {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        waypoints.reserve(latest_trajectory_.poses.size());
        for (const auto & pose : latest_trajectory_.poses) {
          waypoints.push_back(pose.position);
        }
      }
      if (!waypoints.empty()) {
        robot_task_manager::AabbObstacle aabb;
        aabb.center = obstacle_res.pose_base.pose.position;
        aabb.size = obstacle_res.size;
        aabb.has_obstacle = obstacle_res.has_obstacle;
        const auto traj_metrics = robot_task_manager::computeTrajectoryMetrics(
          waypoints, start_pose.pose.position, target_res.pose_base.pose.position,
          obstacle_res.has_obstacle ? &aabb : nullptr, 0.0);
        metrics_row_->path_length_m = traj_metrics.path_length_m;
        metrics_row_->straight_line_distance_m = traj_metrics.straight_line_distance_m;
        metrics_row_->path_efficiency = traj_metrics.path_efficiency;
        metrics_row_->min_clearance_m = traj_metrics.min_clearance_m;
        metrics_row_->average_clearance_m = traj_metrics.average_clearance_m;
        metrics_row_->start_clearance_m = traj_metrics.start_clearance_m;
        metrics_row_->target_clearance_m = traj_metrics.target_clearance_m;
        metrics_row_->min_clearance_with_margin_m = traj_metrics.min_clearance_with_margin_m;
        metrics_row_->clearance_violation_count = traj_metrics.clearance_violation_count;
        metrics_row_->collision_or_inside_obstacle_count = traj_metrics.collision_or_inside_obstacle_count;
        metrics_row_->safety_margin_m = 0.0;
        metrics_row_->clearance_ok = traj_metrics.clearance_ok;
      }
    }
    if (!planned) {
      abort_goal(goal_handle, result, failed_stage, error_msg);
      return;
    }

    if (!execute_motion) {
      publish_feedback(
        goal_handle, "done_plan_only", 100.0f, target_res.pose_base, obstacle_res.pose_base);
      finish_success(
        goal_handle, result, "done_plan_only",
        "MoveTargetRl plan succeeded (source=" + target_res.source +
        "/" + obstacle_res.source + "); execution skipped because execute=false");
      return;
    }

    if (goal_handle->is_canceling() || cancel_requested_.load()) {
      request_cartesian_stop();
      abort_goal(
        goal_handle, result, "canceled",
        "DRL execution canceled before execute_forward by user Stop");
      return;
    }

    publish_feedback(
      goal_handle, "execute_forward", 70.0f, target_res.pose_base, obstacle_res.pose_base);
    std::string msg;
    if (!call_trigger(drl_execute_client_, "/drl/execute_forward", msg, 5.0)) {
      abort_goal(goal_handle, result, "execute_forward", "Start DRL execution failed: " + msg);
      return;
    }

    publish_feedback(
      goal_handle, "execution_status", 85.0f, target_res.pose_base, obstacle_res.pose_base);
    const auto drl_exec_wait_start = std::chrono::steady_clock::now();
    const bool exec_ok = wait_for_drl_execution(error_msg);
    if (metrics_row_) {
      metrics_row_->drl_execution_wait_time_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - drl_exec_wait_start).count();
      metrics_row_->execution_time_s = metrics_row_->drl_execution_wait_time_s;
      metrics_row_->execution_success = exec_ok;
    }
    if (!exec_ok) {
      abort_goal(goal_handle, result, "execution_status", error_msg);
      return;
    }
    if (!verify_final_pose(target_res.pose_base.pose.position, error_msg)) {
      abort_goal(goal_handle, result, "execution_status", error_msg);
      return;
    }

    publish_feedback(goal_handle, "done", 100.0f, target_res.pose_base, obstacle_res.pose_base);
    if (metrics_row_) {
      geometry_msgs::msg::PoseStamped final_pose;
      std::string final_error;
      if (current_pose(final_pose, final_error)) {
        metrics_row_->final_x = final_pose.pose.position.x;
        metrics_row_->final_y = final_pose.pose.position.y;
        metrics_row_->final_z = final_pose.pose.position.z;
        metrics_row_->final_position_error_m =
          position_error(final_pose.pose.position, target_res.pose_base.pose.position);
      }
    }
    finish_success(
      goal_handle, result, "done",
      "MoveTargetRl completed successfully (source=" + target_res.source +
      "/" + obstacle_res.source + ")");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveTargetRlActionServer>();
  node->initialize_logging();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
