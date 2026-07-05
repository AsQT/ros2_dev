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
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "robot_vision_pipeline_msgs/msg/box_array.hpp"

#include "robot_task_manager/action/drl_pick_place.hpp"
#include "robot_task_manager/action/move_gripper.hpp"
#include "robot_task_manager/action/move_to_pose_cartesian.hpp"
#include "robot_task_manager/action_metrics_logger.hpp"
#include "robot_task_manager/log_paths.hpp"
#include "robot_task_manager/rl_obstacle_input.hpp"
#include "robot_task_executor/executor_experiment_logger.hpp"

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
    pose_verify_attempts_ = declare_parameter<int>("pose_verify_attempts", 10);
    pose_verify_retry_delay_sec_ = declare_parameter<double>("pose_verify_retry_delay_sec", 0.2);
    gripper_open_width_m_ = declare_parameter<double>("gripper_open_width_m", 0.05);
    gripper_default_close_width_m_ = declare_parameter<double>("gripper_default_close_width_m", 0.028);
    pick_approach_height_m_ = declare_parameter<double>("pick_approach_height_m", 0.05);
    // Legacy launch compatibility only. PickPlaceRL must always plan
    // PLAN_TO_PRE_PICK from the current TCP directly to pre_pick/re_pick.
    legacy_use_preposition_before_pre_pick_param_ =
      declare_parameter<bool>("use_preposition_before_pre_pick", false);
    if (legacy_use_preposition_before_pre_pick_param_) {
      RCLCPP_WARN(
        get_logger(),
        "[PickPlaceRL] use_preposition_before_pre_pick=true is deprecated and ignored; "
        "skipping intermediate preposition before pre_pick");
    }
    cartesian_velocity_scale_ = declare_parameter<double>("cartesian_velocity_scale", 0.1);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 2.0);
    planner_node_name_ = declare_parameter<std::string>(
      "planner_node_name", "/drl_unified_planner_node");

    // Vision obstacle input for the DRL policy. PickPlaceRL now resolves a
    // best-effort vision box obstacle the SAME way MovePoseRL does (shared
    // helper robot_task_manager/rl_obstacle_input.hpp) instead of always feeding
    // obstacle=0, so both actions hand the shared policy a consistent 15D
    // observation. Best-effort: no fresh box => plan without an obstacle.
    enable_vision_obstacle_ = declare_parameter<bool>("enable_vision_obstacle", true);
    obstacle_class_ = declare_parameter<std::string>("obstacle_class", "box");
    box_objects_topic_ = declare_parameter<std::string>(
      "box_objects_topic", "/vision/box_objects");
    vision_timeout_sec_ = declare_parameter<double>("vision_timeout_sec", 1.0);

    enable_executor_logging_ = declare_parameter<bool>("enable_executor_logging", false);
    log_root_dir_            = declare_parameter<std::string>("log_root_dir", robot_task_manager::kDefaultLogRootDir);
    runtime_mode_            = declare_parameter<std::string>("runtime_mode", "mock");
    executor_log_dir_        = declare_parameter<std::string>(
      "executor_log_dir", robot_task_manager::executorLogBaseDir(log_root_dir_));
    executor_sample_rate_hz_ = declare_parameter<double>("executor_sample_rate_hz", 50.0);
    executor_base_frame_     = declare_parameter<std::string>("executor_base_frame", "base_link");
    executor_tcp_frame_      = declare_parameter<std::string>("executor_tcp_frame", "tcp_link");

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
    drl_planning_status_client_ = create_client<std_srvs::srv::Trigger>("/drl/get_planning_status");
    cartesian_stop_client_ = create_client<std_srvs::srv::Trigger>("/move_cartesian_stop");

    auto qos = rclcpp::QoS(1).reliable().transient_local();
    trajectory_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/drl/forward_trajectory_poses",
      qos,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        latest_trajectory_ = *msg;
        trajectory_seq_++;
      });
    observation_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/drl/last_plan_observation_15d",
      qos,
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
      robot_task_manager::actionMetricsLogDir(log_root_dir_, runtime_mode_, "DrlPickPlace"), get_logger());
    declare_parameter<bool>("use_mock", true);
    declare_parameter<std::string>("hardware_plugin", "unknown");
    declare_parameter<bool>("enable_log_plots", true);
    robot_task_manager::applyLogProvenanceFromParams(this, metrics_logger_);

    action_server_ = rclcpp_action::create_server<DrlPickPlace>(
      this,
      "drl_pickplace",
      std::bind(&DrlPickPlaceActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DrlPickPlaceActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&DrlPickPlaceActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "DrlPickPlace action server ready: /drl_pickplace");
  }

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
        robot_task_manager::executorActionLogDir(log_root_dir_, executor_log_dir_, runtime_mode_, "DrlPickPlace"),
        executor_sample_rate_hz_,
        executor_base_frame_, executor_tcp_frame_);
    } catch (const std::exception & e) {
      logger_.reset();
      RCLCPP_WARN(get_logger(), "DrlPickPlace CSV logger unavailable: %s", e.what());
    }
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
  int pose_verify_attempts_{10};
  double pose_verify_retry_delay_sec_{0.2};
  double gripper_open_width_m_{0.05};
  double gripper_default_close_width_m_{0.028};
  double pick_approach_height_m_{0.05};
  bool legacy_use_preposition_before_pre_pick_param_{false};
  double cartesian_velocity_scale_{0.1};
  double tf_timeout_sec_{2.0};

  bool enable_vision_obstacle_{true};
  std::string obstacle_class_;
  std::string box_objects_topic_;
  double vision_timeout_sec_{1.0};

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
  std::shared_ptr<robot_task_manager::ActionMetricsLogger> metrics_logger_;
  std::shared_ptr<robot_task_manager::ActionMetricsRow> metrics_row_;
  rclcpp::Time goal_start_time_;

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
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr drl_planning_status_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cartesian_stop_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr observation_sub_;
  rclcpp::Subscription<robot_vision_pipeline_msgs::msg::BoxArray>::SharedPtr box_sub_;

  std::mutex vision_mutex_;
  robot_vision_pipeline_msgs::msg::BoxArray latest_box_;
  rclcpp::Time latest_box_stamp_;
  bool have_box_{false};

  // codex.md §2: goal-level pick/place targets kept for the task-eval files.
  geometry_msgs::msg::Pose task_pick_target_;
  geometry_msgs::msg::Pose task_place_target_;
  bool have_task_targets_{false};
  bool task_execute_requested_{false};

  std::mutex active_goal_mutex_;
  MoveGripperGoalHandle::SharedPtr active_gripper_goal_;
  CartesianGoalHandle::SharedPtr active_cartesian_goal_;
  bool goal_active_{false};
  std::atomic<bool> cancel_requested_{false};

  std::mutex trajectory_mutex_;
  geometry_msgs::msg::PoseArray latest_trajectory_;
  uint64_t trajectory_seq_{0};

  std::mutex observation_mutex_;
  std::vector<double> latest_raw_observation_;
  std::vector<double> latest_model_observation_;
  uint64_t observation_seq_{0};

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const DrlPickPlace::Goal> goal)
  {
    const bool log_enabled = goal->enable_metrics_log;
    RCLCPP_INFO(
      get_logger(), "[drl_pickplace server] enable_metrics_log=%s",
      log_enabled ? "true" : "false");

    if (logger_ && log_enabled) {
      action_call_id_ = logger_->log_lifecycle_event(
        "/drl_pickplace", "action_goal_received", "handle_goal", "received", "");
    } else {
      action_call_id_ = 0;
    }
    if (!finite_pose(goal->target_pick.pose) || !finite_pose(goal->target_place.pose)) {
      RCLCPP_WARN(get_logger(), "Reject DrlPickPlace goal: non-finite pose");
      if (logger_ && log_enabled) {
        logger_->log_lifecycle_event(
          "/drl_pickplace", "action_goal_rejected", "handle_goal", "rejected",
          "non-finite pose", "", action_call_id_);
      }
      return rclcpp_action::GoalResponse::REJECT;
    }
    // Reject overlapping goals: a second goal accepted while the first is
    // still mid-sequence would race on the shared DRL planner state
    // (/drl/clear_trajectory, /drl/execute_forward), which is what produced
    // "Cannot clear trajectory while execution is running" in testing.
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    if (goal_active_) {
      RCLCPP_WARN(get_logger(), "Reject DrlPickPlace goal: another goal is already active");
      if (logger_ && log_enabled) {
        logger_->log_lifecycle_event(
          "/drl_pickplace", "action_goal_rejected", "handle_goal", "rejected",
          "another goal is already active", "", action_call_id_);
      }
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (logger_ && log_enabled) {
      logger_->log_lifecycle_event(
        "/drl_pickplace", "action_goal_accepted", "handle_goal", "accepted", "",
        "", action_call_id_);
    }
    cancel_requested_.store(false);
    goal_active_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_WARN(get_logger(), "DrlPickPlace cancel requested");
    cancel_requested_.store(true);
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    if (active_gripper_goal_) {
      move_gripper_client_->async_cancel_goal(active_gripper_goal_);
    }
    if (active_cartesian_goal_) {
      cartesian_client_->async_cancel_goal(active_cartesian_goal_);
    }
    if (cartesian_stop_client_->service_is_ready() ||
      cartesian_stop_client_->wait_for_service(100ms))
    {
      RCLCPP_WARN(get_logger(), "DrlPickPlace: calling /move_cartesian_stop to halt robot motion");
      cartesian_stop_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    } else {
      RCLCPP_WARN(get_logger(), "DrlPickPlace: /move_cartesian_stop not available; canceling action only");
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void request_cartesian_stop()
  {
    if (cartesian_stop_client_->service_is_ready() ||
      cartesian_stop_client_->wait_for_service(100ms))
    {
      RCLCPP_WARN(get_logger(), "DrlPickPlace: calling /move_cartesian_stop to halt robot motion");
      cartesian_stop_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    } else {
      RCLCPP_WARN(get_logger(), "DrlPickPlace: /move_cartesian_stop not available; canceling action only");
    }
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
      out.pose.orientation.x = 1.0;
      out.pose.orientation.y = 1.0;
      out.pose.orientation.z = 0.0;
      out.pose.orientation.w = 0.0;
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

  void finish_metrics(bool success, const std::string & stage, const std::string & message)
  {
    if (!metrics_row_) {
      return;
    }
    metrics_row_->success = success;
    metrics_row_->action_result_success = success;
    metrics_row_->planning_success = success;
    metrics_row_->execution_success = success;
    metrics_row_->failed_stage = success ? "" : stage;
    metrics_row_->message = message;
    metrics_row_->total_action_time_s = (now() - goal_start_time_).seconds();
    metrics_logger_->finish(metrics_row_);
    // After finish(): overwrite the generic markers with the real task-eval
    // files (phase_summary/object_tracking/task summary) for PickPlaceRL.
    write_task_eval_files(success, stage, message);
    metrics_row_.reset();
  }

  // codex.md §2: phase_summary.csv, object_tracking.csv and task-level
  // summary.csv for PickPlaceRL, written into the metrics call dir. Uses the
  // goal pick/place targets and the aggregate metrics row (no robot interaction).
  void write_task_eval_files(bool success, const std::string & stage, const std::string & message)
  {
    if (!metrics_row_ || metrics_row_->internal_call_dir.empty()) {
      return;
    }
    const std::filesystem::path dir(metrics_row_->internal_call_dir);
    auto rpy = [](const geometry_msgs::msg::Pose & p, int idx) {
      tf2::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
      if (q.length2() <= 1e-12) {return std::numeric_limits<double>::quiet_NaN();}
      q.normalize(); double r, pi, y; tf2::Matrix3x3(q).getRPY(r, pi, y);
      return idx == 0 ? r : (idx == 1 ? pi : y);
    };
    auto fmt = [](double v) {
      if (!std::isfinite(v)) {return std::string();}
      std::ostringstream o; o << std::fixed << std::setprecision(6) << v; return o.str();
    };
    const auto & row = *metrics_row_;

    // phase_summary.csv (pick, place). Per-phase timing is not separately
    // measured, so those cells are blank; targets/points/clearance are real.
    std::ofstream ps(dir / "phase_summary.csv", std::ios::out | std::ios::trunc);
    if (ps.is_open()) {
      ps << "phase,start_time_s,end_time_s,duration_s,success,failed_reason,"
            "start_tcp_x,start_tcp_y,start_tcp_z,start_tcp_roll,start_tcp_pitch,start_tcp_yaw,"
            "target_tcp_x,target_tcp_y,target_tcp_z,target_tcp_roll,target_tcp_pitch,target_tcp_yaw,"
            "final_tcp_x,final_tcp_y,final_tcp_z,final_tcp_roll,final_tcp_pitch,final_tcp_yaw,"
            "final_position_error_m,final_orientation_error_rad,path_length_m,planning_time_s,execution_time_s,"
            "rl_num_points,moveit_num_points,min_obstacle_clearance_m\n";
      const std::string ok = success ? "true" : "false";
      const std::string fr = success ? "" : stage;
      auto phase_row = [&](const std::string & name, const geometry_msgs::msg::Pose & tgt) {
        ps << name << ",,,," << ok << "," << fr
           << ",,,,,,"  // start_tcp (not sampled)
           << "," << fmt(tgt.position.x) << "," << fmt(tgt.position.y) << "," << fmt(tgt.position.z)
           << "," << fmt(rpy(tgt, 0)) << "," << fmt(rpy(tgt, 1)) << "," << fmt(rpy(tgt, 2))
           << ",,,,,,"  // final_tcp (not sampled)
           << ",,"      // final errors (not per-phase)
           << "," << fmt(row.path_length_m) << "," << fmt(row.drl_plan_time_s) << ","
           << "," << fmt(row.trajectory_points) << ",not_applicable," << fmt(row.min_clearance_m) << "\n";
      };
      phase_row("pick", task_pick_target_);
      phase_row("place", task_place_target_);
    }

    // object_tracking.csv — pick/place object poses from the goal (real).
    std::ofstream ot(dir / "object_tracking.csv", std::ios::out | std::ios::trunc);
    if (ot.is_open()) {
      ot << "data_available,empty_reason,t_s,phase,object_id,object_source,"
            "object_x,object_y,object_z,object_roll,object_pitch,object_yaw,"
            "object_confidence,object_attached,object_dropped\n";
      ot << "true,,0,pick,unknown,goal_pose," << fmt(task_pick_target_.position.x) << ","
         << fmt(task_pick_target_.position.y) << "," << fmt(task_pick_target_.position.z) << ",,,,,,\n";
      ot << "true,," << fmt(row.total_action_time_s) << ",place,unknown,goal_pose,"
         << fmt(task_place_target_.position.x) << "," << fmt(task_place_target_.position.y) << ","
         << fmt(task_place_target_.position.z) << ",,,,,,\n";
    }

    // task-level summary.csv (overwrites the row-summary with the task schema).
    std::ofstream ts(dir / "summary.csv", std::ios::out | std::ios::trunc);
    if (ts.is_open()) {
      ts << "action_name,hardware_mode,evaluation_group,run_id,call_id,goal_uuid,task_success,failed_phase,"
            "failed_stage,failure_reason,message,execute_requested,planning_only,"
            "total_time_s,total_planning_time_s,total_execution_time_s,pick_time_s,place_time_s,"
            "pick_success,place_success,pick_position_error_m,place_position_error_m,"
            "pick_orientation_error_rad,place_orientation_error_rad,"
            "total_tcp_path_length_m,max_tcp_error_m,rmse_tcp_position_m,rmse_tcp_orientation_rad,path_efficiency,"
            "object_source,object_id,object_dropped,total_rl_points,total_moveit_points,"
            "min_obstacle_clearance_m,collision_detected,workspace_violation\n";
      const std::string collided = std::isfinite(row.collision_or_inside_obstacle_count) ?
        (row.collision_or_inside_obstacle_count > 0 ? "true" : "false") : "";
      ts << "pick_place_rl," << robot_task_manager::hardwareModeFromPath(dir.string())
         << ",03_task_execution_eval," << row.run_id << "," << row.action_call_id << ","
         << row.goal_uuid << "," << (success ? "true" : "false") << ","
         << (success ? "" : stage) << "," << (success ? "" : stage) << ","
         << (success ? "" : stage) << "," << message << ","
         << (task_execute_requested_ ? "true" : "false") << ","
         << (task_execute_requested_ ? "false" : "true") << ","
         << fmt(row.total_action_time_s) << "," << fmt(row.drl_plan_time_s) << ","
         << fmt(row.execution_time_s) << ",,,"
         << (success ? "true" : "") << "," << (success ? "true" : "") << ",,,,,"
         << fmt(row.path_length_m) << ",," << fmt(row.final_position_error_m) << ","
         << fmt(row.final_orientation_error_rad) << "," << fmt(row.path_efficiency) << ","
         << "goal_pose,unknown,," << fmt(row.trajectory_points) << ",not_applicable,"
         << fmt(row.min_clearance_m) << "," << collided << ",\n";
    }
  }

  bool check_cancel(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::shared_ptr<DrlPickPlace::Result> & result,
    const std::string & stage)
  {
    if (!goal_handle->is_canceling() && !cancel_requested_.load()) {
      return false;
    }
    result->success = false;
    result->message = "DrlPickPlace canceled";
    result->failed_stage = stage;
    if (logger_ && action_call_id_ != 0) {
      logger_->log_lifecycle_event(
        "/drl_pickplace", "action_canceled", stage, "canceled", result->message,
        "", action_call_id_);
      logger_->log_lifecycle_event(
        "/drl_pickplace", "action_result", stage, "canceled", result->message,
        "", action_call_id_);
    }
    clear_active_goals();
    release_goal_slot();
    finish_metrics(false, "canceled", result->message);
    goal_handle->canceled(result);
    return true;
  }

  void clear_active_goals()
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    active_gripper_goal_.reset();
    active_cartesian_goal_.reset();
  }

  void release_goal_slot()
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    goal_active_ = false;
  }

  void abort_goal(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::shared_ptr<DrlPickPlace::Result> & result,
    const std::string & stage,
    const std::string & message)
  {
    clear_active_goals();
    release_goal_slot();
    result->success = false;
    if (goal_handle->is_canceling() || cancel_requested_.load()) {
      result->message = "DrlPickPlace canceled during " + stage + ": " + message;
      result->failed_stage = "canceled";
      RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
      if (logger_ && action_call_id_ != 0) {
        logger_->log_lifecycle_event(
          "/drl_pickplace", "action_canceled", stage, "canceled", result->message,
          "", action_call_id_);
        logger_->log_lifecycle_event(
          "/drl_pickplace", "action_result", stage, "canceled", result->message,
          "", action_call_id_);
      }
      finish_metrics(false, "canceled", result->message);
      goal_handle->canceled(result);
      return;
    }

    result->message = message;
    result->failed_stage = stage;
    RCLCPP_ERROR(get_logger(), "DrlPickPlace failed at %s: %s", stage.c_str(), message.c_str());
    if (logger_ && action_call_id_ != 0) {
      logger_->log_lifecycle_event(
        "/drl_pickplace", "action_stage_failed", stage, "failed", message, "", action_call_id_);
      logger_->log_lifecycle_event(
        "/drl_pickplace", "action_result", stage, "aborted", message, "", action_call_id_);
    }
    finish_metrics(false, stage, message);
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
        {"/drl/get_planning_status", drl_planning_status_client_},
      })
    {
      if (!item.second->wait_for_service(std::chrono::seconds(5))) {
        error_msg = "DRL service not available: " + item.first;
        return false;
      }
    }
    return true;
  }

  bool call_move_gripper(double width_m, bool execute, std::string & error_msg)
  {
    MoveGripper::Goal goal;
    goal.position = width_m;
    goal.execute = execute;
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
    bool execute,
    std::string & error_msg)
  {
    MoveToPoseCartesian::Goal goal;
    goal.target_pose = target;
    goal.velocity_scale = cartesian_velocity_scale_;
    goal.execute = execute;
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

  bool transform_point_to_planning_frame(
    const geometry_msgs::msg::Point & point_in,
    const std::string & source_frame,
    geometry_msgs::msg::Point & point_out)
  {
    if (source_frame.empty() || source_frame == planning_frame_) {
      point_out = point_in;
      return true;
    }
    geometry_msgs::msg::PointStamped stamped_in;
    stamped_in.header.frame_id = source_frame;
    stamped_in.point = point_in;
    try {
      const auto stamped_out = tf_buffer_.transform(
        stamped_in, planning_frame_, tf2::durationFromSec(tf_timeout_sec_));
      point_out = stamped_out.point;
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }

  // Resolves the best-effort vision box obstacle for the DRL policy, exactly like
  // MovePoseRL (shared helper robot_task_manager/rl_obstacle_input.hpp). No fresh
  // box => has_obstacle=false (plan without obstacle).
  robot_task_manager::RlObstacleInput resolve_obstacle_input(
    const geometry_msgs::msg::Point & current_tcp_base,
    const geometry_msgs::msg::Point & target_base)
  {
    robot_task_manager::RlObstacleInput none;
    if (!enable_vision_obstacle_) {
      return none;
    }
    robot_vision_pipeline_msgs::msg::BoxArray box_snapshot;
    rclcpp::Time box_stamp;
    bool have_box = false;
    {
      std::lock_guard<std::mutex> lock(vision_mutex_);
      box_snapshot = latest_box_;
      box_stamp = latest_box_stamp_;
      have_box = have_box_;
    }
    const bool fresh = have_box && (now() - box_stamp).seconds() <= vision_timeout_sec_;
    if (!fresh) {
      return none;
    }
    return robot_task_manager::resolveRlObstacleInput(
      box_snapshot, obstacle_class_, current_tcp_base, target_base,
      [this](
        const geometry_msgs::msg::Point & in, const std::string & frame_in,
        geometry_msgs::msg::Point & out) {
        return transform_point_to_planning_frame(in, frame_in, out);
      });
  }

  bool set_drl_target(
    const geometry_msgs::msg::Pose & target,
    bool preposition_before_plan,
    const robot_task_manager::RlObstacleInput & obstacle,
    const geometry_msgs::msg::Point * start_override_base,
    std::string & error_msg)
  {
    auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    req->parameters.push_back(make_double_array_param(
      "manual_default_target",
      {target.position.x, target.position.y, target.position.z}));
    req->parameters.push_back(make_bool_param("preposition_before_plan", preposition_before_plan));
    req->parameters.push_back(make_bool_param("update_start_tcp_from_tf_before_plan", start_override_base == nullptr));
    req->parameters.push_back(make_bool_param("use_manual_start_tcp_before_plan", start_override_base != nullptr));
    req->parameters.push_back(make_double_array_param(
      "manual_start_tcp_base",
      start_override_base ?
      std::vector<double>{
        start_override_base->x,
        start_override_base->y,
        start_override_base->z} :
      std::vector<double>{
        target.position.x,
        target.position.y,
        target.position.z}));
    req->parameters.push_back(make_bool_param("auto_execute_after_plan", false));
    // Obstacle input, aligned with MovePoseRL (shared helper
    // rl_obstacle_input.hpp). PickPlaceRL feeds its OWN freshly-resolved vision
    // box each call so the shared DRL policy gets a consistent 15D observation
    // (obs 9..14) instead of a degenerate all-zero obstacle — see
    // Reports/pickplace_rl_obstacle_input_alignment_report.md.
    //
    // Setting these params EXPLICITLY on every call still prevents the original
    // cross-contamination bug (inheriting a stale MovePoseRL manual box on the
    // shared planner): we always overwrite with PickPlaceRL's own value. When no
    // fresh box is visible, has_obstacle=false => center/size stay 0 and
    // allow_skip=true, i.e. plan without an obstacle — identical to MovePoseRL's
    // no-box case, and never a leftover box.
    req->parameters.push_back(make_double_array_param(
      "manual_default_obstacle_center",
      {obstacle.center_base.x, obstacle.center_base.y, obstacle.center_base.z}));
    req->parameters.push_back(make_double_array_param(
      "manual_default_obstacle_size",
      {obstacle.size.x, obstacle.size.y, obstacle.size.z}));
    req->parameters.push_back(make_bool_param("manual_allow_skip_obstacle", !obstacle.has_obstacle));

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
    // codex.md Part 2: make the obstacle policy for this pick/place target
    // explicit in the log so a future "Target lies inside obstacle" is traceable.
    RCLCPP_INFO(
      get_logger(),
      "[DrlPickPlace] target=(%.4f %.4f %.4f) obstacle_source=PLANNING_SCENE_ONLY "
      "manual_default_obstacle=cleared(center/size=0) manual_allow_skip_obstacle=true "
      "(MovePoseRL manual obstacle NOT applied to PickPlaceRL)",
      target.position.x, target.position.y, target.position.z);
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
      if (cancel_requested_.load()) {
        request_cartesian_stop();
        error_msg = "DrlPickPlace canceled by user Stop";
        return false;
      }
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

      // Poll /drl/get_planning_status so a failed /drl/plan is detected
      // within ~1s instead of only after drl_timeout_sec_, since a failed
      // plan never publishes /drl/forward_trajectory_poses.
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
        request_cartesian_stop();
        error_msg = "DrlPickPlace canceled by user Stop";
        return false;
      }
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

  void log_plan_to_pre_pick_debug(
    const geometry_msgs::msg::PoseStamped & current_tcp,
    const geometry_msgs::msg::Pose & wood_raw_base,
    const geometry_msgs::msg::Pose & pre_pick_target)
  {
    const auto rpy_deg = rpy_deg_from_quat(pre_pick_target.orientation);
    const double dx = pre_pick_target.position.x - wood_raw_base.position.x;
    const double dy = pre_pick_target.position.y - wood_raw_base.position.y;
    const double dz = pre_pick_target.position.z - wood_raw_base.position.z;

    RCLCPP_INFO(
      get_logger(),
      "[PickPlaceRL][PLAN_TO_PRE_PICK] current_tcp=(%.4f, %.4f, %.4f)",
      current_tcp.pose.position.x, current_tcp.pose.position.y, current_tcp.pose.position.z);
    RCLCPP_INFO(
      get_logger(),
      "[PickPlaceRL][PLAN_TO_PRE_PICK] wood_raw_base=(%.4f, %.4f, %.4f)",
      wood_raw_base.position.x, wood_raw_base.position.y, wood_raw_base.position.z);
    RCLCPP_INFO(
      get_logger(),
      "[PickPlaceRL][PLAN_TO_PRE_PICK] pre_pick_target=(%.4f, %.4f, %.4f)",
      pre_pick_target.position.x, pre_pick_target.position.y, pre_pick_target.position.z);
    RCLCPP_INFO(
      get_logger(),
      "[PickPlaceRL][PLAN_TO_PRE_PICK] target_rpy_deg=(%.2f, %.2f, %.2f)",
      rpy_deg[0], rpy_deg[1], rpy_deg[2]);
    RCLCPP_INFO(
      get_logger(),
      "[PickPlaceRL][PLAN_TO_PRE_PICK] target_quat=(%.6f, %.6f, %.6f, %.6f)",
      pre_pick_target.orientation.x, pre_pick_target.orientation.y,
      pre_pick_target.orientation.z, pre_pick_target.orientation.w);
    RCLCPP_INFO(
      get_logger(),
      "[PickPlaceRL][PLAN_TO_PRE_PICK] vision_target_offset=(%.4f, %.4f, %.4f)",
      dx, dy, dz);
    RCLCPP_INFO(
      get_logger(),
      "[PickPlaceRL][PLAN_TO_PRE_PICK] obstacle=resolved_per_plan "
      "(see '[PickPlaceRL] obstacle_input ...' log; vision box via shared "
      "resolveRlObstacleInput, same as MovePoseRL)");
    RCLCPP_INFO(
      get_logger(),
      "[PickPlaceRL][PLAN_TO_PRE_PICK] planner_service=/drl/plan "
      "set_parameters=%s/set_parameters status=/drl/get_planning_status "
      "preposition_before_plan=false update_start_tcp_from_tf_before_plan=true",
      planner_node_name_.c_str());
  }

  bool call_drl_plan_and_execute(
    const geometry_msgs::msg::Pose & target,
    const std::string & phase,
    bool preposition_before_plan,
    bool execute,
    const geometry_msgs::msg::Point * start_override_base,
    std::string & error_msg)
  {
    const int attempts = std::max(1, drl_plan_attempts_);
    std::string last_error;
    for (int attempt = 1; attempt <= attempts; ++attempt) {
      if (cancel_requested_.load()) {
        request_cartesian_stop();
        error_msg = "DrlPickPlace canceled by user Stop";
        return false;
      }
      RCLCPP_INFO(
        get_logger(),
        "DRL plan attempt %d/%d mode=%s target=(%.4f %.4f %.4f) preposition=%s",
        attempt,
        attempts,
        execute ? "execute" : "plan-only",
        target.position.x,
        target.position.y,
        target.position.z,
        preposition_before_plan ? "true" : "false");

      // Resolve PickPlaceRL's own vision obstacle (same logic as MovePoseRL) and
      // feed it to the planner so obs 9..14 are consistent between the actions.
      const geometry_msgs::msg::Point current_tcp_point =
        start_override_base ? *start_override_base : current_pose().pose.position;
      const robot_task_manager::RlObstacleInput obstacle =
        resolve_obstacle_input(current_tcp_point, target.position);
      RCLCPP_INFO(
        get_logger(),
        "[PickPlaceRL] phase=%s obstacle_input source=%s has_obstacle=%s "
        "center=(%.4f %.4f %.4f) size=(%.4f %.4f %.4f)",
        phase.c_str(), obstacle.source.c_str(), obstacle.has_obstacle ? "true" : "false",
        obstacle.center_base.x, obstacle.center_base.y, obstacle.center_base.z,
        obstacle.size.x, obstacle.size.y, obstacle.size.z);

      if (!set_drl_target(target, preposition_before_plan, obstacle, start_override_base, last_error)) {
        error_msg = last_error;
        return false;
      }
      // Dump the intended 15D input BEFORE invoking the planner so
      // rl_input_15d_{pick,place}.csv is never empty even if the planner rejects
      // early (e.g. start outside trained workspace) and never publishes an
      // observation. Overwritten by the authoritative planner observation below
      // when one is published.
      write_pre_plan_rl_input(phase, target, obstacle, start_override_base);
      if (cancel_requested_.load()) {
        request_cartesian_stop();
        error_msg = "DrlPickPlace canceled by user Stop";
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

      bool planner_called = false;
      bool planner_invoked = false;
      bool wrote_phase_logs = false;
      std::string msg;
      if (!call_trigger(drl_clear_client_, "/drl/clear_trajectory", msg, 5.0)) {
        last_error = "Clear DRL trajectory failed: " + msg;
      } else if ((planner_invoked = true) &&
        !(planner_called = call_trigger(drl_plan_client_, "/drl/plan", msg, 5.0)))
      {
        // /drl/plan was invoked but returned failure (e.g. "DRL rollout did not
        // reach target"). The planner still publishes the 15D observation on
        // failure (drl_unified_planner_node._publish_last_plan_observation), so we
        // must still write the phase logs below — otherwise rl_input_15d_pick.csv
        // and planning_pick.csv stay empty exactly when they are needed to diagnose
        // WHY PLAN_TO_PRE_PICK failed. See
        // Reports/pickplace_rl_compare_moveposerl_plan_to_prepick_report.md.
        last_error = "Start DRL planning failed: " + msg;
      } else if (!wait_for_planned_trajectory(seq_before, target, last_error)) {
        // last_error is already populated.
      } else if (cancel_requested_.load()) {
        request_cartesian_stop();
        last_error = "DrlPickPlace canceled by user Stop";
      } else if (!execute) {
        write_phase_plan_logs(phase, target, observation_seq_before, seq_before);
        return true;
      } else if (cancel_requested_.load()) {
        write_phase_plan_logs(phase, target, observation_seq_before, seq_before);
        wrote_phase_logs = true;
        request_cartesian_stop();
        last_error = "DrlPickPlace canceled before DRL execute_forward by user Stop";
      } else {
        std::vector<geometry_msgs::msg::Pose> planned_poses;
        {
          std::lock_guard<std::mutex> lock(trajectory_mutex_);
          if (trajectory_seq_ > seq_before) {
            planned_poses = latest_trajectory_.poses;
          }
        }
        robot_task_manager::AabbObstacle exec_aabb;
        exec_aabb.center = obstacle.center_base;
        exec_aabb.size = obstacle.size;
        exec_aabb.has_obstacle = obstacle.has_obstacle;
        auto tcp_sampler = metrics_logger_->startTcpExecutionSampling(
          metrics_row_, phase, target, planned_poses, {},
          obstacle.has_obstacle ? &exec_aabb : nullptr,
          [this](geometry_msgs::msg::PoseStamped & out, std::string &) {
            out = current_pose();
            return finite_pose(out.pose);
          },
          executor_sample_rate_hz_);
        if (!call_trigger(drl_execute_client_, "/drl/execute_forward", msg, 5.0)) {
          metrics_logger_->stopTcpExecutionSampling(tcp_sampler);
          write_phase_plan_logs(phase, target, observation_seq_before, seq_before);
          wrote_phase_logs = true;
          last_error = "Start DRL execution failed: " + msg;
        } else if (!wait_for_drl_execution(last_error)) {
          metrics_logger_->stopTcpExecutionSampling(tcp_sampler);
          write_phase_plan_logs(phase, target, observation_seq_before, seq_before);
          wrote_phase_logs = true;
          // last_error is already populated.
        } else {
          metrics_logger_->stopTcpExecutionSampling(tcp_sampler);
          write_phase_plan_logs(phase, target, observation_seq_before, seq_before);
          return true;
        }
      }

      if (planner_invoked && !wrote_phase_logs) {
        // Capture logs whenever /drl/plan was invoked, even if it returned
        // failure — the observation (and any partial/failed path) is published on
        // failure too, so rl_input_15d_pick.csv must not be left empty.
        write_phase_plan_logs(phase, target, observation_seq_before, seq_before);
      }

      RCLCPP_WARN(
        get_logger(),
        "DRL plan attempt %d/%d failed: %s",
        attempt,
        attempts,
        last_error.c_str());
      if (cancel_requested_.load()) {
        request_cartesian_stop();
        error_msg = last_error.empty() ? "DrlPickPlace canceled by user Stop" : last_error;
        return false;
      }
      if (attempt < attempts) {
        std::this_thread::sleep_for(500ms);
      }
    }
    error_msg = last_error.empty() ? "DRL planning failed" : last_error;
    return false;
  }

  // Pre-plan RL input dump. Guarantees rl_input_15d_{pick,place}.csv is never
  // empty when the planner is invoked (section 5 of codex.md), and records the
  // EXACT 15D input the server intends the policy to use — most importantly the
  // obstacle fields, which PickPlaceRL clears to zero (set_drl_target) whereas
  // MovePoseRL feeds a real vision box. That difference is the key thing to
  // compare against MovePoseRL's rl_input_15d.csv. The authoritative planner
  // observation (write_phase_plan_logs) overwrites this same file whenever the
  // planner actually published one; if it did not (e.g. it threw before rollout,
  // "outside trained workspace"), this pre-plan snapshot survives.
  void write_pre_plan_rl_input(
    const std::string & phase,
    const geometry_msgs::msg::Pose & target,
    const robot_task_manager::RlObstacleInput & obstacle,
    const geometry_msgs::msg::Point * start_override_base)
  {
    if (!metrics_row_) {
      return;
    }
    const auto actual_tcp = current_pose().pose;
    geometry_msgs::msg::Pose tcp = actual_tcp;
    if (start_override_base) {
      tcp.position = *start_override_base;
    }
    std::vector<double> raw(15, 0.0);
    raw[0] = tcp.position.x;
    raw[1] = tcp.position.y;
    raw[2] = tcp.position.z;
    raw[3] = target.position.x;
    raw[4] = target.position.y;
    raw[5] = target.position.z;
    raw[6] = target.position.x - tcp.position.x;
    raw[7] = target.position.y - tcp.position.y;
    raw[8] = target.position.z - tcp.position.z;
    // Indices 9..14: RAW obstacle input the server intends to feed (relative to
    // target + full size). 0 when no fresh vision box. NOTE: the authoritative
    // planner observation (write_phase_plan_logs) uses NORMALIZED values and
    // overwrites this file when published; this snapshot is the pre-plan fallback.
    if (obstacle.has_obstacle) {
      raw[9] = obstacle.center_base.x - target.position.x;
      raw[10] = obstacle.center_base.y - target.position.y;
      raw[11] = obstacle.center_base.z - target.position.z;
      raw[12] = obstacle.size.x;
      raw[13] = obstacle.size.y;
      raw[14] = obstacle.size.z;
    }
    const bool is_place = phase == "place";
    const std::string source = obstacle.has_obstacle ?
      "drl_pickplace_server:pre_plan_intended_input(obstacle_vision_raw)" :
      "drl_pickplace_server:pre_plan_intended_input(no_obstacle)";
    metrics_logger_->writeRlInput15d(
      *metrics_row_,
      is_place ? "rl_observation_place.csv" : "rl_observation_pick.csv",
      phase,
      raw,
      raw,
      source);
  }

  void write_phase_plan_logs(
    const std::string & phase,
    const geometry_msgs::msg::Pose & target,
    uint64_t observation_seq_before,
    uint64_t trajectory_seq_before)
  {
    if (!metrics_row_) {
      return;
    }
    std::vector<geometry_msgs::msg::Pose> planned_poses;
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      if (trajectory_seq_ > trajectory_seq_before) {
        planned_poses = latest_trajectory_.poses;
      }
    }
    const bool is_place = phase == "place";
    if (planned_poses.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "DRL %s planner produced no fresh accepted trajectory for this attempt",
        phase.c_str());
    }
    metrics_logger_->writePlanningTrajectory(
      *metrics_row_,
      is_place ? "planning_place.csv" : "planning_pick.csv",
      phase,
      planned_poses,
      target.position,
      nullptr,
      true);

    std::vector<double> raw_observation;
    std::vector<double> model_observation;
    if (copy_observation_after(observation_seq_before, raw_observation, model_observation)) {
      metrics_logger_->writeRlInput15d(
        *metrics_row_,
        is_place ? "rl_observation_place.csv" : "rl_observation_pick.csv",
        phase,
        raw_observation,
        model_observation,
        "drl_unified_planner_node:first_raw_observation");
      const std::string stage_tag =
        is_place ? "[PickPlaceRL][PLAN_TO_PLACE]" : "[PickPlaceRL][PLAN_TO_PRE_PICK]";
      RCLCPP_INFO(
        get_logger(),
        "%s rl_observation_15d=%s",
        stage_tag.c_str(),
        format_values(raw_observation).c_str());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "DRL %s trajectory was ready but no fresh /drl/last_plan_observation_15d arrived",
        phase.c_str());
    }
  }

  void log_sequence_step(
    int index,
    const std::string & step_name,
    const std::string & target_frame,
    const geometry_msgs::msg::Pose & pose,
    const std::string & planner_type,
    bool execute) const
  {
    RCLCPP_INFO(
      get_logger(),
      "[PickPlaceRL sequence] %d. step_name=%s target_frame=%s "
      "x=%.4f y=%.4f z=%.4f q=(%.6f %.6f %.6f %.6f) planner_type=%s execute=%s",
      index,
      step_name.c_str(),
      target_frame.c_str(),
      pose.position.x,
      pose.position.y,
      pose.position.z,
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w,
      planner_type.c_str(),
      execute ? "true" : "false");
  }

  void log_pickplace_sequence(
    const geometry_msgs::msg::Pose & current_tcp,
    const geometry_msgs::msg::Pose & pre_pick,
    const geometry_msgs::msg::Pose & pick,
    const geometry_msgs::msg::Pose & lift_pose,
    const geometry_msgs::msg::Pose & place,
    bool execute) const
  {
    RCLCPP_INFO(get_logger(), "PickPlaceRL sequence:");
    log_sequence_step(0, "current_tcp_start", planning_frame_, current_tcp, "TF/current", execute);
    log_sequence_step(1, "current -> pre_pick/re_pick", planning_frame_, pre_pick, "RL", execute);
    log_sequence_step(2, "pre_pick/re_pick -> pick", planning_frame_, pick, "cartesian", execute);
    RCLCPP_INFO(
      get_logger(),
      "[PickPlaceRL sequence] 3. step_name=gripper_close target_frame=n/a "
      "x=nan y=nan z=nan q=(nan nan nan nan) planner_type=gripper execute=%s",
      execute ? "true" : "false");
    log_sequence_step(4, "pick -> retract", planning_frame_, lift_pose, "cartesian", execute);
    log_sequence_step(5, "move to place/pre_place", planning_frame_, place, "RL", execute);
    RCLCPP_INFO(
      get_logger(),
      "[PickPlaceRL sequence] 6. step_name=place/open_gripper target_frame=%s "
      "x=%.4f y=%.4f z=%.4f q=(%.6f %.6f %.6f %.6f) planner_type=gripper execute=%s",
      planning_frame_.c_str(),
      place.position.x,
      place.position.y,
      place.position.z,
      place.orientation.x,
      place.orientation.y,
      place.orientation.z,
      place.orientation.w,
      execute ? "true" : "false");
  }

  // check_orientation must be false for poses reached via call_drl_plan_and_execute():
  // the DRL/cartesian-pose-sequence executor always executes with its own fixed
  // tool orientation (see task_executor_node's "Cartesian orientation FIXED" quat),
  // not the caller-provided target.orientation, so comparing against the goal
  // orientation there would always fail by a constant offset. Poses reached via
  // call_cartesian() (the /move_to_pose_cartesian action, MoveItExecutor-backed)
  // do honor the requested orientation and should keep checking it.
  bool verify_pose(
    const std::string & label,
    const geometry_msgs::msg::Pose & target,
    std::string & error_msg,
    bool check_orientation = true)
  {
    const int attempts = std::max(1, pose_verify_attempts_);
    const auto retry_delay = std::chrono::duration<double>(
      std::max(0.0, pose_verify_retry_delay_sec_));

    double best_pos_err = 1e9;
    double best_ori_err = 1e9;
    geometry_msgs::msg::PoseStamped best_actual;

    for (int attempt = 1; attempt <= attempts; ++attempt) {
      const auto actual = current_pose();
      const double pos_err = position_error(actual.pose, target);
      const double ori_err = orientation_error_rad(actual.pose.orientation, target.orientation);
      if (pos_err < best_pos_err) {
        best_pos_err = pos_err;
        best_ori_err = ori_err;
        best_actual = actual;
      }

      RCLCPP_INFO(
        get_logger(),
        "%s pose check %d/%d | requested=(%.4f %.4f %.4f) actual=(%.4f %.4f %.4f) pos_err=%.5f ori_err=%.5f%s",
        label.c_str(),
        attempt,
        attempts,
        target.position.x,
        target.position.y,
        target.position.z,
        actual.pose.position.x,
        actual.pose.position.y,
        actual.pose.position.z,
        pos_err,
        ori_err,
        check_orientation ? "" : " (orientation not enforced for DRL-executed pose)");

      if (pos_err <= position_tolerance_m_ &&
        (!check_orientation || ori_err <= orientation_tolerance_rad_))
      {
        return true;
      }

      if (attempt < attempts) {
        std::this_thread::sleep_for(retry_delay);
      }
    }

    if (best_pos_err > position_tolerance_m_) {
      error_msg = label + " position tolerance failed: " + std::to_string(best_pos_err);
      RCLCPP_ERROR(
        get_logger(),
        "%s best pose after retries | actual=(%.4f %.4f %.4f) best_pos_err=%.5f best_ori_err=%.5f",
        label.c_str(),
        best_actual.pose.position.x,
        best_actual.pose.position.y,
        best_actual.pose.position.z,
        best_pos_err,
        best_ori_err);
      return false;
    }
    error_msg = label + " orientation tolerance failed: " + std::to_string(best_ori_err);
    return false;
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
    const bool execute_motion = goal->execute;
    goal_start_time_ = now();
    task_pick_target_ = goal->target_pick.pose;
    task_place_target_ = goal->target_place.pose;
    have_task_targets_ = true;
    task_execute_requested_ = execute_motion;
    metrics_row_.reset();
    if (goal->enable_metrics_log) {
      metrics_row_ = metrics_logger_->startCall(
        "DrlPickPlace", "rl_pickplace", robot_task_manager::goalUuidHex(goal_handle->get_goal_id()));
      if (metrics_row_) {
        metrics_row_->execute_requested = execute_motion;
        metrics_row_->start_x = goal->target_pick.pose.position.x;
        metrics_row_->start_y = goal->target_pick.pose.position.y;
        metrics_row_->start_z = goal->target_pick.pose.position.z;
        metrics_row_->target_x = goal->target_place.pose.position.x;
        metrics_row_->target_y = goal->target_place.pose.position.y;
        metrics_row_->target_z = goal->target_place.pose.position.z;
      }
    }

    if (logger_ && action_call_id_ != 0) {
      logger_->log_lifecycle_event(
        "/drl_pickplace", "action_start", "execute", "started", "", "", action_call_id_);
    }

    publish_feedback(goal_handle, execute_motion ? "VALIDATE_GOAL" : "VALIDATE_GOAL_PLAN_ONLY", 1.0f);
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

    RCLCPP_INFO(
      get_logger(),
      "[Z_DEBUG] planning_frame=%s input_pick_frame=%s target_pick_z=%.4f "
      "target_place_z=%.4f pick_approach_height=%.4f close_width=%.4f",
      planning_frame_.c_str(),
      goal->target_pick.header.frame_id.c_str(),
      target_pick.pose.position.z,
      target_place.pose.position.z,
      pick_approach_height_m_,
      close_width_m);
    RCLCPP_INFO(
      get_logger(),
      "execution_velocity_scale=%.3f",
      cartesian_velocity_scale_);

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
    RCLCPP_INFO(
      get_logger(),
      "[Z_DEBUG] pick_sequence_z pre_pick=%.4f pick=%.4f lift=%.4f "
      "pre_pick_minus_pick=%.4f",
      pre_pick.position.z,
      target_pick.pose.position.z,
      lift_pose.position.z,
      pre_pick.position.z - target_pick.pose.position.z);

    publish_feedback(goal_handle, execute_motion ? "OPEN_GRIPPER" : "PLAN_OPEN_GRIPPER_EXECUTION_SKIPPED", 8.0f);
    if (!call_move_gripper(gripper_open_width_m_, execute_motion, error_msg)) {
      abort_goal(goal_handle, result, "OPEN_GRIPPER", error_msg);
      return;
    }
    if (check_cancel(goal_handle, result, "OPEN_GRIPPER")) {
      return;
    }

    publish_feedback(goal_handle, execute_motion ? "PLAN_TO_PRE_PICK" : "PLAN_TO_PRE_PICK_EXECUTION_SKIPPED", 22.0f);
    const auto current_tcp = current_pose();
    log_plan_to_pre_pick_debug(current_tcp, target_pick.pose, pre_pick);
    log_pickplace_sequence(
      current_tcp.pose, pre_pick, target_pick.pose, lift_pose, target_place.pose, execute_motion);
    RCLCPP_INFO(
      get_logger(),
      "[PickPlaceRL] skip intermediate waypoint before pre_pick");
    RCLCPP_INFO(
      get_logger(),
      "[PickPlaceRL] MovePoseRL current -> pre_pick/re_pick");
    RCLCPP_INFO(
      get_logger(),
      "[PickPlaceRL] PLAN_TO_PRE_PICK preposition_before_plan=false "
      "preposition_tcp_base_skipped=true rl_start_pose_source=current_tcp");
    RCLCPP_INFO(
      get_logger(),
      "[PickPlaceRL] stage=PLAN_TO_PRE_PICK target=pre_pick "
      "current_tcp_before_rl_plan=(%.4f, %.4f, %.4f) pre_pick=(%.4f, %.4f, %.4f) "
      "pick=(%.4f, %.4f, %.4f)",
      current_tcp.pose.position.x, current_tcp.pose.position.y, current_tcp.pose.position.z,
      pre_pick.position.x, pre_pick.position.y, pre_pick.position.z,
      target_pick.pose.position.x, target_pick.pose.position.y, target_pick.pose.position.z);
    RCLCPP_INFO(
      get_logger(),
      "[PickPlaceRL] PLAN_TO_PRE_PICK attempt=1 preposition_before_plan=false start=current_tcp");
    bool pre_pick_ok =
      call_drl_plan_and_execute(
        pre_pick, "pick", /*preposition_before_plan=*/false, execute_motion, nullptr, error_msg) &&
      (!execute_motion || verify_pose("PLAN_TO_PRE_PICK", pre_pick, error_msg, false));

    if (!pre_pick_ok && !cancel_requested_.load()) {
      RCLCPP_WARN(
        get_logger(),
        "[PickPlaceRL] PLAN_TO_PRE_PICK failed without intermediate retry: %s",
        error_msg.c_str());
    }

    if (!pre_pick_ok) {
      abort_goal(goal_handle, result, "PLAN_TO_PRE_PICK", error_msg);
      return;
    }
    if (check_cancel(goal_handle, result, "PLAN_TO_PRE_PICK")) {
      return;
    }

    publish_feedback(goal_handle, execute_motion ? "DESCEND_TO_PICK" : "PLAN_DESCEND_TO_PICK_EXECUTION_SKIPPED", 38.0f);
    if (execute_motion) {
      if (!call_cartesian(target_pick.pose, execute_motion, error_msg) ||
          !verify_pose("DESCEND_TO_PICK", target_pick.pose, error_msg))
      {
        abort_goal(goal_handle, result, "DESCEND_TO_PICK", error_msg);
        return;
      }
    } else {
      RCLCPP_INFO(
        get_logger(),
        "[PickPlaceRL] plan-only: virtual DESCEND_TO_PICK from pre_pick to pick");
    }
    if (check_cancel(goal_handle, result, "DESCEND_TO_PICK")) {
      return;
    }

    publish_feedback(goal_handle, execute_motion ? "CLOSE_GRIPPER" : "PLAN_CLOSE_GRIPPER_EXECUTION_SKIPPED", 50.0f);
    if (!call_move_gripper(close_width_m, execute_motion, error_msg)) {
      abort_goal(goal_handle, result, "CLOSE_GRIPPER", error_msg);
      return;
    }
    if (check_cancel(goal_handle, result, "CLOSE_GRIPPER")) {
      return;
    }

    publish_feedback(goal_handle, execute_motion ? "LIFT_FROM_PICK" : "PLAN_LIFT_FROM_PICK_EXECUTION_SKIPPED", 62.0f);
    if (execute_motion) {
      if (!call_cartesian(lift_pose, execute_motion, error_msg) ||
          !verify_pose("LIFT_FROM_PICK", lift_pose, error_msg))
      {
        abort_goal(goal_handle, result, "LIFT_FROM_PICK", error_msg);
        return;
      }
    } else {
      RCLCPP_INFO(
        get_logger(),
        "[PickPlaceRL] plan-only: virtual LIFT_FROM_PICK from pick to lift_pose");
    }
    if (check_cancel(goal_handle, result, "LIFT_FROM_PICK")) {
      return;
    }

    publish_feedback(goal_handle, execute_motion ? "PLAN_TO_PLACE" : "PLAN_TO_PLACE_EXECUTION_SKIPPED", 82.0f);
    const geometry_msgs::msg::Point * place_plan_start_override =
      execute_motion ? nullptr : &lift_pose.position;
    if (!call_drl_plan_and_execute(
        target_place.pose, "place", false, execute_motion, place_plan_start_override, error_msg) ||
        (execute_motion && !verify_pose("PLAN_TO_PLACE", target_place.pose, error_msg, false)))
    {
      abort_goal(goal_handle, result, "PLAN_TO_PLACE", error_msg);
      return;
    }
    if (check_cancel(goal_handle, result, "PLAN_TO_PLACE")) {
      return;
    }

    publish_feedback(goal_handle, execute_motion ? "OPEN_GRIPPER_AT_PLACE" : "PLAN_OPEN_GRIPPER_AT_PLACE_EXECUTION_SKIPPED", 96.0f);
    if (!call_move_gripper(gripper_open_width_m_, execute_motion, error_msg)) {
      abort_goal(goal_handle, result, "OPEN_GRIPPER_AT_PLACE", error_msg);
      return;
    }
    if (check_cancel(goal_handle, result, "OPEN_GRIPPER_AT_PLACE")) {
      return;
    }

    publish_feedback(goal_handle, execute_motion ? "DONE" : "DONE_PLANNING_EXECUTION_SKIPPED", 100.0f);
    result->success = true;
    result->message = execute_motion ?
      "DrlPickPlace completed successfully" :
      "DrlPickPlace planning success; execution skipped";
    result->failed_stage = "";
    if (logger_ && action_call_id_ != 0) {
      logger_->log_lifecycle_event(
        "/drl_pickplace", "action_succeeded", "DONE", "succeeded", result->message,
        "", action_call_id_);
      logger_->log_lifecycle_event(
        "/drl_pickplace", "action_result", "DONE", "succeeded", result->message,
        "", action_call_id_);
    }
    clear_active_goals();
    release_goal_slot();
    finish_metrics(true, "DONE", result->message);
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "DrlPickPlace completed successfully");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DrlPickPlaceActionServer>();
  node->initialize_logging();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
