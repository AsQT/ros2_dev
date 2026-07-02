#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "robot_task_manager/action/move_to_pose_obstacle.hpp"
#include "robot_task_manager/action_metrics_logger.hpp"
#include "robot_task_manager/log_paths.hpp"
#include "robot_task_manager/moveit_executor.hpp"
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

bool finite_point(const geometry_msgs::msg::Point & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

bool finite_vector3(const geometry_msgs::msg::Vector3 & v)
{
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// Every axis must be a real, strictly positive full-size extent — matches
// step 6 of codex.md Part B section 5 ("Validate obstacle size: size.x > 0,
// size.y > 0, size.z > 0"). No default substitution happens here: unlike
// MoveTargetRl, this action has no default_box_size fallback parameter, so
// an invalid vision size is treated as "no usable obstacle", not silently
// replaced by a constant.
bool positive_size(const geometry_msgs::msg::Vector3 & v)
{
  return finite_vector3(v) && v.x > 1e-6 && v.y > 1e-6 && v.z > 1e-6;
}

// Shortest distance from `point` to the segment [seg_a, seg_b] — used to
// pick the box detection that best represents the obstacle actually
// standing between the current TCP and the target (codex.md section 6:
// "chọn box gần đoạn thẳng current TCP -> target nhất", same rule as
// MoveTargetRl's obstacle selection).
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

class MoveToPoseObstacleActionServer : public rclcpp::Node
{
public:
  using MoveToPoseObstacle = robot_task_manager::action::MoveToPoseObstacle;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MoveToPoseObstacle>;

  MoveToPoseObstacleActionServer()
  : Node("move_to_pose_obstacle_action_server"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    planning_group_ = declare_parameter<std::string>("planning_group", "arm");
    base_frame_ = declare_parameter<std::string>("base_frame", "world");
    obstacle_frame_ = declare_parameter<std::string>("obstacle_frame", "base_link");
    ee_link_ = declare_parameter<std::string>("ee_link", "tcp_link");
    obstacle_class_ = declare_parameter<std::string>("obstacle_class", "box");
    vision_timeout_sec_ = declare_parameter<double>("vision_timeout_sec", 1.0);
    box_objects_topic_ = declare_parameter<std::string>(
      "box_objects_topic", "/vision/box_objects");
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 2.0);
    collision_object_id_ = declare_parameter<std::string>(
      "collision_object_id", "move_to_pose_obstacle_box");

    enable_executor_logging_ = declare_parameter<bool>("enable_executor_logging", false);
    log_root_dir_            = declare_parameter<std::string>("log_root_dir", robot_task_manager::kDefaultLogRootDir);
    executor_log_dir_        = declare_parameter<std::string>(
      "executor_log_dir", robot_task_manager::executorLogBaseDir(log_root_dir_));
    executor_sample_rate_hz_ = declare_parameter<double>("executor_sample_rate_hz", 50.0);
    executor_base_frame_     = declare_parameter<std::string>("executor_base_frame", "base_link");
    executor_tcp_frame_      = declare_parameter<std::string>("executor_tcp_frame", "tcp_link");

    // Vision detections are published BEST_EFFORT/VOLATILE by
    // pixel_to_base_mapper_node (robot_vision_pipeline config
    // pixel_to_base_mapper.yaml) — match QoS so the subscription is
    // compatible, same as move_target_rl_server.
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
      robot_task_manager::actionMetricsLogDir(log_root_dir_, "MoveToPoseObstacle"), get_logger());

    action_server_ = rclcpp_action::create_server<MoveToPoseObstacle>(
      this,
      "move_to_pose_obstacle",
      std::bind(&MoveToPoseObstacleActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveToPoseObstacleActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MoveToPoseObstacleActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "MoveToPoseObstacle action server ready: /move_to_pose_obstacle | obstacle_class=%s "
      "box_topic=%s obstacle_frame=%s",
      obstacle_class_.c_str(), box_objects_topic_.c_str(), obstacle_frame_.c_str());
  }

  void initialize_moveit()
  {
    executor_ = std::make_shared<robot_task_manager::MoveItExecutor>();
    executor_->initialize(shared_from_this(), planning_group_, base_frame_);
    executor_->initializeLogging(
      enable_executor_logging_,
      robot_task_manager::executorActionLogDir(log_root_dir_, executor_log_dir_, "MoveToPoseObstacle"),
      executor_sample_rate_hz_,
      executor_base_frame_,
      executor_tcp_frame_,
      "/move_to_pose_obstacle");
  }

private:
  std::string planning_group_;
  std::string base_frame_;
  std::string obstacle_frame_;
  std::string ee_link_;
  std::string obstacle_class_;
  double vision_timeout_sec_{1.0};
  std::string box_objects_topic_;
  double tf_timeout_sec_{2.0};
  std::string collision_object_id_;

  bool enable_executor_logging_{false};
  std::string log_root_dir_;
  std::string executor_log_dir_;
  double executor_sample_rate_hz_{50.0};
  std::string executor_base_frame_;
  std::string executor_tcp_frame_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::shared_ptr<robot_task_manager::MoveItExecutor> executor_;
  std::shared_ptr<robot_task_manager::ActionMetricsLogger> metrics_logger_;
  rclcpp_action::Server<MoveToPoseObstacle>::SharedPtr action_server_;
  rclcpp::Subscription<robot_vision_pipeline_msgs::msg::BoxArray>::SharedPtr box_sub_;

  std::mutex vision_mutex_;
  robot_vision_pipeline_msgs::msg::BoxArray latest_box_;
  rclcpp::Time latest_box_stamp_;
  bool have_box_{false};

  std::mutex goal_active_mutex_;
  bool goal_active_{false};

  // Per-call metrics state. Safe as plain members (not locals threaded
  // through every helper) because goal_active_ already guarantees only one
  // execute() runs at a time — a second goal is rejected in handle_goal()
  // while one is in flight, and this member is only ever touched from the
  // single detached execute() thread that currently owns the goal slot.
  std::shared_ptr<robot_task_manager::ActionMetricsRow> metrics_row_;
  rclcpp::Time goal_start_time_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveToPoseObstacle::Goal> goal)
  {
    if (executor_ && executor_->getLogger()) {
      executor_->getLogger()->log_lifecycle_event(
        "/move_to_pose_obstacle", "action_goal_received", "handle_goal", "received", "");
    }
    if (!std::isfinite(goal->velocity_scale) ||
      goal->velocity_scale <= 0.0 || goal->velocity_scale > 1.0)
    {
      RCLCPP_WARN(get_logger(), "Reject goal because velocity_scale must be in (0,1]");
      if (executor_ && executor_->getLogger()) {
        executor_->getLogger()->log_lifecycle_event(
          "/move_to_pose_obstacle", "action_goal_rejected", "handle_goal", "rejected",
          "velocity_scale must be in (0,1]");
      }
      return rclcpp_action::GoalResponse::REJECT;
    }
    // Reject overlapping goals: a second goal accepted while the first still
    // owns the shared collision object / MoveItExecutor motion mutex would
    // race with it — same guard pattern as MoveTargetRl.
    std::lock_guard<std::mutex> lock(goal_active_mutex_);
    if (goal_active_) {
      RCLCPP_WARN(get_logger(), "Reject MoveToPoseObstacle goal: another goal is already active");
      if (executor_ && executor_->getLogger()) {
        executor_->getLogger()->log_lifecycle_event(
          "/move_to_pose_obstacle", "action_goal_rejected", "handle_goal", "rejected",
          "another goal is already active");
      }
      return rclcpp_action::GoalResponse::REJECT;
    }
    goal_active_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_WARN(get_logger(), "MoveToPoseObstacle cancel requested");
    if (executor_) {
      executor_->stop();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void release_goal_slot()
  {
    std::lock_guard<std::mutex> lock(goal_active_mutex_);
    goal_active_ = false;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread(&MoveToPoseObstacleActionServer::execute, this, goal_handle).detach();
  }

  bool current_tcp_in_obstacle_frame(geometry_msgs::msg::PoseStamped & out, std::string & error_msg)
  {
    out.header.stamp = now();
    out.header.frame_id = obstacle_frame_;
    try {
      const auto tf = tf_buffer_.lookupTransform(
        obstacle_frame_, ee_link_, tf2::TimePointZero, tf2::durationFromSec(tf_timeout_sec_));
      out.pose.position.x = tf.transform.translation.x;
      out.pose.position.y = tf.transform.translation.y;
      out.pose.position.z = tf.transform.translation.z;
      out.pose.orientation = tf.transform.rotation;
      return true;
    } catch (const std::exception & e) {
      error_msg = "Failed to get current TCP pose from TF " + obstacle_frame_ +
        " <- " + ee_link_ + ": " + e.what();
      return false;
    }
  }

  // target_pose in the goal is interpreted in MoveIt's planning frame
  // (base_frame_, "world" by default — unchanged /move_to_pose convention).
  // For obstacle-selection purposes only (choosing which detected box is
  // closest to the TCP->target segment) we also need it expressed in
  // obstacle_frame_ ("base_link"); this does NOT change what gets sent to
  // MoveItExecutor::moveToPose(), which still plans against the original
  // goal->target_pose exactly like /move_to_pose does.
  bool target_in_obstacle_frame(
    const geometry_msgs::msg::Pose & target_pose_planning_frame,
    geometry_msgs::msg::Point & out,
    std::string & error_msg)
  {
    if (base_frame_ == obstacle_frame_) {
      out = target_pose_planning_frame.position;
      return true;
    }
    geometry_msgs::msg::PointStamped stamped_in;
    stamped_in.header.frame_id = base_frame_;
    stamped_in.header.stamp = builtin_interfaces::msg::Time();
    stamped_in.point = target_pose_planning_frame.position;
    try {
      const auto stamped_out = tf_buffer_.transform(
        stamped_in, obstacle_frame_, tf2::durationFromSec(tf_timeout_sec_));
      out = stamped_out.point;
      return true;
    } catch (const std::exception & e) {
      error_msg = "TF transform " + base_frame_ + " -> " + obstacle_frame_ +
        " failed: " + e.what();
      return false;
    }
  }

  bool transform_point_to_obstacle_frame(
    const geometry_msgs::msg::Point & point_in,
    const std::string & source_frame,
    geometry_msgs::msg::Point & point_out,
    std::string & error_msg)
  {
    if (source_frame.empty() || source_frame == obstacle_frame_) {
      point_out = point_in;
      return true;
    }
    geometry_msgs::msg::PointStamped stamped_in;
    stamped_in.header.frame_id = source_frame;
    stamped_in.header.stamp = builtin_interfaces::msg::Time();
    stamped_in.point = point_in;
    try {
      const auto stamped_out = tf_buffer_.transform(
        stamped_in, obstacle_frame_, tf2::durationFromSec(tf_timeout_sec_));
      point_out = stamped_out.point;
      return true;
    } catch (const std::exception & e) {
      error_msg = "TF transform " + source_frame + " -> " + obstacle_frame_ +
        " failed: " + e.what();
      return false;
    }
  }

  // codex2.md section 9.3: the collision object's primitive_pose must be the
  // *transformed box.pose*, not just its transformed position — a box that
  // is rotated in the vision frame (e.g. aruco_world) must keep that
  // rotation in the planning frame (e.g. base_link), otherwise MoveIt would
  // add an axis-aligned box that no longer matches the real footprint.
  bool transform_pose_to_obstacle_frame(
    const geometry_msgs::msg::Pose & pose_in,
    const std::string & source_frame,
    geometry_msgs::msg::Pose & pose_out,
    std::string & error_msg)
  {
    if (source_frame.empty() || source_frame == obstacle_frame_) {
      pose_out = pose_in;
      return true;
    }
    geometry_msgs::msg::PoseStamped stamped_in;
    stamped_in.header.frame_id = source_frame;
    stamped_in.header.stamp = builtin_interfaces::msg::Time();
    stamped_in.pose = pose_in;
    try {
      const auto stamped_out = tf_buffer_.transform(
        stamped_in, obstacle_frame_, tf2::durationFromSec(tf_timeout_sec_));
      pose_out = stamped_out.pose;
      return true;
    } catch (const std::exception & e) {
      error_msg = "TF transform " + source_frame + " -> " + obstacle_frame_ +
        " failed: " + e.what();
      return false;
    }
  }

  struct ObstacleResolution
  {
    bool ok = false;
    bool has_obstacle = false;
    std::string source;  // "vision", "fallback", or "none"
    std::string selected_rule;  // e.g. "closest_to_path", only set when source=="vision"
    std::string frame_in;  // source frame of the vision detection used, e.g. "aruco_world"
    geometry_msgs::msg::PoseStamped pose_obstacle_frame;
    geometry_msgs::msg::Vector3 size;
    std::string failed_stage;
    std::string error_msg;
  };

  ObstacleResolution resolve_obstacle(
    const MoveToPoseObstacle::Goal & goal,
    const geometry_msgs::msg::Point & current_tcp_obstacle_frame,
    const geometry_msgs::msg::Point & target_obstacle_frame)
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
      result.pose_obstacle_frame.header.frame_id = obstacle_frame_;
      result.pose_obstacle_frame.header.stamp = now();
      result.pose_obstacle_frame.pose.position = goal.fallback_obstacle_center;
      result.pose_obstacle_frame.pose.orientation.w = 1.0;
      result.size = goal.fallback_obstacle_size;
      result.source = "fallback";
      result.has_obstacle = true;
      result.ok = true;
      return result;
    }

    if (!goal.use_vision_obstacle) {
      if (goal.require_obstacle) {
        result.failed_stage = "vision_obstacle_3d_unavailable";
        result.error_msg = "use_vision_obstacle=false and use_fallback_obstacle=false, "
          "but require_obstacle=true — no obstacle source selected";
        return result;
      }
      result.ok = true;
      result.has_obstacle = false;
      result.source = "none";
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
      RCLCPP_INFO(
        get_logger(), "[move_to_pose_obstacle] no box obstacle from %s", box_objects_topic_.c_str());
      result.ok = true;
      result.has_obstacle = false;
      result.source = "none";
      return result;
    }

    struct Candidate
    {
      geometry_msgs::msg::Pose pose_obstacle_frame;
      geometry_msgs::msg::Vector3 size;
    };
    std::vector<Candidate> candidates;
    std::string transform_error;
    bool any_transform_failed = false;
    for (const auto & box : box_snapshot.boxes) {
      if (box.class_name != obstacle_class) {
        continue;
      }
      if (!finite_pose(box.pose) || !positive_size(box.size)) {
        // codex.md section 6: keep Box.size.x/y/z as-is, do not substitute a
        // default when it is valid — but a detection with an invalid size
        // (<=0 on any axis) cannot be used as a collision object at all.
        continue;
      }
      geometry_msgs::msg::Pose pose_obstacle_frame;
      if (!transform_pose_to_obstacle_frame(
          box.pose, box_snapshot.header.frame_id, pose_obstacle_frame, transform_error))
      {
        any_transform_failed = true;
        continue;
      }
      Candidate c;
      c.pose_obstacle_frame = pose_obstacle_frame;
      c.size = box.size;
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
            "', finite pose, and a valid positive size on " + box_objects_topic_;
        }
        return result;
      }
      RCLCPP_INFO(
        get_logger(), "[move_to_pose_obstacle] no box obstacle from %s", box_objects_topic_.c_str());
      result.ok = true;
      result.has_obstacle = false;
      result.source = "none";
      return result;
    }

    // codex.md section 6 / codex2.md section 9.1: prefer the box closest to
    // the current-TCP -> target segment (same rule as MoveTargetRl's
    // obstacle selection). Recorded as selected_rule="closest_to_path" below.
    const Candidate * chosen = &candidates.front();
    if (candidates.size() > 1) {
      double best_dist = point_to_segment_distance(
        candidates.front().pose_obstacle_frame.position,
        current_tcp_obstacle_frame, target_obstacle_frame);
      for (size_t i = 1; i < candidates.size(); ++i) {
        const double d = point_to_segment_distance(
          candidates[i].pose_obstacle_frame.position,
          current_tcp_obstacle_frame, target_obstacle_frame);
        if (d < best_dist) {
          best_dist = d;
          chosen = &candidates[i];
        }
      }
    }

    result.pose_obstacle_frame.header.frame_id = obstacle_frame_;
    result.pose_obstacle_frame.header.stamp = now();
    result.pose_obstacle_frame.pose = chosen->pose_obstacle_frame;
    result.size = chosen->size;
    result.source = "vision";
    result.selected_rule = "closest_to_path";
    result.frame_in = box_snapshot.header.frame_id;
    result.has_obstacle = true;
    result.ok = true;

    // codex2.md section 9.4: log the exact fields used to build the
    // collision object so a run can be audited from the console alone.
    RCLCPP_INFO(
      get_logger(),
      "[move_to_pose_obstacle] using vision obstacle: source=%s frame_in=%s "
      "frame_planning=%s center=(%.4f, %.4f, %.4f) size=(%.4f, %.4f, %.4f) "
      "selected_rule=%s",
      box_objects_topic_.c_str(), result.frame_in.c_str(), obstacle_frame_.c_str(),
      result.pose_obstacle_frame.pose.position.x,
      result.pose_obstacle_frame.pose.position.y,
      result.pose_obstacle_frame.pose.position.z,
      result.size.x, result.size.y, result.size.z,
      result.selected_rule.c_str());
    return result;
  }

  moveit_msgs::msg::CollisionObject build_collision_object(
    const geometry_msgs::msg::PoseStamped & pose_obstacle_frame,
    const geometry_msgs::msg::Vector3 & size,
    int32_t operation)
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = obstacle_frame_;
    obj.id = collision_object_id_;
    obj.operation = operation;
    if (operation == moveit_msgs::msg::CollisionObject::ADD) {
      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
      primitive.dimensions = {size.x, size.y, size.z};
      obj.primitives.push_back(primitive);
      obj.primitive_poses.push_back(pose_obstacle_frame.pose);
    }
    return obj;
  }

  void publish_feedback(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::string & stage,
    float progress,
    const geometry_msgs::msg::PoseStamped & obstacle_pose)
  {
    auto feedback = std::make_shared<MoveToPoseObstacle::Feedback>();
    feedback->current_stage = stage;
    feedback->progress = progress;
    geometry_msgs::msg::PoseStamped current_pose;
    std::string pose_error;
    current_tcp_in_obstacle_frame(current_pose, pose_error);
    feedback->current_pose = current_pose;
    feedback->obstacle_pose = obstacle_pose;
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(get_logger(), "[MoveToPoseObstacle] %s | %.1f%%", stage.c_str(), progress);
  }

  // Single exit point for both success and failure: always removes the
  // collision object we may have added (codex.md section 7 — this action
  // treats the obstacle as scoped to a single call, since it exists purely
  // to benchmark one plan against one detected/fallback box; keeping stale
  // geometry in the shared PlanningScene between unrelated calls would be
  // worse than re-adding it fresh next time). See report section 8 for the
  // full justification.
  void finish(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::shared_ptr<MoveToPoseObstacle::Result> & result,
    bool obstacle_added,
    bool success,
    const std::string & stage,
    const std::string & message)
  {
    if (obstacle_added) {
      std::string remove_error;
      const bool removed = executor_->removeCollisionObjects({collision_object_id_}, remove_error);
      if (!removed) {
        RCLCPP_WARN(
          get_logger(), "MoveToPoseObstacle: failed to remove collision object %s: %s",
          collision_object_id_.c_str(), remove_error.c_str());
      }
      if (metrics_row_) {
        metrics_row_->collision_object_removed = removed;
      }
    }

    // Single chokepoint for the opt-in per-goal metrics CSV: every exit
    // path (7 failure sites + 1 success site) already funnels through
    // finish(), so this is the only place that needs to call
    // metrics_logger_->finish() to satisfy "log even on failure".
    const bool canceled = goal_handle->is_canceling();
    const bool effective_success = success && !canceled;
    const std::string effective_stage = canceled ? "canceled" : stage;
    const std::string effective_message = canceled ? "MoveToPoseObstacle canceled" : message;
    if (metrics_row_) {
      metrics_row_->success = effective_success;
      metrics_row_->action_result_success = effective_success;
      metrics_row_->failed_stage = effective_success ? "" : effective_stage;
      metrics_row_->message = effective_message;
      metrics_row_->total_action_time_s = (now() - goal_start_time_).seconds();
      metrics_logger_->finish(metrics_row_);
      metrics_row_.reset();
    }

    release_goal_slot();
    if (canceled) {
      result->success = false;
      result->message = effective_message;
      result->failed_stage = effective_stage;
      if (executor_ && executor_->getLogger()) {
        executor_->getLogger()->log_lifecycle_event(
          "/move_to_pose_obstacle", "action_result", effective_stage, "canceled", effective_message);
      }
      goal_handle->canceled(result);
    } else if (success) {
      result->success = true;
      result->message = message;
      result->failed_stage = "";
      if (executor_ && executor_->getLogger()) {
        executor_->getLogger()->log_lifecycle_event(
          "/move_to_pose_obstacle", "action_result", stage, "succeeded", message);
      }
      goal_handle->succeed(result);
    } else {
      result->success = false;
      result->message = message;
      result->failed_stage = stage;
      RCLCPP_ERROR(get_logger(), "MoveToPoseObstacle failed at %s: %s", stage.c_str(), message.c_str());
      if (executor_ && executor_->getLogger()) {
        executor_->getLogger()->log_lifecycle_event(
          "/move_to_pose_obstacle", "action_stage_failed", stage, "failed", message);
        executor_->getLogger()->log_lifecycle_event(
          "/move_to_pose_obstacle", "action_result", stage, "aborted", message);
      }
      goal_handle->abort(result);
    }
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto result = std::make_shared<MoveToPoseObstacle::Result>();
    const auto goal = goal_handle->get_goal();
    goal_start_time_ = now();

    RCLCPP_INFO(
      get_logger(), "[move_to_pose_obstacle server] enable_metrics_log=%s",
      goal->enable_metrics_log ? "true" : "false");

    metrics_row_.reset();
    if (goal->enable_metrics_log) {
      metrics_row_ = metrics_logger_->startCall(
        "MoveToPoseObstacle", "moveit_baseline",
        robot_task_manager::goalUuidHex(goal_handle->get_goal_id()));
      if (metrics_row_) {
        metrics_row_->execute_requested = goal->execute;
        metrics_row_->obstacle_class = goal->obstacle_class;
        metrics_row_->obstacle_frame = obstacle_frame_;
        metrics_row_->planning_scene_object_id = collision_object_id_;
      }
    }

    geometry_msgs::msg::PoseStamped empty_pose;
    empty_pose.header.frame_id = obstacle_frame_;

    if (executor_ && executor_->getLogger()) {
      executor_->getLogger()->log_lifecycle_event(
        "/move_to_pose_obstacle", "action_start", "execute", "started", "");
    }

    publish_feedback(goal_handle, "validate_goal", 5.0f, empty_pose);
    if (!finite_pose(goal->target_pose)) {
      finish(goal_handle, result, false, false, "validate_goal", "target_pose contains non-finite values");
      return;
    }

    publish_feedback(goal_handle, "get_current_pose", 10.0f, empty_pose);
    geometry_msgs::msg::PoseStamped current_tcp;
    std::string error_msg;
    if (!current_tcp_in_obstacle_frame(current_tcp, error_msg)) {
      finish(goal_handle, result, false, false, "get_current_pose", error_msg);
      return;
    }
    RCLCPP_INFO(
      get_logger(), "MoveToPoseObstacle current_tcp_%s=(%.4f, %.4f, %.4f)",
      obstacle_frame_.c_str(), current_tcp.pose.position.x, current_tcp.pose.position.y,
      current_tcp.pose.position.z);
    if (metrics_row_) {
      metrics_row_->start_x = current_tcp.pose.position.x;
      metrics_row_->start_y = current_tcp.pose.position.y;
      metrics_row_->start_z = current_tcp.pose.position.z;
    }

    geometry_msgs::msg::PoseStamped target_planning_frame;
    target_planning_frame.header.frame_id = base_frame_;
    target_planning_frame.header.stamp = now();
    target_planning_frame.pose = goal->target_pose;
    result->target_pose = target_planning_frame;

    geometry_msgs::msg::Point target_obstacle_frame;
    if (!target_in_obstacle_frame(goal->target_pose, target_obstacle_frame, error_msg)) {
      finish(goal_handle, result, false, false, "tf_transform_failed", error_msg);
      return;
    }
    if (metrics_row_) {
      metrics_row_->target_x = target_obstacle_frame.x;
      metrics_row_->target_y = target_obstacle_frame.y;
      metrics_row_->target_z = target_obstacle_frame.z;
    }

    publish_feedback(goal_handle, "resolve_obstacle", 20.0f, empty_pose);
    const ObstacleResolution obstacle_res = resolve_obstacle(
      *goal, current_tcp.pose.position, target_obstacle_frame);
    if (!obstacle_res.ok) {
      finish(goal_handle, result, false, false, obstacle_res.failed_stage, obstacle_res.error_msg);
      return;
    }
    if (goal->require_obstacle && !obstacle_res.has_obstacle) {
      finish(
        goal_handle, result, false, false, "vision_obstacle_3d_unavailable",
        "require_obstacle=true but no obstacle could be resolved");
      return;
    }
    result->obstacle_pose = obstacle_res.pose_obstacle_frame;
    result->obstacle_size = obstacle_res.size;
    if (metrics_row_) {
      metrics_row_->has_obstacle = obstacle_res.has_obstacle;
      metrics_row_->obstacle_source = obstacle_res.source;
      metrics_row_->obstacle_center_x = obstacle_res.pose_obstacle_frame.pose.position.x;
      metrics_row_->obstacle_center_y = obstacle_res.pose_obstacle_frame.pose.position.y;
      metrics_row_->obstacle_center_z = obstacle_res.pose_obstacle_frame.pose.position.z;
      metrics_row_->obstacle_size_x = obstacle_res.size.x;
      metrics_row_->obstacle_size_y = obstacle_res.size.y;
      metrics_row_->obstacle_size_z = obstacle_res.size.z;
    }
    RCLCPP_INFO(
      get_logger(),
      "MoveToPoseObstacle obstacle_center_%s=(%.4f, %.4f, %.4f) obstacle_size=(%.4f, %.4f, %.4f) "
      "has_obstacle=%s source=%s",
      obstacle_frame_.c_str(),
      obstacle_res.pose_obstacle_frame.pose.position.x,
      obstacle_res.pose_obstacle_frame.pose.position.y,
      obstacle_res.pose_obstacle_frame.pose.position.z,
      obstacle_res.size.x, obstacle_res.size.y, obstacle_res.size.z,
      obstacle_res.has_obstacle ? "true" : "false", obstacle_res.source.c_str());

    bool obstacle_added = false;
    if (obstacle_res.has_obstacle) {
      publish_feedback(goal_handle, "add_collision_object", 30.0f, obstacle_res.pose_obstacle_frame);
      const auto collision_object = build_collision_object(
        obstacle_res.pose_obstacle_frame, obstacle_res.size, moveit_msgs::msg::CollisionObject::ADD);
      if (!executor_->applyCollisionObjects({collision_object}, error_msg)) {
        finish(goal_handle, result, false, false, "add_collision_object", error_msg);
        return;
      }
      obstacle_added = true;
      if (metrics_row_) {
        metrics_row_->collision_object_added = true;
        metrics_row_->obstacle_added_to_planning_scene = true;
      }
      RCLCPP_INFO(
        get_logger(), "MoveToPoseObstacle collision object '%s' added to PlanningScene (frame=%s)",
        collision_object_id_.c_str(), obstacle_frame_.c_str());
    } else if (metrics_row_) {
      metrics_row_->collision_object_added = false;
      metrics_row_->obstacle_added_to_planning_scene = false;
    }

    publish_feedback(
      goal_handle, goal->execute ? "planning_and_execution" : "planning", 50.0f,
      obstacle_res.pose_obstacle_frame);
    robot_task_manager::MoveItPlanMetrics plan_metrics;
    const bool ok = executor_->moveToPose(
      goal->target_pose, error_msg, goal->velocity_scale, 0.3, 5.0, goal->execute,
      metrics_row_ ? &plan_metrics : nullptr);

    if (metrics_row_) {
      metrics_row_->planning_success = plan_metrics.has_plan;
      metrics_row_->planning_time_s = plan_metrics.planning_time_s;
      metrics_row_->moveit_plan_time_s = plan_metrics.planning_time_s;
      metrics_row_->moveit_planning_group = plan_metrics.planning_group;
      metrics_row_->moveit_planner_id = plan_metrics.planner_id;
      metrics_row_->moveit_allowed_planning_time_s = plan_metrics.allowed_planning_time_s;
      metrics_row_->moveit_error_code = plan_metrics.moveit_error_code;
      if (goal->execute && plan_metrics.has_plan) {
        metrics_row_->execution_success = ok;
        metrics_row_->execution_time_s = plan_metrics.execution_time_s;
        metrics_row_->moveit_execute_time_s = plan_metrics.execution_time_s;
      }
      if (plan_metrics.has_plan && !plan_metrics.tcp_waypoints.empty()) {
        std::vector<geometry_msgs::msg::Point> waypoints_obstacle_frame;
        waypoints_obstacle_frame.reserve(plan_metrics.tcp_waypoints.size());
        std::string transform_error;
        for (const auto & wp_base : plan_metrics.tcp_waypoints) {
          geometry_msgs::msg::Point wp_obstacle;
          if (transform_point_to_obstacle_frame(wp_base, base_frame_, wp_obstacle, transform_error)) {
            waypoints_obstacle_frame.push_back(wp_obstacle);
          }
        }
        metrics_row_->trajectory_points = static_cast<double>(waypoints_obstacle_frame.size());
        robot_task_manager::AabbObstacle aabb;
        aabb.center = obstacle_res.pose_obstacle_frame.pose.position;
        aabb.size = obstacle_res.size;
        aabb.has_obstacle = obstacle_res.has_obstacle;
        const auto traj_metrics = robot_task_manager::computeTrajectoryMetrics(
          waypoints_obstacle_frame, current_tcp.pose.position, target_obstacle_frame,
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

    if (!ok) {
      finish(goal_handle, result, obstacle_added, false, "planning_and_execution", error_msg);
      return;
    }

    publish_feedback(goal_handle, "done", 100.0f, obstacle_res.pose_obstacle_frame);
    if (metrics_row_) {
      geometry_msgs::msg::PoseStamped final_tcp;
      std::string final_error;
      if (current_tcp_in_obstacle_frame(final_tcp, final_error)) {
        metrics_row_->final_x = final_tcp.pose.position.x;
        metrics_row_->final_y = final_tcp.pose.position.y;
        metrics_row_->final_z = final_tcp.pose.position.z;
        const double dx = final_tcp.pose.position.x - target_obstacle_frame.x;
        const double dy = final_tcp.pose.position.y - target_obstacle_frame.y;
        const double dz = final_tcp.pose.position.z - target_obstacle_frame.z;
        metrics_row_->final_position_error_m = std::sqrt(dx * dx + dy * dy + dz * dz);
      }
    }
    const std::string message = goal->execute ?
      "MoveToPoseObstacle reached target pose successfully (obstacle_source=" + obstacle_res.source + ")" :
      "MoveToPoseObstacle planning success; execution skipped (obstacle_source=" + obstacle_res.source + ")";
    finish(goal_handle, result, obstacle_added, true, "done", message);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToPoseObstacleActionServer>();
  node->initialize_moveit();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
