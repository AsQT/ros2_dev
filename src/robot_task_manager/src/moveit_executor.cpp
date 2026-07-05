#include "robot_task_manager/moveit_executor.hpp"

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <exception>
#include <sstream>
#include <thread>

#include "moveit/robot_trajectory/robot_trajectory.hpp"

namespace robot_task_manager
{

namespace
{
constexpr auto kPathMarkerColor = rviz_visual_tools::LIME_GREEN;
constexpr auto kTextMarkerColor = rviz_visual_tools::BLACK;
constexpr double kCurrentStateTimeoutSec = 2.0;
}  // namespace

void MoveItExecutor::initialize(
                      const rclcpp::Node::SharedPtr & node,
                      const std::string & planning_group,
                      const std::string & base_frame)
{
  std::lock_guard<std::mutex> lock(motion_mutex_);

  node_ = node;
  planning_group_ = planning_group;
  base_frame_ = base_frame;

  move_group_ =
    std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                                                  node_, 
                                                  planning_group_);

  planning_scene_interface_ =
    std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  visual_tools_ =
    std::make_shared<moveit_visual_tools::MoveItVisualTools>(
                                            node_, 
                                            base_frame_, 
                                            "move_group_visualization");

  move_group_->startStateMonitor();
  visual_tools_->deleteAllMarkers();
  visual_tools_->loadRemoteControl();
  planned_joint_trajectory_pub_ =
    node_->create_publisher<moveit_msgs::msg::RobotTrajectory>(
      "/robot_task_manager/last_planned_joint_trajectory", rclcpp::QoS(10).reliable());

  initialized_ = true;

  RCLCPP_INFO(
    node_->get_logger(),
    "MoveItExecutor initialized | group=%s | planning_frame=%s | ee_link=%s",
    planning_group_.c_str(),
    move_group_->getPlanningFrame().c_str(),
    move_group_->getEndEffectorLink().c_str());
}
/*---------- initializeLogging -------------------------------------------------------*/
void MoveItExecutor::initializeLogging(
                      bool enable,
                      const std::string & log_dir,
                      double sample_rate_hz,
                      const std::string & base_frame,
                      const std::string & tcp_frame,
                      const std::string & action_name)
{
  log_action_name_ = action_name;

  if (!enable) {
    RCLCPP_INFO(node_->get_logger(), "MoveItExecutor CSV logging disabled for %s", action_name.c_str());
    return;
  }

  try {
    log_tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    log_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*log_tf_buffer_);
    executor_logger_ = std::make_shared<robot_task_executor::ExecutorExperimentLogger>(
        node_, log_tf_buffer_, log_dir, sample_rate_hz, base_frame, tcp_frame);
    RCLCPP_INFO(
      node_->get_logger(),
      "MoveItExecutor CSV logging enabled for %s | log_dir=%s | rate=%.1fHz | base=%s | tcp=%s",
      action_name.c_str(), log_dir.c_str(), sample_rate_hz, base_frame.c_str(), tcp_frame.c_str());
  } catch (const std::exception & e) {
    executor_logger_.reset();
    RCLCPP_WARN(node_->get_logger(), "MoveItExecutor CSV logger unavailable: %s", e.what());
  }
}
/*---------- executor logging helpers -------------------------------------------------------*/
uint64_t MoveItExecutor::startExecutorLog(const std::string & execute_mode)
{
  if (!executor_logger_ || !executor_logger_->enabled()) {
    return 0;
  }
  return executor_logger_->start_call(log_action_name_, execute_mode, "moveit_executor");
}

void MoveItExecutor::finishExecutorLog(
    uint64_t call_id,
    const std::string & status,
    bool success,
    const std::string & message,
    double fraction)
{
  if (executor_logger_ && executor_logger_->enabled() && call_id != 0) {
    executor_logger_->log_summary(call_id, status, success, message, fraction);
  }
}

void MoveItExecutor::logRefWaypoint(uint64_t call_id, const geometry_msgs::msg::Pose & pose)
{
  if (executor_logger_ && executor_logger_->enabled() && call_id != 0) {
    executor_logger_->log_ref_waypoint(call_id, 0, pose, node_->get_clock()->now());
  }
}

void MoveItExecutor::logJointCommand(
    uint64_t call_id,
    const moveit_msgs::msg::RobotTrajectory & trajectory)
{
  if (executor_logger_ && executor_logger_->enabled() && call_id != 0) {
    executor_logger_->log_joint_command(call_id, trajectory, log_action_name_);
  }
}

void MoveItExecutor::publishPlannedJointTrajectory(
    const moveit_msgs::msg::RobotTrajectory & trajectory)
{
  if (planned_joint_trajectory_pub_) {
    planned_joint_trajectory_pub_->publish(trajectory);
  }
}

bool MoveItExecutor::executeWithLogging(
    uint64_t call_id,
    const std::string & execute_mode,
    const moveit::planning_interface::MoveGroupInterface::Plan & plan,
    const std::vector<geometry_msgs::msg::Pose> & refs)
{
  logJointCommand(call_id, plan.trajectory);
  if (executor_logger_ && executor_logger_->enabled() && call_id != 0) {
    executor_logger_->start_sampling(call_id, execute_mode, refs);
  }
  const auto exec_result = move_group_->execute(plan);
  if (executor_logger_ && executor_logger_->enabled() && call_id != 0) {
    executor_logger_->stop_sampling(call_id);
  }
  return exec_result == moveit::core::MoveItErrorCode::SUCCESS;
}
/*---------- extractTcpWaypoints -------------------------------------------------------*/
std::vector<geometry_msgs::msg::Point> MoveItExecutor::extractTcpWaypoints(
    const moveit_msgs::msg::RobotTrajectory & trajectory)
{
  std::vector<geometry_msgs::msg::Point> waypoints;
  try {
    if (!move_group_ || trajectory.joint_trajectory.points.empty()) {
      return waypoints;
    }
    const auto current_state = move_group_->getCurrentState(kCurrentStateTimeoutSec);
    if (!current_state) {
      return waypoints;
    }
    robot_trajectory::RobotTrajectory robot_traj(current_state->getRobotModel(), planning_group_);
    robot_traj.setRobotTrajectoryMsg(*current_state, trajectory);

    const std::string & ee_link = move_group_->getEndEffectorLink();
    waypoints.reserve(robot_traj.getWayPointCount());
    for (size_t i = 0; i < robot_traj.getWayPointCount(); ++i) {
      const Eigen::Isometry3d & tf = robot_traj.getWayPoint(i).getGlobalLinkTransform(ee_link);
      geometry_msgs::msg::Point p;
      p.x = tf.translation().x();
      p.y = tf.translation().y();
      p.z = tf.translation().z();
      waypoints.push_back(p);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_->get_logger(), "extractTcpWaypoints failed: %s", e.what());
    waypoints.clear();
  }
  return waypoints;
}
/*---------- validateScalingFactors -------------------------------------------------------*/
bool MoveItExecutor::validateScalingFactors(
                          double velocity_scale,
                          double acceleration_scale,
                          std::string & error_msg) const
  {
    if (velocity_scale <= 0.0 || velocity_scale > 1.0) {
      error_msg = "velocity_scale must be in (0, 1]";
      return false;
    }

    if (acceleration_scale <= 0.0 || acceleration_scale > 1.0) {
      error_msg = "acceleration_scale must be in (0, 1]";
      return false;
    }
    return true;
  }
/*---------- getCurrentStateForPlanning ----------------------------------------------*/
moveit::core::RobotStatePtr MoveItExecutor::getCurrentStateForPlanning(
  double timeout_sec,
  std::string & error_msg)
{
  if (!move_group_) {
    error_msg = "MoveGroupInterface not initialized";
    return nullptr;
  }

  move_group_->startStateMonitor();
  auto current_state = move_group_->getCurrentState(timeout_sec);

  if (!current_state) {
    error_msg =
      "Failed to get current robot state from /joint_states. "
      "Refusing to plan from zero/default state.";
    RCLCPP_ERROR(node_->get_logger(), "%s", error_msg.c_str());
    return nullptr;
  }

  move_group_->setStartState(*current_state);
  return current_state;
}
/*----------- goNamedTarget -------------------------------------------------------*/
bool MoveItExecutor::goNamedTarget(
                      const std::string & target_name,
                      std::string & error_msg,
                      double velocity_scale,
                      double acceleration_scale,
                      double planning_time,
                      bool execute)
{
  std::lock_guard<std::mutex> lock(motion_mutex_);

  const uint64_t call_id = startExecutorLog(execute ? "joint_target" : "plan_only");

  if (!initialized_) {
    error_msg = "MoveItExecutor not initialized";
    finishExecutorLog(call_id, "failed", false, error_msg);
    return false;
  }

  if (!validateScalingFactors(velocity_scale, acceleration_scale, error_msg)) {
    finishExecutorLog(call_id, "failed", false, error_msg);
    return false;
  }

  move_group_->setPlanningTime(planning_time);
  move_group_->setMaxVelocityScalingFactor(velocity_scale);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scale);
  const auto current_state =
    getCurrentStateForPlanning(kCurrentStateTimeoutSec, error_msg);
  if (!current_state) {
    finishExecutorLog(call_id, "failed", false, error_msg);
    return false;
  }

  if (!move_group_->setNamedTarget(target_name)) {
    error_msg = "Named target not found: " + target_name;
    finishExecutorLog(call_id, "failed", false, error_msg);
    return false;
  }

  publishText("Planning to named target: " + target_name);

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  const auto plan_result = move_group_->plan(plan);

  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    error_msg = "Planning to named target failed: " + target_name;
    finishExecutorLog(call_id, "failed", false, error_msg);
    return false;
  }

  const auto * joint_model_group =
    move_group_->getRobotModel()->getJointModelGroup(planning_group_);

  if (joint_model_group) {
    visual_tools_->deleteAllMarkers();
    visual_tools_->publishText(Eigen::Isometry3d::Identity(), "GoHome path", kTextMarkerColor, rviz_visual_tools::XLARGE);
    visual_tools_->publishTrajectoryLine(plan.trajectory, joint_model_group, kPathMarkerColor);
    visual_tools_->trigger();
  }

  if (!execute) {
    publishPlannedJointTrajectory(plan.trajectory);
    logJointCommand(call_id, plan.trajectory);
    publishText("Named target planning succeeded; execution skipped");
    error_msg.clear();
    finishExecutorLog(call_id, "completed", true, "Named target '" + target_name + "' planned successfully");
    return true;
  }

  publishText("Executing named target: " + target_name);

  publishPlannedJointTrajectory(plan.trajectory);
  const bool exec_ok = executeWithLogging(call_id, "joint_target", plan, {});

  if (!exec_ok) {
    error_msg = "Execution to named target failed: " + target_name;
    finishExecutorLog(call_id, "failed", false, error_msg);
    return false;
  }

  publishText("Reached named target: " + target_name);
  finishExecutorLog(call_id, "completed", true, "Named target '" + target_name + "' executed successfully");
  return true;
}
/*------------ moveToPose -----------------------------------------------------------*/
bool MoveItExecutor::moveToPose(
                      const geometry_msgs::msg::Pose & target_pose,
                      std::string & error_msg,
                      double velocity_scale,
                      double acceleration_scale,
                      double planning_time,
                      bool execute,
                      MoveItPlanMetrics * out_metrics)
{
  std::lock_guard<std::mutex> lock(motion_mutex_);

  const uint64_t call_id = startExecutorLog(execute ? "ptp" : "plan_only");

  if (out_metrics) {
    out_metrics->allowed_planning_time_s = planning_time;
    out_metrics->planning_group = planning_group_;
  }

  if (!initialized_) {
    error_msg = "MoveItExecutor not initialized";
    finishExecutorLog(call_id, "failed", false, error_msg);
    return false;
  }

  if (!validateScalingFactors(velocity_scale, acceleration_scale, error_msg)) {
    finishExecutorLog(call_id, "failed", false, error_msg);
    return false;
  }

  move_group_->setPlanningTime(planning_time);
  move_group_->setMaxVelocityScalingFactor(velocity_scale);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scale);
  const auto current_state =
    getCurrentStateForPlanning(kCurrentStateTimeoutSec, error_msg);
  if (!current_state) {
    finishExecutorLog(call_id, "failed", false, error_msg);
    return false;
  }
  move_group_->setPoseTarget(target_pose);
  logRefWaypoint(call_id, target_pose);

  visual_tools_->deleteAllMarkers();
  publishTargetAxis(target_pose, "goal_axis");
  publishText("Planning to pose target");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto plan_start = std::chrono::steady_clock::now();
  const auto plan_result = move_group_->plan(plan);
  const double plan_elapsed_s =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - plan_start).count();

  if (out_metrics) {
    out_metrics->moveit_error_code = plan_result.val;
    out_metrics->planner_id = move_group_->getPlannerId();
  }

  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    move_group_->clearPoseTargets();
    error_msg = "Planning to pose failed";
    finishExecutorLog(call_id, "failed", false, error_msg);
    if (out_metrics) {
      out_metrics->planning_time_s = plan_elapsed_s;
    }
    return false;
  }

  if (out_metrics) {
    out_metrics->has_plan = true;
    // plan.planning_time (MoveIt-reported) is preferred when available;
    // fall back to our own wall-clock measurement around plan() otherwise.
    out_metrics->planning_time_s =
      plan.planning_time > 0.0 ? plan.planning_time : plan_elapsed_s;
    out_metrics->robot_trajectory = plan.trajectory;
    out_metrics->tcp_waypoints = extractTcpWaypoints(plan.trajectory);
    out_metrics->tcp_poses.clear();
    out_metrics->tcp_poses.reserve(out_metrics->tcp_waypoints.size());
    for (const auto & p : out_metrics->tcp_waypoints) {
      geometry_msgs::msg::Pose pose;
      pose.position = p;
      pose.orientation.w = 1.0;
      out_metrics->tcp_poses.push_back(pose);
    }
  }

  const auto * joint_model_group =
    move_group_->getRobotModel()->getJointModelGroup(planning_group_);

  if (joint_model_group) {
    visual_tools_->publishTrajectoryLine(plan.trajectory, joint_model_group, kPathMarkerColor);
    visual_tools_->trigger();
  }

  if (!execute) {
    publishPlannedJointTrajectory(plan.trajectory);
    logJointCommand(call_id, plan.trajectory);
    move_group_->clearPoseTargets();
    publishText("Pose planning succeeded; execution skipped");
    error_msg.clear();
    finishExecutorLog(call_id, "completed", true, "Pose target planned successfully");
    return true;
  }

  publishText("Executing_pose_target");

  publishPlannedJointTrajectory(plan.trajectory);
  const auto exec_start = std::chrono::steady_clock::now();
  const bool exec_ok = executeWithLogging(call_id, "ptp", plan, {target_pose});
  const double exec_elapsed_s =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - exec_start).count();
  if (out_metrics) {
    out_metrics->execution_time_s = exec_elapsed_s;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  move_group_->clearPoseTargets();

  if (!exec_ok) {
    error_msg = "Execution to pose failed";
    finishExecutorLog(call_id, "failed", false, error_msg);
    return false;
  }

  publishText("Reached_pose_target");
  finishExecutorLog(call_id, "completed", true, "Pose target executed successfully");
  return true;
}
/*------------ moveToPoseCartesian -----------------------------------------------------------*/

bool MoveItExecutor::moveToPoseCartesian(
                      const geometry_msgs::msg::Pose & target_pose,
                      std::string & error_msg,
                      double velocity_scale,
                      double acceleration_scale,
                      double planning_time,
                      bool execute)
{
  std::lock_guard<std::mutex> lock(motion_mutex_);

  const uint64_t call_id = startExecutorLog(execute ? "cartesian" : "plan_only");

  if (!initialized_) {
    error_msg = "MoveItExecutor not initialized";
    finishExecutorLog(call_id, "failed", false, error_msg);
    return false;
  }

  if (!validateScalingFactors(velocity_scale, acceleration_scale, error_msg)) {
    finishExecutorLog(call_id, "failed", false, error_msg);
    return false;
  }

  move_group_->setPlanningTime(planning_time);
  move_group_->setMaxVelocityScalingFactor(velocity_scale);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scale);
  const auto current_state =
    getCurrentStateForPlanning(kCurrentStateTimeoutSec, error_msg);
  if (!current_state) {
    finishExecutorLog(call_id, "failed", false, error_msg);
    return false;
  }

  visual_tools_->deleteAllMarkers();
  publishTargetAxis(target_pose, "goal_axis");
  publishText("Planning Cartesian path");
  logRefWaypoint(call_id, target_pose);

  geometry_msgs::msg::Pose start_pose = move_group_->getCurrentPose().pose;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(start_pose);
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;

  const double eef_step = 0.01;
  const bool avoid_collisions = true;

  double fraction = move_group_->computeCartesianPath(
      waypoints,
      eef_step,
      trajectory,
      avoid_collisions);

  if (fraction < 0.99) {
    error_msg = "Cartesian path planning failed, fraction = " + std::to_string(fraction);
    finishExecutorLog(call_id, "failed", false, error_msg, fraction);
    return false;
  }

  const auto * joint_model_group =
    move_group_->getRobotModel()->getJointModelGroup(planning_group_);

  if (joint_model_group) {
    visual_tools_->publishTrajectoryLine(trajectory, joint_model_group, kPathMarkerColor);
    visual_tools_->trigger();
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory = trajectory;

  if (!execute) {
    publishPlannedJointTrajectory(trajectory);
    logJointCommand(call_id, trajectory);
    publishText("Cartesian planning succeeded; execution skipped");
    error_msg.clear();
    finishExecutorLog(call_id, "completed", true, "Cartesian path planned successfully", fraction);
    return true;
  }

  publishText("Executing_cartesian_path");

  publishPlannedJointTrajectory(trajectory);
  const bool exec_ok = executeWithLogging(call_id, "cartesian", plan, {target_pose});

  if (!exec_ok) {
    error_msg = "Execution of Cartesian path failed";
    finishExecutorLog(call_id, "failed", false, error_msg, fraction);
    return false;
  }

  publishText("Reached_cartesian_target");
  finishExecutorLog(call_id, "completed", true, "Cartesian path executed successfully", fraction);
  return true;
}
/*------------ executeCartesianSegment -----------------------------------------------------------*/
bool MoveItExecutor::executeCartesianSegment(
                      const geometry_msgs::msg::Pose & target_pose,
                      std::string & error_msg,
                      double velocity_scale,
                      double acceleration_scale,
                      double planning_time,
                      bool execute,
                      const geometry_msgs::msg::Pose * planned_start_pose,
                      const std::string & stage,
                      JointTrajectoryCallback joint_trajectory_cb)
{
  const uint64_t call_id = startExecutorLog(execute ? "cartesian" : "plan_only");

  if (!initialized_) {
    error_msg = "MoveItExecutor not initialized";
    finishExecutorLog(call_id, "failed", false, error_msg);
    return false;
  }

  if (!validateScalingFactors(velocity_scale, acceleration_scale, error_msg)) {
    finishExecutorLog(call_id, "failed", false, error_msg);
    return false;
  }

  move_group_->setPlanningTime(planning_time);
  move_group_->setMaxVelocityScalingFactor(velocity_scale);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scale);

  const auto current_state =
    getCurrentStateForPlanning(kCurrentStateTimeoutSec, error_msg);
  if (!current_state) {
    finishExecutorLog(call_id, "failed", false, error_msg);
    return false;
  }

  geometry_msgs::msg::Pose start_pose = planned_start_pose ?
    *planned_start_pose : move_group_->getCurrentPose().pose;

  logRefWaypoint(call_id, target_pose);

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(start_pose);
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;

  const double eef_step = 0.01;
  const bool avoid_collisions = true;

  const double fraction = move_group_->computeCartesianPath(
      waypoints,
      eef_step,
      trajectory,
      avoid_collisions);

  if (fraction < 0.99) {
    error_msg = "Cartesian segment planning failed, fraction = " + std::to_string(fraction);
    finishExecutorLog(call_id, "failed", false, error_msg, fraction);
    return false;
  }

  const auto * joint_model_group =
    move_group_->getRobotModel()->getJointModelGroup(planning_group_);

  if (joint_model_group) {
    visual_tools_->publishTrajectoryLine(trajectory, joint_model_group, kPathMarkerColor);
    visual_tools_->trigger();
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory = trajectory;
  if (joint_trajectory_cb) {
    joint_trajectory_cb(stage, trajectory);
  }

  if (!execute) {
    publishPlannedJointTrajectory(trajectory);
    logJointCommand(call_id, trajectory);
    error_msg.clear();
    finishExecutorLog(call_id, "completed", true, "Cartesian segment planned successfully", fraction);
    return true;
  }

  publishPlannedJointTrajectory(trajectory);
  const bool exec_ok = executeWithLogging(call_id, "cartesian", plan, {target_pose});

  if (!exec_ok) {
    error_msg = "Execution of Cartesian segment failed";
    finishExecutorLog(call_id, "failed", false, error_msg, fraction);
    return false;
  }

  finishExecutorLog(call_id, "completed", true, "Cartesian segment executed successfully", fraction);
  return true;
}
/*------------ checkerBoard -----------------------------------------------------------*/
bool MoveItExecutor::checkerBoard(
                      double step,
                      std::string & error_msg,
                      double velocity_scale,
                      double acceleration_scale,
                      double planning_time,
                      bool execute,
                      double measurement_settle_time_s,
                      FeedbackCallback feedback_cb,
                      JointTrajectoryCallback joint_trajectory_cb)
{
  std::lock_guard<std::mutex> lock(motion_mutex_);

  if (!initialized_) {
    error_msg = "MoveItExecutor not initialized";
    return false;
  }

  if (!std::isfinite(step) || step <= 0.0) {
    error_msg = "CheckerBoard step must be finite and > 0.0";
    return false;
  }

  if (!std::isfinite(measurement_settle_time_s) || measurement_settle_time_s < 0.0) {
    error_msg = "measurement_settle_time_s must be finite and >= 0.0";
    return false;
  }

  if (!validateScalingFactors(velocity_scale, acceleration_scale, error_msg)) {
    return false;
  }

  move_group_->setPlanningTime(planning_time);
  move_group_->setMaxVelocityScalingFactor(velocity_scale);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scale);
  const auto current_state =
    getCurrentStateForPlanning(kCurrentStateTimeoutSec, error_msg);
  if (!current_state) {
    return false;
  }

  if (feedback_cb) {
    feedback_cb("CheckerBoard current pose acquired", 5.0f);
  }

  geometry_msgs::msg::Pose start_pose = move_group_->getCurrentPose().pose;

  visual_tools_->deleteAllMarkers();
  publishText("Checker_board");

  std::vector<geometry_msgs::msg::Pose> targets;

  for (int row = 1; row >= -1; --row) {
    if ((1 - row) % 2 == 0) {
      for (int col = -1; col <= 1; ++col) {
        geometry_msgs::msg::Pose p = start_pose;
        p.position.x += col * step;
        p.position.y += row * step;
        targets.push_back(p);
      }
    } else {
      for (int col = 1; col >= -1; --col) {
        geometry_msgs::msg::Pose p = start_pose;
        p.position.x += col * step;
        p.position.y += row * step;
        targets.push_back(p);
      }
    }
  }

  if (feedback_cb) {
    feedback_cb("CheckerBoard generated 3x3 zig-zag grid", 10.0f);
  }

  geometry_msgs::msg::Pose planned_pose = start_pose;
  const double lift_height = step / 2.0;
  const float progress_start = 10.0f;
  const float progress_span = 85.0f;
  const float segment_count = static_cast<float>(targets.size() * 2);

  size_t completed_segments = 0;
  for (size_t i = 0; i < targets.size(); ++i) {
    const auto & target = targets[i];
    const size_t target_number = i + 1;

    geometry_msgs::msg::Pose travel_pose = target;
    travel_pose.position.z = target.position.z + lift_height;

    std::ostringstream travel_stage;
    travel_stage << "Target " << target_number << "/" << targets.size()
                 << ": move to travel pose";
    if (feedback_cb) {
      feedback_cb(
        travel_stage.str(),
        progress_start + progress_span * (static_cast<float>(completed_segments) / segment_count));
    }
    publishText(travel_stage.str());

    if (!executeCartesianSegment(
        travel_pose,
        error_msg,
        velocity_scale,
        acceleration_scale,
        planning_time,
        execute,
        execute ? nullptr : &planned_pose,
        travel_stage.str(),
        joint_trajectory_cb))
    {
      error_msg = travel_stage.str() + " failed: " + error_msg;
      return false;
    }
    planned_pose = travel_pose;
    ++completed_segments;

    geometry_msgs::msg::Pose drop_pose = target;
    std::ostringstream drop_stage;
    drop_stage << "Target " << target_number << "/" << targets.size()
               << ": move down to measurement pose";
    if (feedback_cb) {
      feedback_cb(
        drop_stage.str(),
        progress_start + progress_span * (static_cast<float>(completed_segments) / segment_count));
    }
    publishText(drop_stage.str());

    if (!executeCartesianSegment(
        drop_pose,
        error_msg,
        velocity_scale,
        acceleration_scale,
        planning_time,
        execute,
        execute ? nullptr : &planned_pose,
        drop_stage.str(),
        joint_trajectory_cb))
    {
      error_msg = drop_stage.str() + " failed: " + error_msg;
      return false;
    }
    planned_pose = drop_pose;
    ++completed_segments;

    if (execute) {
      std::ostringstream wait_stage;
      wait_stage << "CheckerBoard target " << target_number << "/" << targets.size()
                 << " measurement wait " << measurement_settle_time_s << "s";
      if (feedback_cb) {
        feedback_cb(
          wait_stage.str(),
          progress_start + progress_span * (static_cast<float>(completed_segments) / segment_count));
      }
      publishText(wait_stage.str());
      std::this_thread::sleep_for(
        std::chrono::duration<double>(measurement_settle_time_s));
    }
  }

  publishText(
    execute ?
    "CheckerBoard segmented motion completed successfully" :
    "CheckerBoard segmented planning succeeded; execution skipped");

  error_msg.clear();
  return true;
}
/*-------------------------------------------------------------------------------------*/
void MoveItExecutor::stop()
{
  // Deliberately does NOT take motion_mutex_: that mutex is held by
  // moveToPose()/goNamedTarget()/moveToPoseCartesian() for the WHOLE
  // plan+execute duration. Taking it here would block until the motion has
  // already finished, so move_group_->stop() would run too late to ever
  // interrupt anything — that is exactly the "bấm Stop nhưng robot vẫn di
  // chuyển" bug (codex.md section 7: Stop must actually halt motion, not
  // just cancel action logic). MoveGroupInterface::stop() is safe to call
  // from a different thread while an execute() is in flight — it cancels the
  // active trajectory via the execution manager. initialized_/move_group_
  // are set once at init and never mutated afterwards, so reading them
  // without the lock is safe; we avoid clearPoseTargets() here to not race
  // with a concurrent plan() on the motion thread.
  if (!initialized_ || !move_group_) {
    if (node_) {
      RCLCPP_WARN(node_->get_logger(), "MoveItExecutor::stop(): ignored because executor is not initialized");
    }
    return;
  }
  RCLCPP_WARN(node_->get_logger(), "MoveItExecutor::stop(): calling move_group_->stop()");
  move_group_->stop();
}
/*-------------------------------------------------------------------------------------*/
void MoveItExecutor::publishText(const std::string & text)
{
  if (!initialized_ || !visual_tools_) {
    return;
  }

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;

  visual_tools_->publishText(
                  text_pose,
                  text,
                  kTextMarkerColor,
                  rviz_visual_tools::XLARGE);
  visual_tools_->trigger();
}
/*-------------------------------------------------------------------------------------*/
void MoveItExecutor::publishTargetAxis(const geometry_msgs::msg::Pose & pose, const std::string & label)
{
  if (!initialized_ || !visual_tools_) {
    return;
  }

  visual_tools_->publishAxisLabeled(pose, label);
  visual_tools_->trigger();
}
/*-------------------------------------------------------------------------------------*/
void MoveItExecutor::deleteAllMarkers()
{
  if (!initialized_ || !visual_tools_) {
    return;
  }

  visual_tools_->deleteAllMarkers();
  visual_tools_->trigger();
}
/*-------------------------------------------------------------------------------------*/
bool MoveItExecutor::applyCollisionObjects(
                      const std::vector<moveit_msgs::msg::CollisionObject> & objects,
                      std::string & error_msg)
{
  if (!initialized_ || !planning_scene_interface_) {
    error_msg = "PlanningSceneInterface not initialized";
    return false;
  }

  try {
    planning_scene_interface_->applyCollisionObjects(objects);
    return true;
  } catch (const std::exception & e) {
    error_msg = std::string("applyCollisionObjects failed: ") + e.what();
    return false;
  }
}
/*-------------------------------------------------------------------------------------*/
bool MoveItExecutor::removeCollisionObjects(
                      const std::vector<std::string> & object_ids,
                      std::string & error_msg)
{
  if (!initialized_ || !planning_scene_interface_) {
    error_msg = "PlanningSceneInterface not initialized";
    return false;
  }

  try {
    planning_scene_interface_->removeCollisionObjects(object_ids);
    return true;
  } catch (const std::exception & e) {
    error_msg = std::string("removeCollisionObjects failed: ") + e.what();
    return false;
  }
}

std::string MoveItExecutor::getPlanningFrame() const
{
  return initialized_ ? move_group_->getPlanningFrame() : "";
}

std::string MoveItExecutor::getEndEffectorLink() const
{
  return initialized_ ? move_group_->getEndEffectorLink() : "";
}

std::string MoveItExecutor::getPlanningGroup() const
{
  return planning_group_;
}

}  // namespace robot_task_manager
