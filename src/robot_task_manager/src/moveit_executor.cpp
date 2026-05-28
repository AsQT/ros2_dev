#include "robot_task_manager/moveit_executor.hpp"

#include <Eigen/Geometry>
#include <exception>
#include <chrono>

namespace robot_task_manager
{

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

  initialized_ = true;

  RCLCPP_INFO(
    node_->get_logger(),
    "MoveItExecutor initialized | group=%s | planning_frame=%s | ee_link=%s",
    planning_group_.c_str(),
    move_group_->getPlanningFrame().c_str(),
    move_group_->getEndEffectorLink().c_str());
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
/*----------- goNamedTarget -------------------------------------------------------*/
bool MoveItExecutor::goNamedTarget(
                      const std::string & target_name,
                      std::string & error_msg,
                      double velocity_scale,
                      double acceleration_scale,
                      double planning_time)
{
  std::lock_guard<std::mutex> lock(motion_mutex_);

  if (!initialized_) {
    error_msg = "MoveItExecutor not initialized";
    return false;  }

  if (!validateScalingFactors(velocity_scale, acceleration_scale, error_msg)) {
    return false;
  }

  move_group_->setPlanningTime(planning_time);
  move_group_->setMaxVelocityScalingFactor(velocity_scale);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scale);
  move_group_->setStartStateToCurrentState();

  if (!move_group_->setNamedTarget(target_name)) {
    error_msg = "Named target not found: " + target_name;
    return false;
  }

  publishText("Planning to named target: " + target_name);

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  const auto plan_result = move_group_->execute(plan);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    error_msg = "Planning to named target failed: " + target_name;
    return false;
  }

  const auto * joint_model_group =
    move_group_->getRobotModel()->getJointModelGroup(planning_group_);

  if (joint_model_group) {
    visual_tools_->deleteAllMarkers();
    visual_tools_->publishText(Eigen::Isometry3d::Identity(), "GoHome path", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_->publishTrajectoryLine(plan.trajectory, joint_model_group);
    visual_tools_->trigger();
  }

  publishText("Executing named target: " + target_name);

  const auto exec_result = move_group_->execute(plan);

  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    error_msg = "Execution to named target failed: " + target_name;
    return false;
  }

  publishText("Reached named target: " + target_name);
  return true;
}
/*------------ moveToPose -----------------------------------------------------------*/
bool MoveItExecutor::moveToPose(
                      const geometry_msgs::msg::Pose & target_pose,
                      std::string & error_msg,
                      double velocity_scale,
                      double acceleration_scale,
                      double planning_time)
{
  std::lock_guard<std::mutex> lock(motion_mutex_);

  if (!initialized_) {
    error_msg = "MoveItExecutor not initialized";
    return false;
  }

  if (!validateScalingFactors(velocity_scale, acceleration_scale, error_msg)) {
    return false;
  }

  move_group_->setPlanningTime(planning_time);
  move_group_->setMaxVelocityScalingFactor(velocity_scale);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scale);
  move_group_->setStartStateToCurrentState();
  move_group_->setPoseTarget(target_pose);

  visual_tools_->deleteAllMarkers();
  publishTargetAxis(target_pose, "goal_axis");
  publishText("Planning to pose target");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto plan_result = move_group_->plan(plan);

  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    move_group_->clearPoseTargets();
    error_msg = "Planning to pose failed";
    return false;
  }

  const auto * joint_model_group =
    move_group_->getRobotModel()->getJointModelGroup(planning_group_);

  if (joint_model_group) {
    visual_tools_->publishTrajectoryLine(plan.trajectory, joint_model_group);
    visual_tools_->trigger();
  }

  publishText("Executing_pose_target");
  
  const auto exec_result = move_group_->execute(plan);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  move_group_->clearPoseTargets();

  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    error_msg = "Execution to pose failed";
    return false;
  }

  publishText("Reached_pose_target");
  return true;
}
/*------------ moveToPoseCartesian -----------------------------------------------------------*/

bool MoveItExecutor::moveToPoseCartesian(
                      const geometry_msgs::msg::Pose & target_pose,
                      std::string & error_msg,
                      double velocity_scale,
                      double acceleration_scale,
                      double planning_time)
{
  std::lock_guard<std::mutex> lock(motion_mutex_);

  if (!initialized_) {
    error_msg = "MoveItExecutor not initialized";
    return false;
  }

  if (!validateScalingFactors(velocity_scale, acceleration_scale, error_msg)) {
    return false;
  }

  move_group_->setPlanningTime(planning_time);
  move_group_->setMaxVelocityScalingFactor(velocity_scale);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scale);
  move_group_->setStartStateToCurrentState();

  visual_tools_->deleteAllMarkers();
  publishTargetAxis(target_pose, "goal_axis");
  publishText("Planning Cartesian path");

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
    return false;
  }

  const auto * joint_model_group =
    move_group_->getRobotModel()->getJointModelGroup(planning_group_);

  if (joint_model_group) {
    visual_tools_->publishTrajectoryLine(trajectory, joint_model_group);
    visual_tools_->trigger();
  }

  publishText("Executing_cartesian_path");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory = trajectory;

  const auto exec_result = move_group_->execute(plan);

  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    error_msg = "Execution of Cartesian path failed";
    return false;
  }

  publishText("Reached_cartesian_target");
  return true;
}
/*------------ checkerBoard -----------------------------------------------------------*/
bool MoveItExecutor::checkerBoard(
                      double step,
                      std::string & error_msg,
                      double velocity_scale,
                      double acceleration_scale,
                      double planning_time)
{
  std::lock_guard<std::mutex> lock(motion_mutex_);

  if (!initialized_) {
    error_msg = "MoveItExecutor not initialized";
    return false;
  }

  if (!validateScalingFactors(velocity_scale, acceleration_scale, error_msg)) {
    return false;
  }

  move_group_->setPlanningTime(planning_time);
  move_group_->setMaxVelocityScalingFactor(velocity_scale);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scale);
  move_group_->setStartStateToCurrentState();

  geometry_msgs::msg::Pose start_pose = move_group_->getCurrentPose().pose;

  visual_tools_->deleteAllMarkers();
  publishText("Checker_board");

  std::vector<geometry_msgs::msg::Pose> targets;

  // 9 điểm, start_pose là tâm, quét zig-zag
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

  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose current_pose = start_pose;
  const double lift_height = step / 2.0;

  // bắt đầu từ tâm hiện tại
  waypoints.push_back(current_pose);

  for (size_t i = 0; i < targets.size(); ++i) {
    const auto & target = targets[i];

    // đi tới điểm mới đồng thời nâng Z
    geometry_msgs::msg::Pose travel_pose = target;
    travel_pose.position.z = target.position.z + lift_height;
    waypoints.push_back(travel_pose);

    // hạ xuống tại điểm đích
    geometry_msgs::msg::Pose drop_pose = target;
    waypoints.push_back(drop_pose);

    current_pose = target;
  }

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
    return false;
  }

  const auto * joint_model_group =
    move_group_->getRobotModel()->getJointModelGroup(planning_group_);

  if (joint_model_group) {
    visual_tools_->publishTrajectoryLine(trajectory, joint_model_group);
    visual_tools_->trigger();
  }

  publishText("Executing_cartesian_path");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory = trajectory;

  const auto exec_result = move_group_->execute(plan);

  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    error_msg = "Execution of Cartesian path failed";
    return false;
  }

  publishText("Reached_cartesian_target");
  return true;
}
/*-------------------------------------------------------------------------------------*/
void MoveItExecutor::stop()
{
  std::lock_guard<std::mutex> lock(motion_mutex_);

  if (!initialized_) {
    return;
  }

  move_group_->stop();
  move_group_->clearPoseTargets();
  publishText("Motion stopped");
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
                  rviz_visual_tools::WHITE,
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
