#include "robot_task_manager/gripper_executor.hpp"

#include <exception>
#include <vector>

namespace robot_task_manager
{

void GripperExecutor::initialize(
                        const rclcpp::Node::SharedPtr & node,
                        const std::string & planning_group,
                        const std::string & base_frame)
{
  std::lock_guard<std::mutex> lock(motion_mutex_);

  node_ = node;
  planning_group_ = planning_group;
  base_frame_ = base_frame;

  max_open_ = node_->declare_parameter<double>("gripper_max_open", 0.05);
  min_open_ = node_->declare_parameter<double>("gripper_min_open", 0.0);

  try {
    move_group_ =  std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                                                                    node_,
                                                                    planning_group_);

    planning_scene_interface_ =   std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    move_group_->startStateMonitor();

    initialized_ = true;

    RCLCPP_INFO(
            node_->get_logger(),
            "GripperExecutor initialized | group=%s | planning_frame=%s | ee_link=%s | range=[%.4f, %.4f]",
            planning_group_.c_str(),
            move_group_->getPlanningFrame().c_str(),
            move_group_->getEndEffectorLink().c_str(),
            min_open_,
            max_open_);
  }
  catch (const std::exception & e) {
    initialized_ = false;

    RCLCPP_ERROR(
      node_->get_logger(),
      "Failed to initialize GripperExecutor: %s",
      e.what());
  }
}

bool GripperExecutor::validateScalingFactors(
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

bool GripperExecutor::moveToOpening(
                        double opening,
                        std::string & error_msg,
                        double velocity_scale,
                        double acceleration_scale)
{
  std::lock_guard<std::mutex> lock(motion_mutex_);

  if (!initialized_ || !move_group_) {
    error_msg = "GripperExecutor is not initialized";
    return false;
  }

  if (!validateScalingFactors(velocity_scale, acceleration_scale, error_msg)) {
    return false;
  }

  if (opening < min_open_ || opening > max_open_) {
    error_msg =
      "gripper opening out of range: " +
      std::to_string(opening) +
      ", range=[" +
      std::to_string(min_open_) +
      ", " +
      std::to_string(max_open_) +
      "]";
    return false;
  }

  move_group_->stop();
  move_group_->clearPoseTargets();
  move_group_->clearPathConstraints();

  move_group_->setMaxVelocityScalingFactor(velocity_scale);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scale);

  std::vector<double> joint_values = move_group_->getCurrentJointValues();

  if (joint_values.empty()) {
    error_msg = "Cannot get current gripper joint values";
    return false;
  }

  if (joint_values.size() == 1) {
    joint_values[0] = opening;
  } else {
    const double each_joint_opening =
      opening / static_cast<double>(joint_values.size());

    for (auto & joint : joint_values) {
      joint = each_joint_opening;
    }
  }

  move_group_->setJointValueTarget(joint_values);

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  const auto plan_result = move_group_->plan(plan);

  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    error_msg = "Gripper planning failed";
    move_group_->clearPoseTargets();
    return false;
  }

  const auto exec_result = move_group_->execute(plan);

  move_group_->clearPoseTargets();

  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    error_msg = "Gripper execution failed";
    return false;
  }

  error_msg.clear();

  RCLCPP_INFO(
    node_->get_logger(),
    "Gripper moved to opening %.4f m",
    opening);

  return true;
}

bool GripperExecutor::open(std::string & error_msg)
{
  return moveToOpening(max_open_, error_msg, 0.5, 0.5);
}

bool GripperExecutor::close(
  double close_opening,
  std::string & error_msg)
{
  return moveToOpening(close_opening, error_msg, 0.5, 0.5);
}

void GripperExecutor::stop()
{
  std::lock_guard<std::mutex> lock(motion_mutex_);

  if (!initialized_ || !move_group_) {
    return;
  }

  move_group_->stop();
  move_group_->clearPoseTargets();

  RCLCPP_WARN(
    node_->get_logger(),
    "Gripper motion stopped");
}

bool GripperExecutor::isInitialized() const
{
  std::lock_guard<std::mutex> lock(motion_mutex_);
  return initialized_;
}

std::string GripperExecutor::getPlanningFrame() const
{
  std::lock_guard<std::mutex> lock(motion_mutex_);

  if (!initialized_ || !move_group_) {
    return "";
  }

  return move_group_->getPlanningFrame();
}

std::string GripperExecutor::getEndEffectorLink() const
{
  std::lock_guard<std::mutex> lock(motion_mutex_);

  if (!initialized_ || !move_group_) {
    return "";
  }

  return move_group_->getEndEffectorLink();
}

std::string GripperExecutor::getPlanningGroup() const
{
  std::lock_guard<std::mutex> lock(motion_mutex_);
  return planning_group_;
}

}  // namespace robot_task_manager