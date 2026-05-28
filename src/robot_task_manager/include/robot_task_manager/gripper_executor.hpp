#ifndef ROBOT_TASK_MANAGER__GRIPPER_EXECUTOR_HPP_
#define ROBOT_TASK_MANAGER__GRIPPER_EXECUTOR_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"

namespace robot_task_manager
{

class GripperExecutor
{
public:
  GripperExecutor() = default;

  void initialize(
    const rclcpp::Node::SharedPtr & node,
    const std::string & planning_group,
    const std::string & base_frame);

  bool moveToOpening(
    double opening,
    std::string & error_msg,
    double velocity_scale = 0.5,
    double acceleration_scale = 0.5);

  bool open(std::string & error_msg);

  bool close(
    double close_opening,
    std::string & error_msg);

  void stop();

  bool isInitialized() const;

  std::string getPlanningFrame() const;
  std::string getEndEffectorLink() const;
  std::string getPlanningGroup() const;

private:
  bool validateScalingFactors(
    double velocity_scale,
    double acceleration_scale,
    std::string & error_msg) const;

private:
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

  std::string planning_group_;
  std::string base_frame_;

  double max_open_ = 0.051;
  double min_open_ = 0.0;

  bool initialized_ = false;

  mutable std::mutex motion_mutex_;
};

}  // namespace robot_task_manager

#endif  // ROBOT_TASK_MANAGER__GRIPPER_EXECUTOR_HPP_