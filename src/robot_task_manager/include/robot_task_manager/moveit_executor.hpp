#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_visual_tools/moveit_visual_tools.h"

namespace robot_task_manager
{

class MoveItExecutor
{
public:
  MoveItExecutor() = default;

  void initialize(
    const rclcpp::Node::SharedPtr & node,
    const std::string & planning_group,
    const std::string & base_frame = "base_link");

  bool goNamedTarget(
    const std::string & target_name,
    std::string & error_msg,
    double velocity_scale = 0.3,
    double acceleration_scale = 0.3,
    double planning_time = 5.0);

  bool moveToPose(
          const geometry_msgs::msg::Pose & target_pose,
          std::string & error_msg,
          double velocity_scale = 0.3,
          double acceleration_scale = 0.3,
          double planning_time = 5.0);
  bool moveToPoseCartesian(
          const geometry_msgs::msg::Pose & target_pose,
          std::string & error_msg,
          double velocity_scale = 0.3,
          double acceleration_scale = 0.3,
          double planning_time = 5.0);
  
  bool checkerBoard(
          double step,
          std::string & error_msg,
          double velocity_scale = 0.3,
          double acceleration_scale = 0.3,
          double planning_time = 5.0);

  void stop();

  void publishText(const std::string & text);
  void publishTargetAxis(const geometry_msgs::msg::Pose & pose, const std::string & label = "goal");
  void deleteAllMarkers();

  bool applyCollisionObjects(
    const std::vector<moveit_msgs::msg::CollisionObject> & objects,
    std::string & error_msg);

  bool removeCollisionObjects(
    const std::vector<std::string> & object_ids,
    std::string & error_msg);

  std::string getPlanningFrame() const;
  std::string getEndEffectorLink() const;
  std::string getPlanningGroup() const;

private:
  bool validateScalingFactors(
    double velocity_scale,
    double acceleration_scale,
    std::string & error_msg) const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

  std::string planning_group_;
  std::string base_frame_;
  bool initialized_{false};
  mutable std::mutex motion_mutex_;
};

}  // namespace robot_task_manager
