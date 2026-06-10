#include "gp7_task_executor/visualization_utils.h"
#include "gp7_task_executor/transform_utils.h"

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/robot_state/robot_state.h"

namespace gp7_task_executor
{

void VisualizationUtils::init(
    std::shared_ptr<rclcpp::Node> node,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
    const std::string& viz_frame,
    TransformUtils* transform_utils)
{
  node_ = std::move(node);
  move_group_ = std::move(move_group);
  viz_frame_ = viz_frame;
  transform_utils_ = transform_utils;

  pub_tool_path_ = node_->create_publisher<nav_msgs::msg::Path>(
      "/task_executor/planned_tool_path", 1);
  pub_waypoint_markers_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/task_executor/planned_waypoints_marker", 1);
  pub_path_line_marker_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "/task_executor/planned_path_line_marker", 1);

  RCLCPP_INFO(logger(),
              "[Viz] Visualization frame: '%s'.", viz_frame_.c_str());
  RCLCPP_INFO(logger(),
              "[Viz] Publishers: /task_executor/planned_tool_path, "
              "/task_executor/planned_waypoints_marker, "
              "/task_executor/planned_path_line_marker");
}

std::string VisualizationUtils::resolve_viz_frame(const std::string& /* planning_frame */)
{
  return viz_frame_;
}

void VisualizationUtils::publish_plan_visualization(
    const moveit_msgs::msg::RobotTrajectory& trajectory,
    const std::string& service_name)
{
  const std::string planning_frame = move_group_->getPlanningFrame();
  const std::string resolved_frame = resolve_viz_frame(planning_frame);
  const std::string ee_link = move_group_->getEndEffectorLink();

  const auto& joint_names = trajectory.joint_trajectory.joint_names;
  const auto& points = trajectory.joint_trajectory.points;

  if (joint_names.empty() || points.empty())
  {
    RCLCPP_WARN(logger(), "[Viz] Trajectory is empty — nothing to visualize.");
    return;
  }

  RCLCPP_INFO(logger(), "[Viz] === Visualization for '%s' ===", service_name.c_str());
  RCLCPP_INFO(logger(), "[Viz] Trajectory points: %zu", points.size());
  RCLCPP_INFO(logger(), "[Viz] EE link: '%s'", ee_link.c_str());
  RCLCPP_INFO(logger(), "[Viz] Frame: '%s'", resolved_frame.c_str());

  std::vector<geometry_msgs::msg::PoseStamped> tcp_poses;
  tcp_poses.reserve(points.size());

  moveit::core::RobotState robot_state(move_group_->getRobotModel());
  for (size_t i = 0; i < points.size(); ++i)
  {
    if (points[i].time_from_start.sec == 0 && points[i].time_from_start.nanosec == 0)
    {
      continue;
    }

    robot_state.setJointGroupPositions(move_group_->getName(), points[i].positions);

    const Eigen::Isometry3d tcp_eigen = robot_state.getGlobalLinkTransform(ee_link);

    geometry_msgs::msg::Pose pose_world;
    pose_world.position.x = static_cast<double>(tcp_eigen.translation().x());
    pose_world.position.y = static_cast<double>(tcp_eigen.translation().y());
    pose_world.position.z = static_cast<double>(tcp_eigen.translation().z());
    const Eigen::Quaterniond quat(tcp_eigen.rotation());
    pose_world.orientation.x = quat.x();
    pose_world.orientation.y = quat.y();
    pose_world.orientation.z = quat.z();
    pose_world.orientation.w = quat.w();

    geometry_msgs::msg::PoseStamped ps_world;
    ps_world.header.stamp = node_->get_clock()->now();
    ps_world.header.frame_id = planning_frame;
    ps_world.pose = pose_world;

    geometry_msgs::msg::PoseStamped ps_base;
    if (planning_frame == viz_frame_)
    {
      ps_base = ps_world;
    }
    else
    {
      if (!transform_utils_)
      {
        RCLCPP_ERROR(logger(),
                     "[Viz] FK pose is in '%s' but transform_utils_ is null — "
                     "cannot convert to '%s'. Skipping transform.",
                     planning_frame.c_str(), viz_frame_.c_str());
        return;
      }
      try
      {
        ps_base = transform_utils_->transform_to_planning_frame(ps_world, viz_frame_);
      }
      catch (const std::exception& ex)
      {
        RCLCPP_ERROR(logger(),
                     "[Viz] TF transform FK pose from '%s' to '%s' failed: %s. Skipping transform.",
                     planning_frame.c_str(), viz_frame_.c_str(), ex.what());
        return;
      }
    }

    geometry_msgs::msg::PoseStamped tcp_pose;
    tcp_pose.header.stamp = node_->get_clock()->now();
    tcp_pose.header.frame_id = viz_frame_;
    tcp_pose.pose = ps_base.pose;
    tcp_poses.push_back(tcp_pose);
  }

  RCLCPP_INFO(logger(), "[Viz] Unique TCP poses: %zu", tcp_poses.size());
  RCLCPP_INFO(logger(), "[Viz] Start xyz: (%.4f, %.4f, %.4f)",
               tcp_poses.front().pose.position.x,
               tcp_poses.front().pose.position.y,
               tcp_poses.front().pose.position.z);
  RCLCPP_INFO(logger(), "[Viz] Goal xyz:   (%.4f, %.4f, %.4f)",
               tcp_poses.back().pose.position.x,
               tcp_poses.back().pose.position.y,
               tcp_poses.back().pose.position.z);

  // 1. nav_msgs/Path
  nav_msgs::msg::Path nav_path;
  nav_path.header.stamp = node_->get_clock()->now();
  nav_path.header.frame_id = resolved_frame;
  for (const auto& pose : tcp_poses)
  {
    nav_path.poses.push_back(pose);
  }
  pub_tool_path_->publish(nav_path);

  // 2. MarkerArray: waypoint spheres + start/goal
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker waypoint_spheres;
  waypoint_spheres.header.stamp = node_->get_clock()->now();
  waypoint_spheres.header.frame_id = resolved_frame;
  waypoint_spheres.ns = "trajectory_waypoints";
  waypoint_spheres.id = 0;
  waypoint_spheres.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  waypoint_spheres.action = visualization_msgs::msg::Marker::ADD;
  waypoint_spheres.scale.x = 0.015;
  waypoint_spheres.scale.y = 0.015;
  waypoint_spheres.scale.z = 0.015;
  waypoint_spheres.color.r = 0.0;
  waypoint_spheres.color.g = 0.8;
  waypoint_spheres.color.b = 1.0;
  waypoint_spheres.color.a = 1.0;
  for (const auto& pose : tcp_poses)
  {
    waypoint_spheres.points.emplace_back(pose.pose.position);
  }
  marker_array.markers.push_back(waypoint_spheres);

  if (!tcp_poses.empty())
  {
    visualization_msgs::msg::Marker start_marker;
    start_marker.header.stamp = node_->get_clock()->now();
    start_marker.header.frame_id = resolved_frame;
    start_marker.ns = "start_goal";
    start_marker.id = 1;
    start_marker.type = visualization_msgs::msg::Marker::SPHERE;
    start_marker.action = visualization_msgs::msg::Marker::ADD;
    start_marker.scale.x = 0.04;
    start_marker.scale.y = 0.04;
    start_marker.scale.z = 0.04;
    start_marker.color.r = 0.2;
    start_marker.color.g = 1.0;
    start_marker.color.b = 0.2;
    start_marker.color.a = 1.0;
    start_marker.pose.position = tcp_poses.front().pose.position;
    start_marker.pose.orientation.w = 1.0;
    marker_array.markers.push_back(start_marker);

    visualization_msgs::msg::Marker goal_marker;
    goal_marker.header.stamp = node_->get_clock()->now();
    goal_marker.header.frame_id = resolved_frame;
    goal_marker.ns = "start_goal";
    goal_marker.id = 2;
    goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
    goal_marker.action = visualization_msgs::msg::Marker::ADD;
    goal_marker.scale.x = 0.04;
    goal_marker.scale.y = 0.04;
    goal_marker.scale.z = 0.04;
    goal_marker.color.r = 1.0;
    goal_marker.color.g = 0.2;
    goal_marker.color.b = 0.2;
    goal_marker.color.a = 1.0;
    goal_marker.pose.position = tcp_poses.back().pose.position;
    goal_marker.pose.orientation.w = 1.0;
    marker_array.markers.push_back(goal_marker);
  }
  pub_waypoint_markers_->publish(marker_array);

  // 3. LINE_STRIP marker
  visualization_msgs::msg::Marker path_line;
  path_line.header.stamp = node_->get_clock()->now();
  path_line.header.frame_id = resolved_frame;
  path_line.ns = "tcp_path_line";
  path_line.id = 0;
  path_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_line.action = visualization_msgs::msg::Marker::ADD;
  path_line.scale.x = 0.008;
  path_line.color.r = 1.0;
  path_line.color.g = 0.84;
  path_line.color.b = 0.0;
  path_line.color.a = 0.9;
  for (const auto& pose : tcp_poses)
  {
    path_line.points.emplace_back(pose.pose.position);
  }
  pub_path_line_marker_->publish(path_line);

  RCLCPP_INFO(logger(),
              "[Viz] Published: Path (%zu poses), MarkerArray (%zu markers), LINE_STRIP (%zu points).",
              nav_path.poses.size(), marker_array.markers.size(), path_line.points.size());
  RCLCPP_INFO(logger(), "[Viz] =======================================");
}

}  // namespace gp7_task_executor
