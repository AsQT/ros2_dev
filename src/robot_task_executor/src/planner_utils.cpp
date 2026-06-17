#include "robot_task_executor/planner_utils.h"
#include "robot_task_executor/transform_utils.h"
#include "robot_task_executor/waypoint_loader.h"

#include <cmath>

namespace robot_task_executor
{

void PlannerUtils::init(
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
    std::shared_ptr<rclcpp::Node> node,
    const double planning_time,
    const int num_planning_attempts,
    const double max_velocity_scaling,
    const double max_acceleration_scaling,
    const WaypointLoader* waypoint_loader,
    TransformUtils* transform_utils,
    const std::string& task_frame)
{
  move_group_ = std::move(move_group);
  node_ = std::move(node);
  waypoint_loader_ = waypoint_loader;
  transform_utils_ = transform_utils;
  planning_time_ = planning_time;
  num_planning_attempts_ = num_planning_attempts;
  max_velocity_scaling_ = max_velocity_scaling;
  max_acceleration_scaling_ = max_acceleration_scaling;
  task_frame_ = task_frame;

  // Fixed orientation for Cartesian moves used by the DRL mock hardware path.
  cartesian_quat_.x = 0.7071068;
  cartesian_quat_.y = 0.7071068;
  cartesian_quat_.z = 0.0;
  cartesian_quat_.w = 0.0;

  RCLCPP_INFO(logger(),
              "[Planner] Cartesian fixed orientation: "
              "quat=(%.6f, %.6f, %.6f, %.6f)",
              cartesian_quat_.x, cartesian_quat_.y,
              cartesian_quat_.z, cartesian_quat_.w);
}

void PlannerUtils::set_task_frame(const std::string& frame)
{
  task_frame_ = frame;
}

void PlannerUtils::rpy_to_quaternion(const double roll, const double pitch, const double yaw,
                                   double& qx, double& qy, double& qz, double& qw)
{
  const double cy = std::cos(yaw   * 0.5);
  const double sy = std::sin(yaw   * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll  * 0.5);
  const double sr = std::sin(roll  * 0.5);

  qw = cr * cp * cy + sr * sp * sy;
  qx = sr * cp * cy - cr * sp * sy;
  qy = cr * sp * cy + sr * cp * sy;
  qz = cr * cp * sy - sr * sp * cy;

  const double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  qw /= norm;
  qx /= norm;
  qy /= norm;
  qz /= norm;
}

PlanResult PlannerUtils::plan_joint_target(
    const std::string& target_name,
    const std::vector<double>& joint_positions,
    const std::vector<std::string>& joint_names,
    const bool execute)
{
  PlanResult result;

  RCLCPP_INFO(logger(), "[Planner] Joint target '%s': %zu joints, execute=%d",
               target_name.c_str(), joint_names.size(), execute);

  move_group_->setStartStateToCurrentState();

  std::map<std::string, double> target_map;
  for (size_t i = 0; i < joint_names.size(); ++i)
  {
    target_map[joint_names[i]] = joint_positions[i];
  }

  if (!move_group_->setJointValueTarget(target_map))
  {
    RCLCPP_ERROR(logger(), "[Planner] setJointValueTarget() returned false for '%s'",
                 target_name.c_str());
    return result;
  }

  move_group_->setPlanningTime(planning_time_);
  move_group_->setNumPlanningAttempts(num_planning_attempts_);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto plan_result = move_group_->plan(plan);

  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    result.error_code = plan_result.val;
    RCLCPP_ERROR(logger(), "[Planner] plan() failed for '%s' (error code: %d)",
                 target_name.c_str(), result.error_code);
    return result;
  }

  RCLCPP_INFO(logger(), "[Planner] plan() succeeded for '%s' (%lu trajectory points).",
               target_name.c_str(), plan.trajectory.joint_trajectory.points.size());

  result.trajectory = plan.trajectory;
  result.success = true;

  if (execute)
  {
    RCLCPP_INFO(logger(), "[Planner] Executing '%s'...", target_name.c_str());
    const auto exec_result = move_group_->execute(plan.trajectory);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      result.error_code = exec_result.val;
      result.success = false;
      RCLCPP_ERROR(logger(), "[Planner] execute() failed for '%s' (error code: %d)",
                    target_name.c_str(), result.error_code);
      return result;
    }
    RCLCPP_INFO(logger(), "[Planner] Execution of '%s' completed successfully.",
                target_name.c_str());
  }
  else
  {
    RCLCPP_INFO(logger(), "[Planner] Plan-only for '%s': trajectory not executed.",
                target_name.c_str());
  }

  return result;
}

PlanResult PlannerUtils::plan_named_pose_target(
    const std::string& target_name,
    const geometry_msgs::msg::PoseStamped& stored_pose,
    const bool execute)
{
  PlanResult result;

  RCLCPP_INFO(logger(), "[Planner] Named pose target: '%s'", target_name.c_str());
  RCLCPP_INFO(logger(), "[Planner]   frame_id: '%s'", stored_pose.header.frame_id.c_str());
  RCLCPP_INFO(logger(), "[Planner]   position: (%.4f, %.4f, %.4f)",
              stored_pose.pose.position.x,
              stored_pose.pose.position.y,
              stored_pose.pose.position.z);
  RCLCPP_INFO(logger(), "[Planner]   quaternion: (%.6f, %.6f, %.6f, %.6f)",
              stored_pose.pose.orientation.x,
              stored_pose.pose.orientation.y,
              stored_pose.pose.orientation.z,
              stored_pose.pose.orientation.w);

  const std::string ee_link = move_group_->getEndEffectorLink();

  geometry_msgs::msg::Pose target_pose = stored_pose.pose;
  if (stored_pose.header.frame_id != task_frame_)
  {
    if (!transform_utils_)
    {
      RCLCPP_ERROR(logger(),
                   "[Planner] transform_utils_ is null — cannot transform pose from '%s' to '%s'",
                   stored_pose.header.frame_id.c_str(), task_frame_.c_str());
      return result;
    }
    RCLCPP_INFO(logger(),
                "[Planner] Transforming pose from '%s' to '%s'",
                stored_pose.header.frame_id.c_str(), task_frame_.c_str());
    try
    {
      geometry_msgs::msg::PoseStamped pose_in = stored_pose;
      pose_in.header.stamp = node_->get_clock()->now();
      const auto transformed = transform_utils_->transform_to_planning_frame(pose_in, task_frame_);
      target_pose = transformed.pose;
    }
    catch (const std::exception& ex)
    {
      RCLCPP_ERROR(logger(),
                   "[Planner] TF transform failed for named pose '%s': %s",
                   target_name.c_str(), ex.what());
      return result;
    }
  }

  move_group_->setStartStateToCurrentState();
  move_group_->setPoseReferenceFrame(task_frame_);
  move_group_->setPlanningTime(planning_time_);
  move_group_->setNumPlanningAttempts(num_planning_attempts_);

  if (!move_group_->setPoseTarget(target_pose, ee_link))
  {
    RCLCPP_ERROR(logger(),
                 "[Planner] setPoseTarget() returned false for '%s' on link '%s'",
                 target_name.c_str(), ee_link.c_str());
    move_group_->clearPoseTargets();
    return result;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto plan_result = move_group_->plan(plan);

  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    result.error_code = plan_result.val;
    RCLCPP_ERROR(logger(),
                 "[Planner] plan() failed for '%s' (error code: %d)",
                 target_name.c_str(), result.error_code);
    move_group_->clearPoseTargets();
    return result;
  }

  RCLCPP_INFO(logger(), "[Planner] plan() succeeded for '%s' (%lu trajectory points).",
               target_name.c_str(), plan.trajectory.joint_trajectory.points.size());

  result.trajectory = plan.trajectory;
  result.success = true;

  if (execute)
  {
    RCLCPP_INFO(logger(), "[Planner] Executing trajectory for '%s'...", target_name.c_str());
    const auto exec_result = move_group_->execute(plan.trajectory);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      result.error_code = exec_result.val;
      result.success = false;
      RCLCPP_ERROR(logger(),
                   "[Planner] execute() failed for '%s' (error code: %d)",
                   target_name.c_str(), result.error_code);
      move_group_->clearPoseTargets();
      return result;
    }
    RCLCPP_INFO(logger(), "[Planner] Execution of '%s' completed successfully.",
                target_name.c_str());
  }
  else
  {
    RCLCPP_INFO(logger(), "[Planner] Plan-only for '%s': trajectory not executed.",
                target_name.c_str());
  }

  move_group_->clearPoseTargets();
  return result;
}

PlanResult PlannerUtils::plan_cartesian_from_poses(
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    const double eef_step,
    const double jump_threshold,
    const std::string& service_name)
{
  PlanResult result;

  RCLCPP_INFO(logger(), "[Cartesian] === %s ===", service_name.c_str());
  RCLCPP_INFO(logger(), "[Cartesian] task_frame: '%s'", task_frame_.c_str());
  RCLCPP_INFO(logger(), "[Cartesian] Waypoints: %zu", waypoints.size());

  move_group_->setStartStateToCurrentState();
  move_group_->setPoseReferenceFrame(task_frame_);

  const geometry_msgs::msg::Pose current_eef = move_group_->getCurrentPose().pose;
  RCLCPP_INFO(logger(), "[Cartesian] Current eef xyz (frame='%s'): (%.4f, %.4f, %.4f)",
               task_frame_.c_str(),
               current_eef.position.x,
               current_eef.position.y,
               current_eef.position.z);

  for (size_t i = 0; i < waypoints.size(); ++i)
  {
    RCLCPP_INFO(logger(), "[Cartesian]   waypoint[%zu] xyz (frame='%s'): (%.4f, %.4f, %.4f)",
                 i, task_frame_.c_str(),
                 waypoints[i].position.x,
                 waypoints[i].position.y,
                 waypoints[i].position.z);
  }

  last_cartesian_fraction_ = move_group_->computeCartesianPath(
      waypoints, eef_step, jump_threshold, result.trajectory,
      /*avoid_collisions=*/true, nullptr);

  RCLCPP_INFO(logger(), "[Cartesian] computeCartesianPath fraction=%.6f (%lu trajectory points).",
               last_cartesian_fraction_,
               result.trajectory.joint_trajectory.points.size());

  result.fraction = last_cartesian_fraction_;
  result.success = (last_cartesian_fraction_ >= CARTESIAN_SUCCESS_THRESHOLD);
  return result;
}

PlanResult PlannerUtils::execute_pose_waypoints_ptp(
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    const std::string& service_name,
    const bool execute)
{
  PlanResult result;

  if (waypoints.empty())
  {
    RCLCPP_ERROR(logger(), "[PTP] %s: no waypoints.", service_name.c_str());
    return result;
  }

  const std::string ee_link = move_group_->getEndEffectorLink();
  move_group_->setPoseReferenceFrame(task_frame_);
  move_group_->setPlanningTime(planning_time_);
  move_group_->setNumPlanningAttempts(num_planning_attempts_);
  move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_);
  move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_);

  RCLCPP_WARN(
      logger(),
      "[PTP] %s: executing %zu pose waypoints with point-to-point planning fallback.",
      service_name.c_str(),
      waypoints.size());

  size_t completed = 0;
  for (size_t i = 0; i < waypoints.size(); ++i)
  {
    move_group_->setStartStateToCurrentState();
    move_group_->clearPoseTargets();

    bool using_position_only = false;
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    const auto plan_position_only = [&]() -> moveit::core::MoveItErrorCode {
      move_group_->setStartStateToCurrentState();
      move_group_->clearPoseTargets();
      const auto& pos = waypoints[i].position;
      RCLCPP_WARN(
          logger(),
          "[PTP] %s: trying position-only fallback at waypoint %zu/%zu "
          "xyz=(%.4f, %.4f, %.4f).",
          service_name.c_str(),
          i,
          waypoints.size(),
          pos.x,
          pos.y,
          pos.z);
      if (!move_group_->setPositionTarget(pos.x, pos.y, pos.z, ee_link))
      {
        RCLCPP_ERROR(
            logger(),
            "[PTP] %s: setPositionTarget failed at waypoint %zu/%zu.",
            service_name.c_str(),
            i,
            waypoints.size());
        return moveit::core::MoveItErrorCode(moveit_msgs::msg::MoveItErrorCodes::FAILURE);
      }
      using_position_only = true;
      return move_group_->plan(plan);
    };

    moveit::core::MoveItErrorCode plan_result(
        moveit_msgs::msg::MoveItErrorCodes::FAILURE);
    if (move_group_->setPoseTarget(waypoints[i], ee_link))
    {
      plan_result = move_group_->plan(plan);
      if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_WARN(
            logger(),
            "[PTP] %s: pose plan failed at waypoint %zu/%zu "
            "(error code: %d); falling back to position-only.",
            service_name.c_str(),
            i,
            waypoints.size(),
            plan_result.val);
        plan_result = plan_position_only();
      }
    }
    else
    {
      RCLCPP_WARN(
          logger(),
          "[PTP] %s: setPoseTarget failed at waypoint %zu/%zu; "
          "falling back to position-only.",
          service_name.c_str(),
          i,
          waypoints.size());
      plan_result = plan_position_only();
    }

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      result.error_code = plan_result.val;
      RCLCPP_ERROR(
          logger(),
          "[PTP] %s: plan failed at waypoint %zu/%zu after pose and "
          "position-only attempts (error code: %d).",
          service_name.c_str(),
          i,
          waypoints.size(),
          result.error_code);
      move_group_->clearPoseTargets();
      result.fraction = static_cast<double>(completed) / static_cast<double>(waypoints.size());
      return result;
    }

    if (execute)
    {
      const auto exec_result = move_group_->execute(plan.trajectory);
      if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
      {
        result.error_code = exec_result.val;
        RCLCPP_ERROR(
            logger(),
            "[PTP] %s: execute failed at waypoint %zu/%zu (error code: %d).",
            service_name.c_str(),
            i,
            waypoints.size(),
            result.error_code);
        move_group_->clearPoseTargets();
        result.fraction = static_cast<double>(completed) / static_cast<double>(waypoints.size());
        return result;
      }
    }

    ++completed;
    RCLCPP_INFO(
        logger(),
        "[PTP] %s: completed waypoint %zu/%zu%s.",
        service_name.c_str(),
        completed,
        waypoints.size(),
        using_position_only ? " (position-only)" : "");
  }

  move_group_->clearPoseTargets();
  result.success = true;
  result.fraction = 1.0;
  return result;
}

PlanResult PlannerUtils::plan_cartesian_named_sequence(
    const std::vector<std::string>& waypoint_names)
{
  PlanResult result;

  if (!waypoint_loader_)
  {
    RCLCPP_ERROR(logger(), "[Cartesian] waypoint_loader_ is null in plan_cartesian_named_sequence.");
    return result;
  }

  RCLCPP_INFO(logger(), "[NAMED] Named sequence: %zu waypoints  (task_frame='%s')",
               waypoint_names.size(), task_frame_.c_str());

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.reserve(waypoint_names.size());

  for (size_t i = 0; i < waypoint_names.size(); ++i)
  {
    const auto* stored_pose = waypoint_loader_->get_cartesian_point(waypoint_names[i]);
    if (!stored_pose)
    {
      RCLCPP_ERROR(logger(), "[NAMED] Unknown cartesian point: '%s'",
                   waypoint_names[i].c_str());
      return result;
    }

    geometry_msgs::msg::Pose waypoint_pose = stored_pose->pose;

    if (stored_pose->header.frame_id != task_frame_)
    {
      RCLCPP_WARN(logger(),
                  "[NAMED] Waypoint '%s' frame_id='%s' != task_frame='%s' — "
                  "YAML poses must already be in '%s'.  Skipping transform.",
                  waypoint_names[i].c_str(),
                  stored_pose->header.frame_id.c_str(),
                  task_frame_.c_str(),
                  task_frame_.c_str());
      waypoint_pose = stored_pose->pose;
    }

    waypoints.push_back(waypoint_pose);

    RCLCPP_INFO(logger(), "[NAMED]   [%zu] '%s' xyz (frame='%s'): (%.4f, %.4f, %.4f)  "
                 "quat=(%.6f, %.6f, %.6f, %.6f)",
                 i, waypoint_names[i].c_str(), task_frame_.c_str(),
                 waypoint_pose.position.x,
                 waypoint_pose.position.y,
                 waypoint_pose.position.z,
                 waypoint_pose.orientation.x,
                 waypoint_pose.orientation.y,
                 waypoint_pose.orientation.z,
                 waypoint_pose.orientation.w);
  }

  RCLCPP_INFO(logger(), "[NAMED] Total waypoints: %zu  (computeCartesianPath starts from current state).",
               waypoints.size());

  move_group_->setStartStateToCurrentState();
  move_group_->setPoseReferenceFrame(task_frame_);

  const geometry_msgs::msg::Pose current_eef = move_group_->getCurrentPose().pose;
  RCLCPP_INFO(logger(), "[NAMED] Current eef xyz (frame='%s'): (%.4f, %.4f, %.4f)  "
               "quat=(%.6f, %.6f, %.6f, %.6f)",
               task_frame_.c_str(),
               current_eef.position.x,
               current_eef.position.y,
               current_eef.position.z,
               current_eef.orientation.x,
               current_eef.orientation.y,
               current_eef.orientation.z,
               current_eef.orientation.w);

  last_cartesian_fraction_ = move_group_->computeCartesianPath(
      waypoints, DEFAULT_EEF_STEP, DEFAULT_JUMP_THRESHOLD, result.trajectory,
      /*avoid_collisions=*/true, nullptr);

  RCLCPP_INFO(logger(), "[NAMED] computeCartesianPath fraction=%.6f  trajectory_points=%lu",
               last_cartesian_fraction_,
               result.trajectory.joint_trajectory.points.size());

  result.fraction = last_cartesian_fraction_;
  result.success = (last_cartesian_fraction_ >= CARTESIAN_SUCCESS_THRESHOLD);

  return result;
}

bool PlannerUtils::execute_trajectory(const moveit_msgs::msg::RobotTrajectory& trajectory)
{
  move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_);
  move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_);

  auto trajectory_to_execute = trajectory;
  auto& joint_trajectory = trajectory_to_execute.joint_trajectory;
  if (!joint_trajectory.points.empty() && !joint_trajectory.joint_names.empty())
  {
    move_group_->setStartStateToCurrentState();
    const auto current_state = move_group_->getCurrentState(1.0);
    if (current_state)
    {
      auto& first_point = joint_trajectory.points.front();
      if (first_point.positions.size() == joint_trajectory.joint_names.size())
      {
        double max_abs_delta = 0.0;
        std::string max_delta_joint;
        for (size_t i = 0; i < joint_trajectory.joint_names.size(); ++i)
        {
          const auto& joint_name = joint_trajectory.joint_names[i];
          const double current_position = current_state->getVariablePosition(joint_name);
          const double delta = std::abs(first_point.positions[i] - current_position);
          if (delta > max_abs_delta)
          {
            max_abs_delta = delta;
            max_delta_joint = joint_name;
          }
          first_point.positions[i] = current_position;
        }
        RCLCPP_INFO(
            logger(),
            "[Planner] Synced trajectory start to current robot state before execute "
            "(max_delta=%.6f rad at '%s').",
            max_abs_delta,
            max_delta_joint.c_str());
      }
      else
      {
        RCLCPP_WARN(
            logger(),
            "[Planner] Cannot sync trajectory start: first point has %zu positions "
            "but trajectory has %zu joint names.",
            first_point.positions.size(),
            joint_trajectory.joint_names.size());
      }
    }
    else
    {
      RCLCPP_WARN(logger(), "[Planner] Current robot state unavailable; executing trajectory as planned.");
    }
  }

  const auto exec_result = move_group_->execute(trajectory_to_execute);
  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(logger(), "[Planner] execute() failed (error code: %d)", exec_result.val);
    return false;
  }
  RCLCPP_INFO(logger(), "[Planner] Trajectory executed successfully.");
  return true;
}

}  // namespace robot_task_executor
