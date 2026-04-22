#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <utility>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("panda_checkerboard");

std::vector<geometry_msgs::msg::Pose> makeCheckerboardPoses(
    const geometry_msgs::msg::Pose &center_pose,
    double step)
{
  std::vector<geometry_msgs::msg::Pose> poses;

  // zig-zag:
  // 1 2 3
  // 6 5 4
  // 7 8 9
  std::vector<std::pair<double, double>> offsets = {
      {-step, +step}, {0.0, +step}, {+step, +step},
      {+step, 0.0},   {0.0, 0.0},   {-step, 0.0},
      {-step, -step}, {0.0, -step}, {+step, -step}};

  for (const auto &offset : offsets)
  {
    geometry_msgs::msg::Pose p = center_pose;
    p.position.x += offset.first;
    p.position.y += offset.second;
    poses.push_back(p);
  }

  return poses;
}


bool moveToPose(
    moveit::planning_interface::MoveGroupInterface &move_group,
    moveit_visual_tools::MoveItVisualTools &visual_tools,
    const geometry_msgs::msg::Pose &target_pose)
{
  move_group.setJointValueTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
  bool joint_success = (move_group.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (joint_success) {
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(target_pose, "Point");
    //visual_tools.publishTrajectoryLine(joint_plan.trajectory, joint_model_group);
    visual_tools.trigger();

    move_group.execute(joint_plan);
    return true;
  } else {
  return false; }
}



int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto motion_planning_panda_node = std::make_shared<rclcpp::Node>("panda_move_group",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = rclcpp::get_logger("panda_move_group");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(motion_planning_panda_node);
  std::thread spinner([&executor]() { executor.spin(); });
  
  /*_______ Tạo MoveGroupInterface _________*/
  static const std::string PLANNING_GROUP = "arm";
  const std::string BASE_FRAME = "world";
  const std::string TIP_LINK = "tcp_link";

  moveit::planning_interface::MoveGroupInterface move_group(motion_planning_panda_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  /*_______ Cấu hình cơ bản cho MoveGroupInterface _________*/
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(5);
  move_group.setMaxVelocityScalingFactor(0.2);
  move_group.setMaxAccelerationScalingFactor(0.2);

  const moveit::core::JointModelGroup * joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  /*_______ Tạo MoveItVisualTools _________*/
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(
                                          motion_planning_panda_node, 
                                          BASE_FRAME, 
                                          "move_group_visualization");
  
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(
                text_pose,   "Panda_Checker_board", 
                rvt::WHITE,   rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Nhan Next de bat dau chay point 1");

  /*_________ Tạo checker board ____________________________________-*/
  geometry_msgs::msg::Pose center_pose = move_group.getCurrentPose().pose;
  center_pose.position.z += 0.10;
  constexpr double STEP = 0.05;  // 5 cm
  auto checkerboard_poses = makeCheckerboardPoses(center_pose, STEP);
  /*__________________________________________________________________-*/
  visual_tools.prompt("Nhan Next duy chuyen vao vi tri trung tam");

  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.x = 1.0;
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.0;
  target_pose.position.x = 0.5;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.15;
 
  move_group.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan pose_plan;
  bool pose_success = (move_group.plan(pose_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (pose_success) {
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Move_to_center_check_point", rvt::GREEN, rvt::XLARGE);
    visual_tools.publishAxisLabeled(target_pose, "center");
    visual_tools.publishTrajectoryLine(pose_plan.trajectory, joint_model_group);
    visual_tools.trigger();
    RCLCPP_INFO(logger, "Plan pose target thanh cong");
    //move_group.execute(pose_plan);
  } else {
    RCLCPP_ERROR(logger, "Plan pose target that bai");
  }
  move_group.clearPoseTargets();
  /* Ma trận đểm */
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;
  waypoints.push_back(start_pose);

  double spacing = 0.05;   // 5 cm
  int grid_size = 3;

  for (int i = 0; i < grid_size; ++i)
  {
    for (int j = 0; j < grid_size; ++j)
    {
      geometry_msgs::msg::Pose wp = start_pose;

      // offset quanh tâm (0,0)
      wp.position.x += (i - 1) * spacing;
      wp.position.y += (j - 1) * spacing;

      if(j % 2 == 0)
      {
        wp.position.z += 0.05;
      } else
      {
        wp.position.z -= 0.05;
      }
      waypoints.push_back(wp);
    }
  }


  moveit_msgs::msg::RobotTrajectory cartesian_trajectory;
  const double eef_step = 0.01;
  const double jump_threshold = 0.0;
  double fraction = move_group.computeCartesianPath(
    waypoints, eef_step, jump_threshold, cartesian_trajectory);

  RCLCPP_INFO(logger, "Cartesian path fraction = %.3f", fraction);

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Plan 3: cartesian path", rvt::GREEN,
                           rvt::XLARGE);
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    visual_tools.publishAxisLabeled(waypoints[i], "wp_" + std::to_string(i));
  }
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  visual_tools.trigger();

  if (fraction > 0.9) {
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory = cartesian_trajectory;

    visual_tools.prompt("Nhan Next de execute cartesian path");
    move_group.execute(cartesian_plan);
  } else {
    RCLCPP_WARN(logger, "Cartesian path khong du fraction de execute");
  }





  rclcpp::shutdown();
  if (spinner.joinable()) spinner.join();
  return 0;
}