#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto motion_planning_robot_node = std::make_shared<rclcpp::Node>("robot_move_group",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = rclcpp::get_logger("robot_move_group");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(motion_planning_robot_node);
  std::thread spinner([&executor]() { executor.spin(); });
  
  /*_______ Tạo MoveGroupInterface _________*/
  static const std::string PLANNING_GROUP = "arm";
  const std::string BASE_FRAME = "world";
  const std::string TIP_LINK = "tcp_link";

  moveit::planning_interface::MoveGroupInterface move_group(motion_planning_robot_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  /*_______ Cấu hình cơ bản cho MoveGroupInterface _________*/
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(5);
  move_group.setMaxVelocityScalingFactor(0.2);
  move_group.setMaxAccelerationScalingFactor(0.2);

  //RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  //RCLCPP_INFO(logger, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  const moveit::core::JointModelGroup * joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  /*_______ Tạo MoveItVisualTools _________*/
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(
                                          motion_planning_robot_node, 
                                          BASE_FRAME, 
                                          "move_group_visualization");
  
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(
                text_pose,   "robot move_group_interface", 
                rvt::WHITE,   rvt::XLARGE);
  visual_tools.trigger();
  /*_______ Hiển thị trạng thái hiện tại_________*/
  visual_tools.prompt("Nhan Next trong RVizVisualToolsGui de bat dau demo");

  auto current_state = move_group.getCurrentState(5.0);
  std::vector<double> joint_values;
  current_state->copyJointGroupPositions(joint_model_group, joint_values);
  
  RCLCPP_INFO(logger, "Current joint values:");
  for (size_t i = 0; i < joint_values.size(); ++i) {
    RCLCPP_INFO(logger, "  joint_%zu = %.4f", i + 1, joint_values[i]);
  }
  auto current_pose_msg = move_group.getCurrentPose();
  RCLCPP_INFO(logger,
  "Current pose: frame=%s pos=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f)",
  current_pose_msg.header.frame_id.c_str(),
  current_pose_msg.pose.position.x,
  current_pose_msg.pose.position.y,
  current_pose_msg.pose.position.z,
  current_pose_msg.pose.orientation.x,
  current_pose_msg.pose.orientation.y,
  current_pose_msg.pose.orientation.z,
  current_pose_msg.pose.orientation.w);
  /*======================= planning theo joint target =================*/
  
  std::vector<double> target_joints = joint_values;
  if (target_joints.size() >= 6) {
    target_joints[0] = 0.87;
    target_joints[1] = 0.47;
    target_joints[2] = 0.75;
    target_joints[3] = -0.87;
    target_joints[4] = 0.87; 
    target_joints[5] = 0.5; }

  move_group.setJointValueTarget(target_joints);

  moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
  bool joint_success = (move_group.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (joint_success) {
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(
                  text_pose, "Plan_1:_joint_target", 
                  rvt::GREEN,  rvt::XLARGE);
    visual_tools.publishTrajectoryLine(joint_plan.trajectory, joint_model_group);
    visual_tools.trigger();
    RCLCPP_INFO(logger, "Plan joint target thanh cong");

    visual_tools.prompt("Nhan Next de execute joint target");
    move_group.execute(joint_plan);
  } else {
    RCLCPP_ERROR(logger, "Plan joint target that bai");  }

  /*================= planning theo pose target =================*/
  visual_tools.prompt("Nhan Next de planning pose target");

  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.x = 0.5;
  target_pose.orientation.y = 0.5;
  target_pose.orientation.z = 0.5;
  target_pose.orientation.w = 0.5;
  target_pose.position.x = 0.4;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.45;

  move_group.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan pose_plan;
  bool pose_success = (move_group.plan(pose_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (pose_success) {
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Plan 2: pose target", rvt::GREEN, rvt::XLARGE);
    visual_tools.publishAxisLabeled(target_pose, "pose_goal");
    visual_tools.publishTrajectoryLine(pose_plan.trajectory, joint_model_group);
    visual_tools.trigger();
    RCLCPP_INFO(logger, "Plan pose target thanh cong");

    visual_tools.prompt("Nhan Next de execute pose target");
    move_group.execute(pose_plan);
  } else {
    RCLCPP_ERROR(logger, "Plan pose target that bai");
  }
  move_group.clearPoseTargets();
  /*================= Cartesian path ngắn =================*/
  visual_tools.prompt("Nhan Next de tao cartesian path");
  geometry_msgs::msg::Pose star_pose;
  star_pose.orientation.x = 0.5;
  star_pose.orientation.y = 0.5;
  star_pose.orientation.z = 0.5;
  star_pose.orientation.w = 0.5;
  star_pose.position.x = 0.5;
  star_pose.position.y = 0.0;
  star_pose.position.z = 0.59;
  
  move_group.setPoseTarget(star_pose);
  moveit::planning_interface::MoveGroupInterface::Plan pose_plan_start;
  bool pose_success_satrt = (move_group.plan(pose_plan_start) == moveit::core::MoveItErrorCode::SUCCESS);
  if (pose_success_satrt) {
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(star_pose, "pose_start");
    visual_tools.publishTrajectoryLine(pose_plan_start.trajectory, joint_model_group);
    visual_tools.trigger();
    RCLCPP_INFO(logger, "Plan pose target thanh cong");
    move_group.execute(pose_plan_start);
  } else {
    RCLCPP_ERROR(logger, "Plan pose target that bai");
  }
  move_group.clearPoseTargets();


  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;
  waypoints.push_back(start_pose);

  geometry_msgs::msg::Pose w1 = start_pose;
  w1.position.z -= 0.10;
  waypoints.push_back(w1);

  geometry_msgs::msg::Pose w2 = w1;
  w2.position.y += 0.10;
  waypoints.push_back(w2);

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

  visual_tools.prompt("Demo xong. Nhan Next de thoat");

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}