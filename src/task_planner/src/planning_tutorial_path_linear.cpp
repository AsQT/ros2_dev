
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("planning_tutorial");

int main(int argc, char** argv)
{
  /*___________________________ setup ___________________________________________*/
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "planning_tutorial",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  constexpr const char* PLANNING_GROUP = "arm";
  constexpr const char* BASE_FRAME = "world";
  constexpr const char* TIP_LINK = "tcp_link";

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  auto display_publisher =
      node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(
      node, BASE_FRAME, "/rviz_visual_tools", move_group.getRobotModel());
  visual_tools.enableBatchPublishing();
  visual_tools.deleteAllMarkers();

  /*-----------------------------------------------------------------------*/
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.2;

  visual_tools.publishText(text_pose, "Panda Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setMaxVelocityScalingFactor(0.2);
  move_group.setMaxAccelerationScalingFactor(0.2);
  move_group.allowReplanning(true);

  if (!move_group.startStateMonitor(5.0))
  {
    RCLCPP_WARN(LOGGER, "Khong the khoi dong startStateMonitor trong 5s");
  }

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  visual_tools.prompt("Nhan Next de planning pose goal");

  /*_______________________ Pose goal demo _______________________________________________*/

  // step 1: get setStartStateToCurrentState
  move_group.setStartStateToCurrentState();
  geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose(TIP_LINK).pose;

  // step 2: get target_pose
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation = start_pose.orientation;
  target_pose.position.x = 0.30;
  target_pose.position.y = 0.40;
  target_pose.position.z = 0.75;
  // step 3: set target_pose
  move_group.setPoseReferenceFrame(BASE_FRAME);
  move_group.setPoseTarget(target_pose, TIP_LINK);
  // step 4: planning
  moveit::planning_interface::MoveGroupInterface::Plan pose_plan;
  bool pose_ok = static_cast<bool>(move_group.plan(pose_plan));

  if (pose_ok)
  {
    // 1) Hiển thị planned path kiểu tutorial chuẩn của MoveIt
    moveit_msgs::msg::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory_start = pose_plan.start_state;
    display_trajectory.trajectory.push_back(pose_plan.trajectory);
    display_publisher->publish(display_trajectory);

    // 2) Hiển thị thêm đường path marker giống ảnh mẫu
    // Dùng vài điểm trung gian để tạo đường cong nhìn trực quan hơn
    std::vector<geometry_msgs::msg::Pose> viz_points;
    viz_points.push_back(start_pose);

    geometry_msgs::msg::Pose p1 = start_pose;
    p1.position.z += 0.06;
    p1.position.y += 0.08;
    viz_points.push_back(p1);

    geometry_msgs::msg::Pose p2 = p1;
    p2.position.x += 0.10;
    p2.position.y += 0.08;
    viz_points.push_back(p2);

    viz_points.push_back(target_pose);

    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(target_pose, "goal_1");
    visual_tools.publishPath(viz_points, rvt::LIME_GREEN, rvt::SMALL); // hieenr thij path
    visual_tools.publishText(text_pose, "Pose Goal Plan", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    RCLCPP_INFO(LOGGER, "Pose planning thanh cong");
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Pose planning that bai");
  }

  visual_tools.prompt("Nhan Next de execute pose goal");

  if (pose_ok)
  {
    auto exec_code = move_group.execute(pose_plan);
    if (exec_code == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(LOGGER, "Execute pose target thanh cong");
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "MoveGroupInterface::execute() failed or timeout reached");
    }
  }

  visual_tools.prompt("Nhan Next de tao cartesian path");


 /*_______________________ Cartesian path demo _______________________________________________*/
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose cart_start = move_group.getCurrentPose(TIP_LINK).pose;
  waypoints.push_back(cart_start);

  geometry_msgs::msg::Pose w1 = cart_start;
  w1.position.z -= 0.10;
  waypoints.push_back(w1);

  geometry_msgs::msg::Pose w2 = w1;
  w2.position.y += 0.10;
  waypoints.push_back(w2);

  geometry_msgs::msg::Pose w3 = w2;
  w3.position.x += 0.10;
  waypoints.push_back(w3);

  moveit_msgs::msg::RobotTrajectory cartesian_trajectory;
  const double eef_step = 0.01;
  const double jump_threshold = 0.0;
  double fraction =
      move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, cartesian_trajectory);

  RCLCPP_INFO(LOGGER, "Cartesian path fraction = %.3f", fraction);

  if (fraction > 0.99)
  {
    moveit_msgs::msg::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory.push_back(cartesian_trajectory);
    display_publisher->publish(display_trajectory);

    visual_tools.deleteAllMarkers();
    for (std::size_t i = 0; i < waypoints.size(); ++i)
    {
      visual_tools.publishAxisLabeled(waypoints[i], "wp_" + std::to_string(i));
    }
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory = cartesian_trajectory;

    visual_tools.prompt("Nhan Next de execute cartesian path");
    auto exec_code = move_group.execute(cartesian_plan);
    if (exec_code == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(LOGGER, "Execute cartesian path thanh cong");
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Execute cartesian path that bai hoac timeout");
    }
  }
  else
  {
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Cartesian path fraction < 1.0", rvt::YELLOW, rvt::XLARGE);
    visual_tools.trigger();
    RCLCPP_WARN(LOGGER, "Cartesian path khong du fraction de execute");
  }

 /*_______________________ Hết demo _______________________________________________*/
  visual_tools.prompt("Demo xong. Nhan Next de thoat");

  rclcpp::shutdown();
  executor.cancel();
  if (spinner.joinable())
  {
    spinner.join();
  }
  return 0;
}