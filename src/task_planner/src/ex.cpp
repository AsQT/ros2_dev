/*__________ MOGROUP INTERFACE __________________*/
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("move_group_plan_demo");

  // Nhóm lập kế hoạch, ví dụ Panda
  moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");

  // Pose mục tiêu cho đầu công tác
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.28;
  target_pose.position.y = -0.20;
  target_pose.position.z = 0.50;

  // Đặt goal
  move_group.setPoseTarget(target_pose);

  // Kết quả plan
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  // Chỉ planning, chưa execute
  auto result = move_group.plan(plan);

  if (result == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "Plan success");
    RCLCPP_INFO(node->get_logger(), "Planning time: %.3f s", plan.planning_time_);
    RCLCPP_INFO(node->get_logger(), "Trajectory points: %zu",
                plan.trajectory_.joint_trajectory.points.size());

    // Nếu muốn chạy thật:
    // move_group.execute(plan);
    // hoặc move_group.move();
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Plan failed");
  }

  rclcpp::shutdown();
  return 0;
}
/*__________ PLANNING INTERFACE __________________*/

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("planning_interface_demo");

  const std::string PLANNING_GROUP = "panda_arm";

  // 1) Load robot model
  robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
  moveit::core::RobotModelPtr robot_model = robot_model_loader.getModel();

  // 2) Tạo planning scene
  planning_scene::PlanningScenePtr planning_scene(
      new planning_scene::PlanningScene(robot_model));

  moveit::core::RobotState& current_state =
      planning_scene->getCurrentStateNonConst();
  const moveit::core::JointModelGroup* joint_model_group =
      current_state.getJointModelGroup(PLANNING_GROUP);
  current_state.setToDefaultValues(joint_model_group, "ready");

  // 3) Nạp planner plugin
  pluginlib::ClassLoader<planning_interface::PlannerManager> loader(
      "moveit_core", "planning_interface::PlannerManager");

  planning_interface::PlannerManagerPtr planner_instance =
      planning_interface::PlannerManagerPtr(
          loader.createUnmanagedInstance("ompl_interface/OMPLPlanner"));

  planner_instance->initialize(robot_model, node, node->get_namespace());

  // 4) Tạo motion plan request
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "panda_link0";
  goal_pose.pose.orientation.w = 1.0;
  goal_pose.pose.position.x = 0.30;
  goal_pose.pose.position.y = 0.40;
  goal_pose.pose.position.z = 0.75;

  std::vector<double> tol_pos(3, 0.01);
  std::vector<double> tol_ang(3, 0.01);
Excute
  moveit_msgs::msg::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints(
          "panda_link8", goal_pose, tol_pos, tol_ang);

  req.group_name = PLANNING_GROUP;
  req.goal_constraints.push_back(pose_goal);

  // 5) Tạo planning context và solve
  planning_interface::PlanningContextPtr context =
      planner_instance->getPlanningContext(planning_scene, req, res.error_code_);

  context->solve(res);

  if (res.error_code_.val == res.error_code_.SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "Plan success");
    RCLCPP_INFO(node->get_logger(), "Planning time: %.3f s", res.planning_time_);
    RCLCPP_INFO(node->get_logger(), "Trajectory points: %zu",
                res.trajectory_.joint_trajectory.points.size());
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Plan failed");
  }
  // 6) Excute
 if (res.error_code_.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS || !res.trajectory_)
  {
    RCLCPP_ERROR(node->get_logger(), "Planning failed");
    rclcpp::shutdown();
    return 1;
  }

  moveit_msgs::msg::RobotTrajectory traj_msg;
  res.trajectory_->getRobotTrajectoryMsg(traj_msg);

  moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");
  auto exec_result = move_group.execute(traj_msg);

  if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
    RCLCPP_INFO(node->get_logger(), "Execution success");
  else
    RCLCPP_ERROR(node->get_logger(), "Execution failed");

}