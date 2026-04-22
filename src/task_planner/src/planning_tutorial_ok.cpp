#include <pluginlib/class_loader.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/conversions.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_planning_api");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  std::shared_ptr<rclcpp::Node> motion_planning_panda_node =
      rclcpp::Node::make_shared("motion_planning_api", node_options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(motion_planning_panda_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  const std::string PLANNING_GROUP = "arm";
  const std::string BASE_FRAME = "world";
  const std::string TIP_LINK = "tcp_link";

  robot_model_loader::RobotModelLoader robot_model_loader(motion_planning_panda_node, "robot_description");
  const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::vector<std::string> planner_plugin_names;

  if (!motion_planning_panda_node->get_parameter("ompl.planning_plugins", planner_plugin_names))
    RCLCPP_FATAL(LOGGER, "Could not find planner plugin names");

  try {
      planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
          "moveit_core", "planning_interface::PlannerManager"));
    }
  catch (pluginlib::PluginlibException& ex) {
      RCLCPP_FATAL(LOGGER, "Exception while creating planning plugin loader %s", ex.what());
    }

  if (planner_plugin_names.empty()) {
      RCLCPP_ERROR(LOGGER,
                  "No planner plugins defined. Please make sure that the planning_plugins parameter is not empty.");
      return -1;
    }

  const auto& planner_name = planner_plugin_names.at(0);
  try {
      planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_name));
      if (!planner_instance->initialize(robot_model, motion_planning_panda_node,
                                        motion_planning_panda_node->get_namespace()))
        RCLCPP_FATAL(LOGGER, "Could not initialize planner instance");
      RCLCPP_INFO(LOGGER, "Using planning interface '%s'", planner_instance->getDescription().c_str());
    }
  catch (pluginlib::PluginlibException& ex) {
      const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
      std::stringstream ss;
      for (const auto& cls : classes)
        ss << cls << " ";
      RCLCPP_ERROR(LOGGER, "Exception while loading planner '%s': %s\nAvailable plugins: %s", planner_name.c_str(),
                  ex.what(), ss.str().c_str());
    }
  moveit::planning_interface::MoveGroupInterface 
          move_group(motion_planning_panda_node, PLANNING_GROUP);
 
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(motion_planning_panda_node, 
                                                      BASE_FRAME,
                                                      "move_group_visualization", 
                                                      move_group.getRobotModel());
  visual_tools.enableBatchPublishing();
  visual_tools.deleteAllMarkers();  
  visual_tools.trigger();
  /*________________________________________*/
  /*---------- Setup Input DATA -----------------------*/  


  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::msg::PoseStamped pose;
  /*         Display          */
  pose.header.frame_id = BASE_FRAME;
  visual_tools.loadRemoteControl();
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText( text_pose, "Motion_Planning", 
                            rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  visual_tools.trigger();

  /*============================ QUY TRÌNH PLANNING VỚI GOAL POSE ======================= */
  
  /*---------- POSE 1 -----------------------*/
  pose.pose.position.x = 0.3;
  pose.pose.position.y = 0.4;
  pose.pose.position.z = 0.75;
  pose.pose.orientation.w = 1.0;
  /*---------- CONTRAIN_1--------------------*/
  std::vector<double> tolerance_pose(3, 0.01);// tolerance :sai số cho phép
  std::vector<double> tolerance_angle(3, 0.01); 
  /*---------- Xác định goal --------------------*/
  moveit_msgs::msg::Constraints 
    pose_goal = 
      kinematic_constraints::constructGoalConstraints(
                                        TIP_LINK,       
                                        pose, 
                                        tolerance_pose, 
                                        tolerance_angle);
  /*------- Đưa goal vào request ----------------*/
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal); 

  /*------- Khai báo goup và workspace --------------*/
  req.group_name = PLANNING_GROUP;
  req.workspace_parameters.min_corner.x   = 
    req.workspace_parameters.min_corner.y =
    req.workspace_parameters.min_corner.z = -5.0;
  req.workspace_parameters.max_corner.x   = 
    req.workspace_parameters.max_corner.y =
    req.workspace_parameters.max_corner.z = 5.0;
  
  planning_scene->getCurrentStateNonConst().update();
  req.start_state = moveit_msgs::msg::RobotState();
  moveit::core::robotStateToRobotStateMsg(planning_scene->getCurrentState(), req.start_state);
  req.allowed_planning_time = 5.0;
  /*---------- Tạo planning context -----------------------*/
  planning_interface::PlanningContextPtr 
    context = planner_instance->getPlanningContext(
                                  planning_scene, // scene hiện tại
                                  req,            // request hiện tại
                                  res.error_code);// planner hiên tại
  /*---------- CHECH_CONTEXT_1 ------------------*/
  if (!context) {
    RCLCPP_ERROR(LOGGER, "Failed to create planning context");
    return -1;  }
  /*---------- SOLVE_RESPHONE_1------------------*/
  context->solve(res);

  if (res.error_code.val != res.error_code.SUCCESS) {
    RCLCPP_ERROR(
      LOGGER, "Could not compute plan successfully");
    return -1;  }

  /*---------- DISPLAY_1------------------*/
  std::shared_ptr<rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>> 
    display_publisher =
      motion_planning_panda_node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
                                                                      "/display_planned_path", 1);
  /*---------- Chuyển request sang message hiển thị ---------------*/
  moveit_msgs::msg::MotionPlanResponse response;
  res.getMessage(response);
  /*---------- Hiển thị quỹ đạo ---------------*/
  moveit_msgs::msg::DisplayTrajectory display_trajectory;
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  visual_tools.publishTrajectoryLine(
                        display_trajectory.trajectory.back(), 
                        joint_model_group);
  visual_tools.trigger();
  display_publisher->publish(display_trajectory); // Publish quỹ đạo

  robot_state->setJointGroupPositions(
                        joint_model_group, 
                        response.trajectory.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());
  visual_tools.publishAxisLabeled(pose.pose, "goal_1");
  visual_tools.publishText( text_pose, "Pose_Goal_(1)", 
                            rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue Pose 2");

  /*=================== QUY TRÌNH PLANNING VỚI JOINT SPASE GOAL ======================= */
 
  /*---------- POSE 2 -----------------------*/
  moveit::core::RobotState goal_state(robot_model);
  std::vector<double> joint_values = { 1.0, 1.7, 0.7, -1.5, -0.7, 2.0, 0.2 };
  goal_state.setJointGroupPositions(joint_model_group, joint_values);
  /*---------- Xác định goal --------------------*/
  // Không cần nạp giới hạn sai số cho phép
  moveit_msgs::msg::Constraints 
    joint_goal = kinematic_constraints::constructGoalConstraints(
                                          goal_state, 
                                          joint_model_group);
  /*------- Đưa goal vào request ----------------*/
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);
  planning_scene->getCurrentStateNonConst().update();
  req.start_state = moveit_msgs::msg::RobotState();
  moveit::core::robotStateToRobotStateMsg(planning_scene->getCurrentState(), req.start_state);
  req.allowed_planning_time = 5.0;
  /* tạo context , không cần kiểm tra */
  context = planner_instance->getPlanningContext(planning_scene, req, res.error_code);
  /* thực hiện lời giải */
  context->solve(res);

  if (res.error_code.val != res.error_code.SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
    return -1;
  }

  res.getMessage(response);

  display_trajectory.trajectory.push_back(response.trajectory);
  visual_tools.publishTrajectoryLine(
                  display_trajectory.trajectory.back(), 
                  joint_model_group);
  visual_tools.trigger();
  display_publisher->publish(display_trajectory);
  robot_state->setJointGroupPositions(
                  joint_model_group, 
                  response.trajectory.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());

  visual_tools.publishAxisLabeled(pose.pose, "goal_2");
  visual_tools.publishText( text_pose, "Joint_Space_Goal_(2)", 
                            rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue Pose3");

  /*================= QUY TRÌNH PLANNING VỚI PATH CONTRAINTS ======================= */

  pose.pose.position.x = 0.30;
  pose.pose.position.y = -0.25;
  pose.pose.position.z = 0.65;
  pose.pose.orientation.w = 1.0;

  moveit_msgs::msg::Constraints pose_goal_3 =
      kinematic_constraints::constructGoalConstraints(
                              TIP_LINK, 
                              pose, 
                              tolerance_pose, 
                              tolerance_angle);

  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal_3);

  geometry_msgs::msg::QuaternionStamped quaternion; // tạo quaternion định hướng
  quaternion.header.frame_id = BASE_FRAME; // hướng theo Base_frame
  req.path_constraints = // gán path vào request
    kinematic_constraints::constructGoalConstraints(
                            TIP_LINK, 
                            quaternion);
  
  req.workspace_parameters.min_corner.x   = 
    req.workspace_parameters.min_corner.y =
    req.workspace_parameters.min_corner.z = -2.0;
  req.workspace_parameters.max_corner.x   = 
    req.workspace_parameters.max_corner.y =
    req.workspace_parameters.max_corner.z = 2.0;

  planning_scene->getCurrentStateNonConst().update();
  req.start_state = moveit_msgs::msg::RobotState();
  moveit::core::robotStateToRobotStateMsg(planning_scene->getCurrentState(), req.start_state);
  req.allowed_planning_time = 5.0;

  context = planner_instance->getPlanningContext(planning_scene, req, res.error_code);

  context->solve(res);

  res.getMessage(response);

  display_trajectory.trajectory.push_back(response.trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();
  display_publisher->publish(display_trajectory);
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());

  visual_tools.publishAxisLabeled(pose.pose, "goal_3");
  visual_tools.publishText( text_pose, "Orientation_Constrained_Motion_Plan_(3)", 
                            rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to exit the demo");
  planner_instance.reset();
  rclcpp::shutdown();
  return 0;
}
