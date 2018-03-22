/* Author: José Brito */

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <boost/scoped_ptr.hpp>



int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");



  // Start
  // ^^^^^
  //
  // Setting up to start using a planner is pretty easy. Planners are
  // setup as plugins in MoveIt! and you can use the ROS pluginlib
  // interface to load any planner that you want to use. Before we
  // can load the planner, we need two objects: a RobotModel
  // and a PlanningScene.
  // We will start by instantiating a
  // `RobotModelLoader`_
  // object, which will look up
  // the robot description on the ROS parameter server and construct a
  // :moveit_core:`RobotModel` for us to use.
  //
  // .. _RobotModelLoader: http://docs.ros.org/indigo/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  // Using the :moveit_core:`RobotModel`, we can construct a
  // :planning_scene:`PlanningScene` that maintains the state of
  // the world (including the robot).
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  // We will now construct a loader to load a planner, by name.
  // Note that we are using the ROS pluginlib library here.
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

  // We will get the name of planning plugin we want to load
  // from the ROS param server, and then load the planner
  // making sure to catch all exceptions.
  if (!node_handle.getParam("planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                         << "Available plugins: " << ss.str());
  }

  // Sleep a little to allow time to startup rviz, etc.
  ros::WallDuration sleep_time(5.0);
  sleep_time.sleep();



  // Pose Goal
  // ^^^^^^^^^
  //
  // We will now create a motion plan request for the main group of the manipulator
  // specifying the desired pose of the end-effector as input.
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "link1";
  pose.pose.position.x = 0.6;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  // We will create the request as a constraint using a helper function available
  // from the `kinematic_constraints` package.
  //
  // .. _kinematic_constraints: http://docs.ros.org/indigo/api/moveit_core/html/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132
  req.group_name = "main_group";
  moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("end_link", pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);

  // We now construct a planning context that encapsulate the scene,
  // the request and the response. We call the planner using this
  // planning context
  planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);
  if (res.error_code_.val != res.error_code_.SUCCESS) {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }



  // Visualize the result
  // ^^^^^^^^^^^^^^^^^^^^
  //
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  // Visualize the trajectory
  ROS_INFO("Visualizing the trajectory");
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  // Now we should see the planned trajectory
  display_publisher.publish(display_trajectory);

  sleep_time.sleep();



  // Joint Space Goals
  // ^^^^^^^^^^^^^^^^^
  //
  // First, set the state in the planning scene to the final state of the last plan
  robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
  planning_scene->setCurrentState(response.trajectory_start);
  const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("main_group");
  robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

  // Now, setup a joint space goal

  // Get the initial state of the robot
  robot_state::RobotState goal_state(robot_model);
  std::vector<double> joint_values(7, 0.0);
  joint_values[0] = -1.0;
  goal_state.setJointGroupPositions(joint_model_group, joint_values);
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

  // Call the planner and visualize the trajectory
  // Re-construct the planning context
  context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  // Call the Planner
  context->solve(res);
  // Check that the planning was successful
  if (res.error_code_.val != res.error_code_.SUCCESS) {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  // Visualize the trajectory
  ROS_INFO("Visualizing the trajectory");
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);

  // Now you should see two planned trajectories in series
  display_publisher.publish(display_trajectory);



  // We will add more goals. But first, set the state in the
  // planning scene to the final state of the last plan
  robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

  // Now, we go back to the first goal
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal);
  context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);
  res.getMessage(response);
  display_trajectory.trajectory.push_back(response.trajectory);
  // Now we should see three planned trajectories in series
  display_publisher.publish(display_trajectory);



  // Adding Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's add a new pose goal again. This time we will also add a path constraint to the motion.
  // Let's create a new pose goal
  pose.pose.position.x = 0.3;
  pose.pose.position.y = 0.3;
  pose.pose.position.z = 0.3;
  moveit_msgs::Constraints pose_goal_2 = kinematic_constraints::constructGoalConstraints("end_link", pose, tolerance_pose, tolerance_angle);
  // First, set the state in the planning scene to the final state of the last plan
  robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  // Now, let's try to move to this new pose goal
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal_2);

  // But let's impose a path constraint on the motion.
  // Here, we are asking for the end-effector to stay level
  geometry_msgs::QuaternionStamped quaternion;
  quaternion.header.frame_id = "end_link";
  quaternion.quaternion.w = 1.0;
  req.path_constraints = kinematic_constraints::constructGoalConstraints("end_link", quaternion);

  // Imposing path constraints requires the planner to reason in the space of possible positions of the end-effector
  // (the workspace of the robot)
  // because of this, we need to specify a bound for the allowed planning volume as well;
  // Note: a default bound is automatically filled by the WorkspaceBounds request adapter (part of the OMPL pipeline,
  // but that is not being used in this example).
  // We use a bound that definitely includes the reachable space for the arm. This is fine because sampling is not done
  // in this volume
  // when planning for the arm; the bounds are only used to determine if the sampled configurations are valid.
  req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
      req.workspace_parameters.min_corner.z = -1.0;
  req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
      req.workspace_parameters.max_corner.z = 1.0;

  // Call the planner and visualize all the plans created so far.
  context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);
  res.getMessage(response);
  display_trajectory.trajectory.push_back(response.trajectory);
  // Now we should see four planned trajectories in series
  display_publisher.publish(display_trajectory);

  sleep_time.sleep();
  ROS_INFO("Done");
  planner_instance.reset();

  return 0;
}
