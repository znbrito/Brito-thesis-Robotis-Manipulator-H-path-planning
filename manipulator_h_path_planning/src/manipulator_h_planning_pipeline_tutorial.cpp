/* Author: José Brito */

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>



int main(int argc, char** argv)
{
  ros::init(argc, argv, "manipulator_h_motion_pipeline_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");



  // Start
  // ^^^^^
  //
  // Setting up to start using a planning pipeline is pretty easy.
  // Before we can load the planner, we need two objects:
  // a RobotModel and a PlanningScene.
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

  // We can now setup the PlanningPipeline
  // object, which will use the ROS param server
  // to determine the set of request adapters and the
  // planning plugin to use
  planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));

  // Sleep a little to allow time to startup rviz, etc.
  ros::WallDuration sleep_time(5.0);
  sleep_time.sleep();



  // Pose Goal
  // ^^^^^^^^^
  //
  // We will now create a motion plan request for the main group of the manipulator,
  // specifying the desired pose of the end-effector as input.
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "link1";
  pose.pose.position.x = 0.4;
  pose.pose.position.y = 0.2;
  pose.pose.position.z = 0.1;
  pose.pose.orientation.w = 1.0;

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  // We will create the request as a constraint using a helper function available
  // from the
  // `kinematic_constraints`_
  // package.
  //
  // .. _kinematic_constraints: http://docs.ros.org/indigo/api/moveit_core/html/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132
  req.group_name = "main_group";
  moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("end_link", pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);

  // Now, call the pipeline and check whether planning was successful.
  planning_pipeline->generatePlan(planning_scene, req, res);
  // Check that the planning was successful
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
  display_publisher.publish(display_trajectory);

  sleep_time.sleep();



  // Joint Space Goals
  // ^^^^^^^^^^^^^^^^^
  //
  // First, set the state in the planning scene to the final state of the last plan
  robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
  planning_scene->setCurrentState(response.trajectory_start);
  const robot_model::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("main_group");
  robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

  // Now, setup a joint space goal based on the initial position
  robot_state::RobotState goal_state(robot_model);
  std::vector<double> joint_values(7, 0.0);
  joint_values[0] = -1.0;
  goal_state.setJointGroupPositions(joint_model_group, joint_values);
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

  // Call the pipeline and visualize the trajectory
  planning_pipeline->generatePlan(planning_scene, req, res);
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
  sleep_time.sleep();



  // Using a Planning Request Adapter
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // A planning request adapter allows us to specify a series of operations that
  // should happen either before planning takes place or after the planning
  // has been done on the resultant path

  // First, let's purposefully set the initial state to be outside the
  // joint limits and let the
  // planning request adapter deal with it
  // First, set the state in the planning scene to the final state of the last plan
  robot_state = planning_scene->getCurrentStateNonConst();
  planning_scene->setCurrentState(response.trajectory_start);
  robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

  // Now, set one of the joints slightly outside its upper limit
  const robot_model::JointModel* joint_model = joint_model_group->getJointModel("joint2");
  const robot_model::JointModel::Bounds& joint_bounds = joint_model->getVariableBounds();
  std::vector<double> tmp_values(1, 0.0);
  tmp_values[0] = joint_bounds[0].min_position_ - 0.01;
  robot_state.setJointPositions(joint_model, tmp_values);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal);

  // Call the planner again and visualize the trajectories
  planning_pipeline->generatePlan(planning_scene, req, res);
  if (res.error_code_.val != res.error_code_.SUCCESS) {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }

  // Visualize the trajectory
  ROS_INFO("Visualizing the trajectory");
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  // Now you should see three planned trajectories in series
  display_publisher.publish(display_trajectory);

  sleep_time.sleep();
  ROS_INFO("Done");
  return 0;

}
