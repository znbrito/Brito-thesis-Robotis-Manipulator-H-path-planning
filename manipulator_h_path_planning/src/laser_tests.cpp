/* Author: José Brito */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf/tf.h>

#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>

#include <fstream>
#include <string>

#include <geometric_shapes/shape_operations.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "husky_manipulator_h_move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();



  // Setup
  static const std::string PLANNING_GROUP = "arm_group";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);



  // Visualization
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("odom");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.2; // above the manipulator
  visual_tools.publishText(text_pose, "Environment tests with the manipulator", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();  



  // First, save the actual pose of the robot
  geometry_msgs::Pose actual_pose = move_group.getCurrentPose().pose;

  // Create a target pose goal
  geometry_msgs::Pose target_pose1;

  // Using a tf quaternion because all the orientations need normalized
  // quaternions and this class can define and normalize quaternions
  tf::Quaternion aux;

  aux.setEuler(0.000006,0.004333, 1.571072); // Yaw, Pitch, Roll
  aux.normalize();

  target_pose1.orientation.w = aux.getW();
  target_pose1.orientation.x = aux.getX();
  target_pose1.orientation.y = aux.getY();
  target_pose1.orientation.z = aux.getZ();

  target_pose1.position.x = -0.2;
  target_pose1.position.y = 0.7;
  target_pose1.position.z = 0.7;
  move_group.setPoseTarget(target_pose1);

  // Print start and pose positions
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishAxisLabeled(actual_pose, "start");
  visual_tools.trigger();

  // Wait for the user click on the RVizVisualToolsGui or N if he has the 'Key Tool' selected. Also print a specific message in the terminal
  visual_tools.prompt("Click 'Next' in the RVizVisualToolsGui or N if you have the 'Key Tool' selected");



  // When the robot is positioned in order to get to the goal pose, update the end effector actual pose and do path planning
  actual_pose = move_group.getCurrentPose().pose;

  // Create a plan variable
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Plan %s", success ? "SUCCEEDED" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Moving manipulator to predefined position", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishAxisLabeled(actual_pose, "start");
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("end_link"), joint_model_group, rvt::PURPLE);
  visual_tools.trigger();
  move_group.execute(my_plan);

  // Wait for the user click on the RVizVisualToolsGui or N if he has the 'Key Tool' selected. Also print a specific message in the terminal
  visual_tools.prompt("Click 'Next' in the RVizVisualToolsGui or N if you have the 'Key Tool' selected");



  // After path planning, print the goal position
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(target_pose1, "goal");

  // Wait for the user click on the RVizVisualToolsGui or N if he has the 'Key Tool' selected. Also print a specific message in the terminal
  visual_tools.prompt("Click 'Next' in the RVizVisualToolsGui or N if you have the 'Key Tool' selected");



  // After mapping the tree, put the manipulator close to the new position and do path planning again to show that it is not possible
  actual_pose = move_group.getCurrentPose().pose;

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Plan %s", success ? "SUCCEEDED" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Moving manipulator to predefined position", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishAxisLabeled(actual_pose, "start");
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("end_link"), joint_model_group, rvt::PURPLE);
  visual_tools.trigger();
  move_group.execute(my_plan);


  ros::shutdown();
  return 0;
}
