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
  // ^^^^^
  //
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the "JointModelGroup". For this manipulator, the only group created is the "arm_group" and
  // compreends all his joints and end effector
  static const std::string PLANNING_GROUP = "arm_group";

  // The "moveit::planning_interface::MoveGroupInterface" class can be easily
  // setup using just the name of the planning group you would like to control and plan for
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // To add and remove collision objects in our "virtual world" scene, we use
  // the "moveit::planning_interface::PlanningSceneInterface" class
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance
  const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Create the client that will call the gazebo service to spawn objects
  ros::ServiceClient gazebo_spawn_model = node_handle.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");

  // Create another client that will call the gazebo service to delete this object
  ros::ServiceClient gazebo_delete_model = node_handle.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");


  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in Rviz. It is important to define the frame in which the objects
  // are going to be visualized
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link"); // OR "odom"
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in Rviz
  visual_tools.loadRemoteControl();

  // Rviz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.2; // above the manipulator
  visual_tools.publishText(text_pose, "Environment tests with the manipulator", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to Rviz for large visualizations
  visual_tools.trigger();



  // Save the initial pose of the robot
  geometry_msgs::Pose initial_pose = move_group.getCurrentPose().pose;



  // Create a plan variable
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // Move the manipulator to its home position
  move_group.setNamedTarget("home_pose");
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (home position planning) %s", success ? "" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Moving manipulator to home position", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("end_link"), joint_model_group, rvt::PURPLE);
  visual_tools.trigger();
  move_group.execute(my_plan);
  // Wait for the user click on the RVizVisualToolsGui or N if he has the 'Key Tool' selected. Also print a specific message in the terminal
  visual_tools.prompt("Click 'Next' in the RVizVisualToolsGui or N if you have the 'Key Tool' selected");



  // Clear all markers
  visual_tools.deleteAllMarkers();

  // Add a oak tree to the planning scene
  moveit_msgs::CollisionObject oak_tree;
  oak_tree.header.frame_id = move_group.getPlanningFrame();
  oak_tree.id = "oak_tree";
  shapes::Mesh* m = shapes::createMeshFromResource("package://manipulator_h_path_planning/meshes/oak_tree/meshes/oak_tree.dae");
  shape_msgs::Mesh oak_tree_mesh;
  shapes::ShapeMsg oak_tree_mesh_msg;
  shapes::constructMsgFromShape(m, oak_tree_mesh_msg);
  oak_tree_mesh = boost::get<shape_msgs::Mesh>(oak_tree_mesh_msg);
  oak_tree.meshes.resize(1);
  oak_tree.meshes[0] = oak_tree_mesh;
  oak_tree.mesh_poses.resize(1);
  oak_tree.mesh_poses[0].position.x = 0;
  oak_tree.mesh_poses[0].position.y = 1;
  oak_tree.mesh_poses[0].position.z = 0;
  oak_tree.mesh_poses[0].orientation.w= 1.0;
  oak_tree.mesh_poses[0].orientation.x= 0.0;
  oak_tree.mesh_poses[0].orientation.y= 0.0;
  oak_tree.mesh_poses[0].orientation.z= 0.0;

  oak_tree.meshes.push_back(oak_tree_mesh);
  oak_tree.mesh_poses.push_back(oak_tree.mesh_poses[0]);
  oak_tree.operation = oak_tree.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(oak_tree);

  // Now, let's add the collision object into the world
  ROS_INFO("Added an oak tree object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);


  // Now, to add/spawn the object to/in Gazebo, run the following in a terminal
  // $rosrun gazebo_ros spawn_model -file /home/josebrito/catkin_ws/src/brito_thesis/manipulator_h_path_planning/meshes/oak_tree/model.sdf -sdf -model oak_tree -y 2

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Oak tree added to simulation", rvt::WHITE, rvt::XLARGE);
  // Wait for the user click on the RVizVisualToolsGui or N if he has the 'Key Tool' selected. Also print a specific message in the terminal
  visual_tools.prompt("Click 'Next' in the RVizVisualToolsGui or N if you have the 'Key Tool' selected");



  // Now let's plan to an impossible position
  geometry_msgs::Pose target_pose1;

  // Using a tf quaternion because all the orientations need normalized
  // quaternions and this class can define and normalize quaternions
  tf::Quaternion aux;

  aux.setW(initial_pose.orientation.w);
  aux.setX(initial_pose.orientation.x);
  aux.setY(initial_pose.orientation.y);
  aux.setZ(initial_pose.orientation.z);
  aux.normalize();

  target_pose1.orientation.w = aux.getW();
  target_pose1.orientation.x = aux.getX();
  target_pose1.orientation.y = aux.getY();
  target_pose1.orientation.z = aux.getZ();

  target_pose1.position.x = initial_pose.position.x - 0.03;
  target_pose1.position.y = initial_pose.position.y + 0.1;
  target_pose1.position.z = initial_pose.position.z;
  move_group.setPoseTarget(target_pose1);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Pose goal planning is impossible! It ) %s", success ? "" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Planning to an impossible goal position", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("end_link"), joint_model_group, rvt::PURPLE);
  visual_tools.trigger();
  move_group.execute(my_plan);
  // Wait for the user click on the RVizVisualToolsGui or N if he has the 'Key Tool' selected. Also print a specific message in the terminal
  visual_tools.prompt("Click 'Next' in the RVizVisualToolsGui or N if you have the 'Key Tool' selected");

  // The manipulator won't move because the position is inside the tree!



  // Now let's move it to a position that is the most close as possible from the tree's log
  // First define a position
  geometry_msgs::Pose target_pose2 = target_pose1;

  //Let's alter one parameter to make the position possible
  target_pose2.position.y -= 0.04;
  move_group.setPoseTarget(target_pose2);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal planning) %s", success ? "" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Pose goal planning", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("end_link"), joint_model_group, rvt::PURPLE);
  visual_tools.trigger();
  move_group.execute(my_plan);
  // Wait for the user click on the RVizVisualToolsGui or N if he has the 'Key Tool' selected. Also print a specific message in the terminal
  visual_tools.prompt("Click 'Next' in the RVizVisualToolsGui or N if you have the 'Key Tool' selected");



  // Now, let's remove the collision object from the world.
  std::vector<std::string> object_ids;
  object_ids.push_back(oak_tree.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Deleted the oak tree from the world", rvt::WHITE, rvt::XLARGE);
  ROS_INFO_NAMED("tutorial", "Removed the oak tree from the world");
  // Wait for the user click on the RVizVisualToolsGui or N if he has the 'Key Tool' selected. Also print a specific message in the terminal
  visual_tools.prompt("Click 'Next' in the RVizVisualToolsGui or N if you have the 'Key Tool' selected");


  // Manually remove the object from Gazebo!!!!!!!!!!


  // And now let's plan again to the first pose because now it will be possible to reach that pose!
  move_group.setPoseTarget(target_pose1);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (pose goal planning again) %s", success ? "" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Pose goal planning again", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("end_link"), joint_model_group, rvt::CYAN);
  visual_tools.trigger();
  move_group.execute(my_plan);
  // Wait for the user click on the RVizVisualToolsGui or N if he has the 'Key Tool' selected. Also print a specific message in the terminal
  visual_tools.prompt("Click 'Next' in the RVizVisualToolsGui or N if you have the 'Key Tool' selected");


  ros::shutdown();
  return 0;
}
