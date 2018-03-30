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



int main(int argc, char **argv)
{
  ros::init(argc, argv, "manipulator_h_move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();



  // Setup
  // ^^^^^
  //
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the "JointModelGroup". For this manipulator, the only group created is the "main_group" and
  // compreends all his joints and end effector
  static const std::string PLANNING_GROUP = "main_group";

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
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in Rviz
  visual_tools.loadRemoteControl();

  // Rviz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 0.6; // above the manipulator
  visual_tools.publishText(text_pose, "Manipulator-H MoveGroupInterface tests", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to Rviz for large visualizations
  visual_tools.trigger();



  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Save the initial pose of the robot
  geometry_msgs::Pose initial_pose = move_group.getCurrentPose().pose;

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose1;

  // Using a tf quaternion because all the orientations need normalized
  // quaternions and this class can define and normalize quaternions
  tf::Quaternion aux;

  aux.setW(1);
  aux.setX(0);
  aux.setY(0);
  aux.setZ(0);
  aux.normalize();

  target_pose1.orientation.w = aux.getW();
  target_pose1.orientation.x = aux.getX();
  target_pose1.orientation.y = aux.getY();
  target_pose1.orientation.z = aux.getZ();

  target_pose1.position.x = 0.6;
  target_pose1.position.y = 0;
  target_pose1.position.z = 0.2;
  move_group.setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal planning) %s", success ? "" : "FAILED");



  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  //
  // We can also visualize the plan as a line with markers in Rviz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose goal planning", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("end_link"), joint_model_group, rvt::LIME_GREEN);
  visual_tools.trigger();



  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // Moving to a pose goal is similar to the step above
  // except we now use the move() function. Note that
  // the pose goal we had set earlier is still active
  // and so the robot will try to move to that goal. We will
  // not use that function in this tutorial since it is
  // a blocking function and requires a controller to be active
  // and report success on execution of a trajectory.

  // Uncomment below line when working with a real robot (or Gazebo simulation)
  move_group.move();

  // Wait for the user click on the RVizVisualToolsGui or N if he has the 'Key Tool' selected. Also print a specific message in the terminal
  visual_tools.prompt("Click 'Next' in the RVizVisualToolsGui or N if you have the 'Key Tool' selected");



  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  // To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);


  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = -1.0;  // radians
  move_group.setJointValueTarget(joint_group_positions);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal planning) %s", success ? "" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space goal planning", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("end_link"), joint_model_group, rvt::GREEN);
  visual_tools.trigger();

  // Actually move the robot
  move_group.execute(my_plan);
  // Wait for the user click on the RVizVisualToolsGui or N if he has the 'Key Tool' selected. Also print a specific message in the terminal
  visual_tools.prompt("Click 'Next' in the RVizVisualToolsGui or N if you have the 'Key Tool' selected");



  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // First of all, the path planned  under constraints only works if, for example, the
  // start state satisfies the path constraints. If, for example, the path constraint
  // is to follow a path while mantainging a desired end effector orientation (for
  // example the initial one), the start pose has to satisfie this orientation. So,
  // since the last movement didn't satisfy this orientation constraint, we have to
  // first plan to the same goal postion but now with the correct orientation, so:

  // Set the desired pose as the current pose
  geometry_msgs::Pose start_pose2 = move_group.getCurrentPose().pose;

  // Overwrite the wanted orientation
  aux.setW(1);
  aux.setX(0);
  aux.setY(0);
  aux.setZ(0);
  aux.normalize();

  start_pose2.orientation.w = aux.getW();
  start_pose2.orientation.x = aux.getX();
  start_pose2.orientation.y = aux.getY();
  start_pose2.orientation.z = aux.getZ();

  start_pose2.position.x = 0.2;
  start_pose2.position.y = -0.1;
  start_pose2.position.z = 0.2;

  move_group.setPoseTarget(start_pose2);

  // Move the robot so that it now satisfies the orientation constraint before
  // we start to plan under path constraints
  move_group.move();

  // Path constraints can easily be specified for a link on the robot.
  // Let's specify a path constraint and a pose goal for our group.
  // First define the path constraint.
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "end_link";
  ocm.header.frame_id = "link1"; // "link1" OR "world", NOTE SURE ABOUT THIS! WORKS BOTH WAYS!

  // Re-use the predifined quaternion
  aux.setW(1);
  aux.setX(0);
  aux.setY(0);
  aux.setZ(0);
  aux.normalize();

  ocm.orientation.w = aux.getW();
  ocm.orientation.x = aux.getX();
  ocm.orientation.y = aux.getY();
  ocm.orientation.z = aux.getZ();

  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);

  // Now we will plan to the earlier pose target from the new
  // start state that we have just created.
  move_group.setPoseTarget(target_pose1);

  // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // If needed, increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  //move_group.setPlanningTime(10.0);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints path planning) %s", success ? "" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose2, "start");
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishText(text_pose, "Constrained goal planning", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("end_link"), joint_model_group, rvt::BLUE);
  visual_tools.trigger();
  move_group.execute(my_plan);
  // Wait for the user click on the RVizVisualToolsGui or N if he has the 'Key Tool' selected. Also print a specific message in the terminal
  visual_tools.prompt("Click 'Next' in the RVizVisualToolsGui or N if you have the 'Key Tool' selected");

  // When done with the path constraint be sure to clear it.
  move_group.clearPathConstraints();



  // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  //
  // First of all, let's go back to the position where the cartesian path is going to start
  /*
  start_pose2.position.x += 0.2;

  move_group.setPoseTarget(start_pose2);
  move_group.move();

  // You can plan a cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through. Note that we are starting
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list but adding it can help with visualizations
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose2);

  geometry_msgs::Pose target_pose3 = start_pose2;

  target_pose3.position.z += 0.2;
  waypoints.push_back(target_pose3);  // up 

  target_pose3.position.y += 0.1;
  waypoints.push_back(target_pose3);  // left

  target_pose3.position.z -= 0.05;
  target_pose3.position.y += 0.2;
  waypoints.push_back(target_pose3);  // down and left

  // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
  // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
  // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
  move_group.setMaxVelocityScalingFactor(0.1);

  // We want the cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause                              // WATCH OUT HERE WHEN USING A REAL ROBOT
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 1.0; // Maximum distance allowed between waypoints
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (cartesian path planning) (%.2f%% acheived)", fraction * 100.0);

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Cartesian path planning", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  }
  visual_tools.trigger();
  move_group.move();
  // Wait for the user click on the RVizVisualToolsGui or N if he has the 'Key Tool' selected. Also print a specific message in the terminal
  visual_tools.prompt("Click 'Next' in the RVizVisualToolsGui or N if you have the 'Key Tool' selected");
  */

  /*my_plan.trajectory_ = trajectory;
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (cartesian path planning)%s(%.2f%% acheived)", success ? " " : " FAILED ", fraction * 100.0);
  move_group.execute(my_plan);
  // Wait for the user click on the RVizVisualToolsGui or N if he has the 'Key Tool' selected. Also print a specific message in the terminal
  visual_tools.prompt("Click 'Next' in the RVizVisualToolsGui or N if you have the 'Key Tool' selected");*/



  // Adding/Removing Objects and Attaching/Detaching Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // First let's move the robot to the initial pose of the simulation
  move_group.setPoseTarget(initial_pose);
  move_group.move();


  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.1;

  //Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;

  // Re-use the predifined quaternion
  aux.setW(1);
  aux.setX(0);
  aux.setY(0);
  aux.setZ(0);
  aux.normalize();

  box_pose.orientation.w = aux.getW();
  box_pose.orientation.x = aux.getX();
  box_pose.orientation.y = aux.getY();
  box_pose.orientation.z = aux.getZ();

  box_pose.position.x = 0.55;
  box_pose.position.y = 0;
  box_pose.position.z = 0.45;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Sleep to allow MoveGroup to recieve and process the collision object message
  ros::Duration(1.0).sleep();



  // Now let's add the object to Gazebo
  gazebo_msgs::SpawnModel object;

  // Read the URDF file
  std::ifstream ifss("/home/josebrito/catkin_ws/src/brito_thesis/gazebo_objects/object.urdf");
  std::string line, tudo;
  if (ifss.is_open()) {
    // While loop to add the URDF file code into a string variable
    while ( getline (ifss,line) ) {
        tudo += line;
    }
    ifss.close();
    object.request.model_xml = tudo;
  }
  else {
    ROS_INFO_NAMED("tutorial", "Couldn't open the URDF file!");
  }

  // Set the names of the object and the reference frame
  std::string object_name = "my_box";
  object.request.model_name = object_name;
  object.request.reference_frame = "world";

  // Call the service to spawn the object
  if (gazebo_spawn_model.call(object))
  {
    ROS_INFO("Success, object created");
  }
  else
  {
    ROS_ERROR("Failed to call service");
  }

  // Instead of doing the above steps, you can only launhc the following on a new terminal
  // "rosrun gazebo_ros spawn_model -file /home/josebrito/catkin_ws/src/brito_thesis/gazebo_objects/object.urdf -urdf -model my_box"

  // Show text in Rviz of status
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();



  // Now when we plan a trajectory that it will avoid the obstacle
  move_group.setPoseTarget(target_pose1);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obstacle goal planning", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("end_link"), joint_model_group, rvt::CYAN);
  visual_tools.trigger();
  move_group.execute(my_plan);
  // Wait for the user click on the RVizVisualToolsGui or N if he has the 'Key Tool' selected. Also print a specific message in the terminal
  visual_tools.prompt("Click 'Next' in the RVizVisualToolsGui or N if you have the 'Key Tool' selected");



  // Once again, let's move the robot to the initial pose of the simulation
  move_group.setPoseTarget(initial_pose);
  move_group.move();

  // Now, let's remove the collision object from the world.
  ROS_INFO_NAMED("tutorial", "Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // Show text in Rviz of status
  visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Sleep to give Rviz time to show the object is no longer there
  ros::Duration(1.0).sleep();



  // Remove the object from Gazebo
  gazebo_msgs::DeleteModel model;
  model.request.model_name = object_name;

  // Call the service to delete the object
  if (gazebo_delete_model.call(model))
  {
    ROS_INFO("Success, object deleted");
  }
  else
  {
    ROS_ERROR("Failed to call service");
  }



  // Plan to the same pose but now with the object attached
  move_group.setPoseTarget(target_pose1);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 6 (pose goal planning again) %s", success ? "" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Goal pose planning again", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("end_link"), joint_model_group, rvt::MAGENTA);
  visual_tools.trigger();
  move_group.execute(my_plan);
  // Wait for the user click on the RVizVisualToolsGui or N if he has the 'Key Tool' selected. Also print a specific message in the terminal
  visual_tools.prompt("Click 'Next' in the RVizVisualToolsGui or N if you have the 'Key Tool' selected");



  // Planing to a Robot Pose postion (default position defined in the MoveIt! Setup Assistant)
  //
  // Planning to a previously defined position can be done by invoking the function
  // "setNamedTraget" in a "MoveGroupInterface" object
  move_group.setNamedTarget("home_pose");
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 7 (named target goal planning) %s", success ? "" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Goal pose planning again", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("end_link"), joint_model_group, rvt::PURPLE);
  visual_tools.trigger();
  move_group.execute(my_plan);


  ros::shutdown();
  return 0;
}
