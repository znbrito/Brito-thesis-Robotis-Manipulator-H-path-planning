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

#include <moveit_msgs/GetPlanningScene.h>



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



  // First, let's plan to the home position
  move_group.setNamedTarget("home_pose");
  move_group.move();




  // Now, save the actual pose of the robot
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

  target_pose1.position.x = 0;
  target_pose1.position.y = 0.65;
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




  // Adding object to the environment
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.005;
  primitive.dimensions[1] = 0.005;
  primitive.dimensions[2] = 0.005;

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

  box_pose.position.x = target_pose1.position.x;
  box_pose.position.y = target_pose1.position.y;
  box_pose.position.z = target_pose1.position.z;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // Add object into the world
  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

  moveit_msgs::PlanningScene planning_scene_add;
  planning_scene_add.world.collision_objects.push_back(collision_object);
  planning_scene_add.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene_add);




  ros::ServiceClient client = node_handle.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");

  moveit_msgs::GetPlanningScene srv;
  srv.request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;

  // Wait until box is published in RViz
  bool object_in_world = false;
  while(!object_in_world) {
    ROS_INFO("Waiting for box to appear...");
    if (client.call(srv)) {
      for (int i = 0; i < (int)srv.response.scene.world.collision_objects.size(); ++i) {
        if (srv.response.scene.world.collision_objects[i].id == "box") {
          object_in_world = true;
          ROS_INFO("Box has appeared!");
        }
      }
    }
    else {
      ROS_WARN("Failed to call service /get_planning_scene");
    }
  }
  // Box now exists in MoveIt!




  moveit_msgs::PlanningScene currentScene;
  moveit_msgs::PlanningScene newSceneDiff;

  moveit_msgs::GetPlanningScene scene_srv;
  scene_srv.request.components.components = scene_srv.request.components.ALLOWED_COLLISION_MATRIX;

  if(!client.call(scene_srv)) {
    ROS_WARN("Failed to call service /get_planning_scene");
  }
  else {
    //ROS_INFO_STREAM("Initial scene!");
    currentScene = scene_srv.response.scene;
    moveit_msgs::AllowedCollisionMatrix currentACM = currentScene.allowed_collision_matrix;

    //ROS_INFO_STREAM("size of acm_entry_names before " << currentACM.entry_names.size());
    //ROS_INFO_STREAM("size of acm_entry_values before " << currentACM.entry_values.size());
    //ROS_INFO_STREAM("size of acm_entry_values[0].entries before " << currentACM.entry_values[0].enabled.size());


    // Filling the AllowedCollisionMatrix message

    // Add a new name called "box" to the collision matrix
    currentACM.entry_names.push_back("box");

    moveit_msgs::AllowedCollisionEntry entry;
    entry.enabled.resize(currentACM.entry_names.size()); // "currentACM.entry_names" already has the updated size

    // Allow collisions with every link -> entry.enabled[i] = TRUE; and resize the "enabled" vector inside the "AllowedCollisionEntry" variable
    for(int i = 0; i < entry.enabled.size(); i++) {
      entry.enabled[i] = true;
    }

    // Extend the last column of the matrix for the "box" object
    for(int i = 0; i < currentACM.entry_values.size(); i++) {
      currentACM.entry_values[i].enabled.push_back(true);
    }

    // Add new row to allowed collision matrix, for the "box" object that will be added
    currentACM.entry_values.push_back(entry);

    // Update and publish the new planning scene with the only thing that changed: the collision matrix
    newSceneDiff.is_diff = true;
    newSceneDiff.allowed_collision_matrix = currentACM;
    planning_scene_diff_publisher.publish(newSceneDiff);


    //for (int i = 0; i < currentACM.entry_names.size(); i++) {
    //  ROS_INFO_STREAM("Linha " << i << ": " << currentACM.entry_values[i] << "\n");
    //}
    //ROS_INFO_STREAM("\n\n\n Size of entry_values " << currentACM.entry_values.size() << "\n");
    //ROS_INFO_STREAM("Size of entry_names " << currentACM.entry_names.size() << "\n\n\n");
  }



  // Show the updated planning scene
  //if(!client.call(scene_srv)) {
  //  ROS_WARN("Failed to call service /get_planning_scene");
  //}
  //else {
  //  ROS_INFO_STREAM("Modified scene!");
  //  currentScene = scene_srv.response.scene;
  //  moveit_msgs::AllowedCollisionMatrix currentACM = currentScene.allowed_collision_matrix;

  //  ROS_INFO_STREAM("size of acm_entry_names after " << currentACM.entry_names.size());
  //  ROS_INFO_STREAM("size of acm_entry_values after " << currentACM.entry_values.size());
  //  ROS_INFO_STREAM("size of acm_entry_values[0].entries after " << currentACM.entry_values[0].enabled.size());
  //}




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
