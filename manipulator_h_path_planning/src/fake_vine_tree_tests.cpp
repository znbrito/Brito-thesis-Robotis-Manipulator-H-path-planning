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



int box_id = 0;


void add_available_pose(moveit::planning_interface::MoveGroupInterface &move_group, ros::NodeHandle &node_handle, geometry_msgs::Pose available_pose) {

  box_id++;

  // Adding object to the environment
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  std::string box = "box"+box_id;
  collision_object.id = box;

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.005;
  primitive.dimensions[1] = 0.005;
  primitive.dimensions[2] = 0.005;

  //Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;

  // Create a quaternion so that we can normalize it
  tf::Quaternion aux;
  aux.setW(1);
  aux.setX(0);
  aux.setY(0);
  aux.setZ(0);
  aux.normalize();

  box_pose.orientation.w = aux.getW();
  box_pose.orientation.x = aux.getX();
  box_pose.orientation.y = aux.getY();
  box_pose.orientation.z = aux.getZ();

  box_pose.position.x = available_pose.position.x;
  box_pose.position.y = available_pose.position.y;
  box_pose.position.z = available_pose.position.z;

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
      ROS_INFO("Service called!");
      for (int i = 0; i < (int)srv.response.scene.world.collision_objects.size(); ++i) {
        if (srv.response.scene.world.collision_objects[i].id == box) {
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
    currentACM.entry_names.push_back(box);

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
}



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



  // Creating the target poses
  geometry_msgs::Pose target_pose1;

  // Using a tf quaternion because all the orientations need normalized
  // quaternions and this class can define and normalize quaternions
  tf::Quaternion aux;

  aux.setEuler(0.000006, 0.004333, 1.571072); // Yaw, Pitch, Roll
  aux.normalize();

  target_pose1.orientation.w = aux.getW();
  target_pose1.orientation.x = aux.getX();
  target_pose1.orientation.y = aux.getY();
  target_pose1.orientation.z = aux.getZ();
  ////////////////////////////////////////////////////////////////


  // Set the robot pose targets

  // Pose 1
  target_pose1.position.x = 0.68;
  target_pose1.position.y = 0.56;
  target_pose1.position.z = 0.7;

  // Pose 2
  geometry_msgs::Pose target_pose2 = target_pose1;
  target_pose2.position.x = 0.76;
  target_pose2.position.z = 0.8;

  // Pose 3
  geometry_msgs::Pose target_pose3 = target_pose1;
  target_pose3.position.x = 0.84;
  target_pose3.position.z = 0.7;

  // Pose 4
  geometry_msgs::Pose target_pose4 = target_pose1;
  target_pose4.position.x = 1.1;

  // Pose 5
  geometry_msgs::Pose target_pose5 = target_pose1;
  target_pose5.position.x = 1.3;
  target_pose5.position.z = 0.82;

  // Pose 6
  geometry_msgs::Pose target_pose6 = target_pose1;
  target_pose6.position.x = 1.52;

  // Defining a vector with all poses
  std::vector<geometry_msgs::Pose> poses;
  poses.push_back(target_pose1);
  poses.push_back(target_pose2);
  poses.push_back(target_pose3);
  poses.push_back(target_pose4);
  poses.push_back(target_pose5);
  poses.push_back(target_pose6);



  // Initializating the topic that will publish Husky's commands
  ros::Publisher husky_vel_pub = node_handle.advertise<geometry_msgs::Twist>("husky_velocity_controller/cmd_vel", 1000, true);

  // Defining Husky's linear velocity
  geometry_msgs::Twist msg;
  msg.linear.x = 0.1; // Linear velocity set for Husky

  // Make Husky map the whole vine
  ros::Time endTime = ros::Time::now() + ros::Duration(13);
  while(ros::Time::now() < endTime)
  {
    husky_vel_pub.publish(msg);
    ros::Duration(0.1).sleep();
  }


  // Sleep during 0.5s to stop the robot so that there won't be any slippage when the robots starts to move again
  ros::Duration(0.5).sleep();


  // Now, make Husky come back to its initial position
  msg.linear.x = -0.1;

  endTime = ros::Time::now() + ros::Duration(12);
  while(ros::Time::now() < endTime)
  {
    husky_vel_pub.publish(msg);
    ros::Duration(0.1).sleep();
  }

  // Now set the Husky's velocity to be positive again
  msg.linear.x = 0.1;


  // Create a plan variable and a variable to signal successful or erroneous plans
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = 0;

  // Set the planning time to a maximum of 200ms
  move_group.setPlanningTime(0.2);

  // Go through the entire cycle while
  for (std::vector<geometry_msgs::Pose>::iterator it = poses.begin(); it != poses.end(); ++it) {

    // Add pose in the allowed collision matrix
    add_available_pose(move_group, node_handle, *it);

    // Add pose as a new target and publish an labelled axis on it
    move_group.setPoseTarget(*it);
    std::string label = "Pose "+box_id;
    visual_tools.publishAxisLabeled(*it, label);
    visual_tools.trigger();

    // Slowly move the robot forward if he can't execute the plan
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    while (!success) {
      husky_vel_pub.publish(msg);
      success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }

    // Execute the plan
    move_group.execute(my_plan);

    // Move again the the home pose
    move_group.setNamedTarget("home_pose");
    move_group.move();
  }


  ros::shutdown();
  return 0;
}
