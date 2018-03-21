/* Author: José Brito */

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "right_arm_kinematics");
  ros::AsyncSpinner spinner(1);
  spinner.start();



  // Start
  // ^^^^^
  // Setting up to start using the RobotModel class is very easy. In
  // general, you will find that most higher-level components will
  // return a shared pointer to the RobotModel. You should always use
  // that when possible. In this example, we will start with such a
  // shared pointer and discuss only the basic API. You can have a
  // look at the actual code API for these classes to get more
  // information about how to use more features provided by these
  // classes.

  // We will start by instantiating a "RobotModelLoader" object,
  // which will look up the robot description on the ROS parameter
  // server and construct a "RobotModelPtr" object for us to use.
  //
  // .. _RobotModelLoader: http://docs.ros.org/indigo/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  // Using the :moveit_core:`RobotModel`, we can construct a
  // :moveit_core:`RobotState` that maintains the configuration
  // of the robot. We will set all joints in the state to their
  // default values. We can then get a
  // :moveit_core:`JointModelGroup`, which represents the robot
  // model for a particular group. In this case, the only group
  // availabe is the one consisting of all the links and joints
  // of the Robotis Manipulator-H
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("main_group");

  const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();



  // Get Joint Values
  // ^^^^^^^^^^^^^^^^
  //
  // We can retreive the current set of joint values stored in the state for the right arm.
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  // Joint Limits
  // ^^^^^^^^^^^^
  //
  // setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
  // Set one joint in the right arm outside its joint limit
  joint_values[1] = 3;
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

  // Check whether any joint is outside its joint limits
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  // Enforce the joint limits for this state and check again, i. e., make the joints go to valid positions
  kinematic_state->enforceBounds();
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));



  // Forward Kinematics
  // ^^^^^^^^^^^^^^^^^^
  //
  // Now, we can compute forward kinematics for a set of random joint
  // values. Note that we would like to find the pose of the
  // "end_link" which is the most distal link in the
  // "main_group" of the robot.
  kinematic_state->setToRandomPositions(joint_model_group);
  const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("end_link");

  // Print end-effector pose. Remember that this is in the model frame
  ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
  ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());



  // Inverse Kinematics
  // ^^^^^^^^^^^^^^^^^^
  //
  // We can now solve inverse kinematics (IK) for the manipulator
  // To solve IK, we will need the following:
  // * The desired pose of the end-effector (by default, this is the last link in the "main_group" chain), which is the end_effector_state that we computed in the step above.
  // * The number of attempts to be made at solving IK: 5
  // * The timeout for each attempt: 0.1 s
  bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 5, 0.1);

  // Now, we can print out the IK solution (if found):
  if (found_ik){
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }



  // Get the Jacobian
  // ^^^^^^^^^^^^^^^^
  //
  // We can also get the Jacobian from the :moveit_core:`RobotState`.
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  kinematic_state->getJacobian(joint_model_group,
                               kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                               reference_point_position, jacobian);

  ROS_INFO_STREAM("Jacobian: " << jacobian);

  ros::shutdown();
  return 0;
}
