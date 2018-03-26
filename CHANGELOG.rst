^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package brito_thesis
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.1 (2017-03-26 @ 2:42 PM)
-----------
* Modified the controller type used by MoveIt! in the RObotis Manipulator-H to make it possible for the Husky robot to move without lifting up his back/front wheels, depending on the movements done. 

* After the changes I noticed something on Robotis Manipulator-H. His initial position is now a little leaning forward. THIS MAY BE ALTERED IN THE FUTURE!!

* Try to move the robot by clicking on the keyboard while selecting the second terminal, after running the following in 2 different terminals:
 - FIRST WINDOW: roslaunch husky_gazebo husky_empty_world.launch manipulator_h_enabled:=true
 - SECOND WINDOW: rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=husky_velocity_controller/cmd_vel


0.3.0 (2017-03-26 @ 12:20 AM)
-----------
* Modified the packages "manipulator_h_description" and "manipulator_h_gazebo" in order to be able to run Husky with Manipulator-H on top of it. Modified files:
 - manipulator_h_description/urdf/manipulator_h.xacro
 - manipulator_h_description/urdf/manipulator_h.gazebo

* Added the following Husky packages from the INDIGO devel:
 - husky/
  - husky_control;
  - husky_description;
  - husky_msgs;
  - husky_navigation;
  - husky_ur5_moveit_config;

Downloaded from "https://github.com/husky/husky.git";
Also:
 - husky_simulator/
  - husky_gazebo;
  - husky_simulator;

Downloaded from "https://github.com/husky/husky_simulator.git";
And finally:
 - husky_desktop/
  - husky_desktop;
  - husky_viz;

Downloaded from "https://github.com/husky/husky_desktop.git".
Modified files: 
 - husky_gazebo/launch/husky_empty_world.launch
 - husky_gazebo/launch/spawn_husky.launch
 - husky_gazebo/urdf/description.gazebo.xacro
 - husky_description/urdf/husky.urdf.xacro
 - husky_gazebo/urdf/husky.gazebo.xacro

* Right now it is only possible to visualize the husky + the plugin in Gazebo. In order to run the simulation, run the following in the terminal:
 - HUSKY: $roslaunch husky_gazebo husky_empty_world.launch 
 - HUSKY + MANIPULATOR-H: $roslaunch husky_gazebo husky_empty_world.launch manipulator_h_enabled:=true
 - HUSKY + UR5: $roslaunch husky_gazebo husky_empty_world.launch ur5_enabled:=true
 - HUSKY + KINECT: $roslaunch husky_gazebo husky_empty_world.launch kinect_enabled:=true
 - HUSKY + LASER: $roslaunch husky_gazebo husky_empty_world.launch laser_enabled:=true


0.2.7 (2017-03-22 @ 2:55 PM)
-----------
* Added motion pipeline tutorial from MoveIt!, addapted to the Robotis Manipulator-H

* In order to run the simulation, run the following in the terminal:
 - FIRST WINDOW: roslaunch manipulator_h_path_planning manipulator_h_planning_pipeline_tutorial.launch


0.2.6 (2017-03-22 @ 2:14 PM)
-----------
* Added motion planners tutorial from MoveIt!, addapted to the Robotis Manipulator-H

* In order to run the simulation, run the following in the terminal:
 - FIRST WINDOW: roslaunch manipulator_h_path_planning manipulator_h_motion_planning_api_tutorial.launch


0.2.5 (2017-03-22 @ 11:24 AM)
-----------
* Added ROS API planning scene from MoveIt!, addapted to the Robotis Manipulator-H

* In order to run the simulation, run the following in the terminal:
 - FIRST WINDOW: roslaunch manipulator_h_path_planning manipulator_h_planning_scene_ros_api_tutorial.launch


0.2.4 (2017-03-22 @ 10:38 AM)
-----------
* Added planning scene tutorial from MoveIt!, addapted to the Robotis Manipulator-H

* In order to run the simulation, run the following in the terminal:
 - FIRST WINDOW: roslaunch manipulator_h_path_planning manipulator_h_planning_scene_tutorial.launch 


0.2.3 (2017-03-21 @ 7:35 PM)
-----------
* Added the kinematic model tutorial from MoveIt!, addapted to the Robotis Manipulator-H

* In order to run the simulation, run the following in the terminal:
 - FIRST WINDOW: roslaunch manipulator_h_path_planning manipulator_h_kinematic_model_tutorial.launch 


0.2.2 (2017-03-21 @ 6:46 PM)
-----------
* Programm is now fully functional. Removed the attach and dettach functions. Inserted 2 services, one to add and another to remove gazebo objects, so the object can be visualized both in Gazebo and RViz

* In order to run the simulation, run the following in the terminal:
 - FIRST WINDOW: roslaunch manipulator_h_gazebo manipulator_h_gazebo.launch
 - SECOND WINDOW: roslaunch manipulator_h_moveit_config manipulator_h_planning_execution.launch 
 - THIRD WINDOW: roslaunch manipulator_h_path_planning manipulator_h_move_group_interface_tutorial.launch


0.2.1 (2017-03-21 @ 11:30 AM)
-----------
* Modified the configurations in "manipulator_h_moveit_config" package because the planned paths programmed were being executed in the Gazebo simulator but the joint values weren't being updated. Problem is now solved. Programmed examples for goal pose planning, joint state goal planning and path planning under path constraints is now fully operable. Note that in RViz, the goal state is only updated when the path is planned and executed in the RViz GUI.

* In order to run the simulation, run the following in the terminal:
 - FIRST WINDOW: roslaunch manipulator_h_gazebo manipulator_h_gazebo.launch
 - SECOND WINDOW: roslaunch manipulator_h_moveit_config manipulator_h_planning_execution.launch 
 - THIRD WINDOW: roslaunch manipulator_h_path_planning manipulator_h_move_group_interface_tutorial.launch


0.2.0 (2017-03-16 @ 4:37 PM)
-----------
* Added the "manipulator_h_gazebo" package. The Gazebo simulator is now integrated with MoveIt!. Plans made in RViz can now be executed (and not only planned) and it is possible to see the robot moving in the Gazebo simulation;

* The following links were usefull to understand how to integrate Gazebo with MoveIt!:
 - https://github.com/AS4SR/general_info/wiki/ROS-MoveIt!-and-Gazebo-Integration-(WIP);
 - https://www.youtube.com/watch?v=j6bBxfD_bYs;
 - http://wiki.ros.org/joint_trajectory_controller

* In order to run the simulation, run the following in the terminal:
 - FIRST WINDOW: roslaunch manipulator_h_gazebo manipulator_h_gazebo.launch
 - SECOND WINDOW: roslaunch manipulator_h_moveit_config manipulator_h_planning_execution.launch  



0.1.2 (2018-03-15 @ 2:44 AM)
-----------
* Created the "manipulator_h_path_planning" package; 

* MoveIt! move group tutorial fully operational for Robotis Manipulator-H. The tutorial for the PR2 robot is available at "http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/pr2_tutorials/planning/src/doc/move_group_interface_tutorial.html". Code inside the "manipulator_h_path_planning";

* "manipulator_h_moveit_config" package slightly altered when testing possible errors for the tutorial to not run. Nothing important, different configurations didn't affect anything, they weren't the source of the error that was happening at the time.

* In order to run the simulation, run the following in the terminal:
 - FIRST WINDOW: roslaunch manipulator_h_moveit_config demo.launch 
 - SECOND WINDOW: roslaunch manipulator_h_path_planning manipulator_h_move_group_interface_tutorial.launch



0.1.1 (2018-03-15 @ 12:48 AM)
-------------------
* Added "manipulator_h_description" package and created a fully operational MoveIt! configuration package for this manipulator, named "manipulator_h_moveit_config";

* RViz showing the manipulator with no errors.

* In order to run the simulation, run the following in the terminal:
 - FIRST WINDOW: roslaunch manipulator_h_moveit_config demo.launch 



0.1.0 (2018-03-15 @ 12:34 AM)
-------------------
* First commit.
