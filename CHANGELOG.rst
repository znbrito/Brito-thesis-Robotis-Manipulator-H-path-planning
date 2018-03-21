^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package brito_thesis
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
