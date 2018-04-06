^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package brito_thesis
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.1 (2017-04-06 @ 2:07 PM)
-----------

* After the responde from v4hn at my post in this link: https://answers.ros.org/question/287686/problem-with-allowedcollisionmatrix-and-octomap-in-moveit/?answer=287747#post-id-287747
I figured out how to change the padding of the cube that updates the collision octomap. You have to change the "padding_offset" parameter in the "sensors_Hokuyo_UST_10LX_pointcloud2.yaml" file inside the "husky_manipulator_h_path_planning" package, its value used to be "0.1" and now is "0.05". Note that as you decrease this value, you have to set the center point of the cube closer to the octomap tree's limits or, otherwise, the padding will be so small that it won't affect (update) the collision octomap.

* Changed the "laser_tests.cpp" file so that now the oak tree is automatically eliminated during the script. Also changed the point where the cube is "spawned" to a spot that is closer to the collision octomap limits, so that I can decrease its padding and make a smaller gap in the octomap, which will not negatively influence much the remaining environment.

* In order to test the code, run the following in a terminal:
 - FIRST WINDOW: roslaunch husky_gazebo husky_laser_tests.launch vertical_laser_enabled:=true manipulator_h_enabled:=true
 - SECOND WINDOW: roslaunch husky_manipulator_h_moveit_config husky_manipulator_h_planning_execution.launch
 - THIRD WINDOW: roslaunch manipulator_h_path_planning laser_tests.launch 


0.7.0 (2017-04-05 @ 6:42 PM)
-----------

* Altered the "laser_tests.cpp" file so that now it is possible to perform path planning to a point in the world whom was previously identified as a collision object by the laser and was mapped in the octomap. To do this, I update the AllowedCollisionMatrix of MoveIt! so that it allows collisions between a "fictitious" cube and all parts of the manipulator. This makes it possible open a gap in the octomap, with this cube is close to its limits. To show that this behaviour alls the planner to plan into a position that was previously obstructed with the octomap, I tried to plan to a point inside of a tree before and after I use this "fictitious" cube to tell MoveIt! to not obstruct the environment around my goal pose.

* I changed the "resolution" parameter of my "octomap_server" node in order to map the tree with less gaps. Resolution value was "0.05" and now is "0.02".

* Also changed the goal point to one that is easier to reach while mapping a smaller part of the tree.

* The tree is now spawned once again in the "husky_laser_tests.launch" file.

* In order to test the code, run the following in a terminal:
 - FIRST WINDOW: roslaunch husky_gazebo husky_laser_tests.launch vertical_laser_enabled:=true manipulator_h_enabled:=true
 - SECOND WINDOW: roslaunch husky_manipulator_h_moveit_config husky_manipulator_h_planning_execution.launch
 - THIRD WINDOW: roslaunch manipulator_h_path_planning laser_tests.launch 

* Start by launching all terminals and then move the robot around to map the tree till the goal point is all covered by the octomap. After that, put the robot in a valid position to reach the goal pose and click "Next" in the RViz window to complete the test.


0.6.3 (2017-04-03 @ 8:16 PM)
-----------

* Added a path planning script to make it possible to test if the manipulator can do path planning before and after detecting a tree with the laser. Altered the "husky_laser_tests.launch" to not spawn the tree in the launch file as the test script needs the tree not to be spawned at the beginning of the simulation.

* In order to test the code, run the following in a terminal:
 - FIRST WINDOW: roslaunch husky_gazebo husky_laser_tests.launch vertical_laser_enabled:=true manipulator_h_enabled:=true
 - SECOND WINDOW: roslaunch husky_manipulator_h_moveit_config husky_manipulator_h_planning_execution.launch
 - THIRD WINDOW: roslaunch manipulator_h_path_planning laser_tests.launch 
 - Only run the FOURTH WINDOW after the path planning from the THIRD WINDOW ended and, after that, after you manually plan the manipulator to its "home_position". After running the FOURTH WINDOW, run THIRD WINDOW again.
 - FOURTH WINDOW: rosrun gazebo_ros spawn_model -file /home/josebrito/catkin_ws/src/brito_thesis/manipulator_h_path_planning/meshes/oak_tree/model.sdf -sdf -model oak_tree -y 1

* Also some experiments with a false localization node that adds a "/map" frame to the world. For that I installed the "fake_localization" package from Synaptic and started to make changes in the following files:
 - Added the node in the "husky_laser_tests.launch" inside the "husly_gazebo" package
 - Altered the parent frame from "odom" to "map" in the virtual joint tag inside the "husky.srdf" file inside the "husky_manipulator_h_moveit_config" package
 - In RViz, change the global frame to "map"
 - In the file "laser_tests.cpp" I altered the visual_tools frame from "odom" to "map"

* These changes were not yet commited because I have to check if the frame "map" is working correctly


0.6.2 (2017-04-03 @ 1:34 PM)
-----------

* MoveIt! can now automatically map objects from the world as collision objects, using the laser Hokuyo UST-10LX.

* Modified the "husky_manipulator_h_moveit_config":
 - Launched the setup assistant and also used "vertical_laser_enabled:=true", then ran again the collision matrix in order to make MoveIt! aware of the laser and its frames
 - In file "husky.srdf", inside the "config" folder:
   - Altered the virtual joint type from "fixed" to "planar" because now path planning will be done considering Husky's movements; Check the MoveIt! tutorial in the following link:
      http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html#step-3-add-virtual-joints
   - Altered parent frame of the virtual joint from "world" to "odom"
   - Added the file "config/sensors_Hokuyo_UST_10LX_pointcloud2.yaml", which configures MoveIt! to automatically map obstacles using the Hokuyo UST-10LX laser, more specifically, using the pointCloud2 map which is is the output of the "octomap_server" topic. Check this link for more information: 
      http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/pr2_tutorials/planning/src/doc/perception_configuration.html
 - In file "sensor_manager.launch.xml", inside the "launch" folder:
   - Now loading the sensor plugin to the ROS parameter server
   - Watchout for the fact that "odom_combined" is now the default octomap's frame but it is configured in "husky_manipulator_h_planning_execution.launch" to be the "odom" frame
 - In file "husky_manipulator_h_planning_execution.launch" included "sensor_manager.launch.xml" file.
  
* Modified the "husky_gazebo" package: 
  - Removed both static tfs from "spawn_husky.launch" file inside the "husky_gazebo" package because they are not needed;
  - In the "husky_laser_tests.launch", disabled the RViz visualization, since now all RViz visualizations are done by the RViz window launched by MoveIt! when launching the "husky_manipulator_h_planning_execution.launch" inside the "husky_manipulator_h_moveit_config" package.

* Move around the tree in order to map her and make MoveIt! interpret her as a collision object
 - FIRST WINDOW: roslaunch husky_gazebo husky_laser_tests.launch vertical_laser_enabled:=true manipulator_h_enabled:=true
 - SECOND WINDOW: roslaunch husky_manipulator_h_moveit_config husky_manipulator_h_planning_execution.launch 


0.6.1 (2017-04-02 @ 6:03 PM)
-----------
* Added the package "laser_to_pcl" that tranforms measures from the laser Hokuyo UST-10LX into point clouds. Also installed the package "octomap_server" to make it possible to make a map of point cloud detected by the laser. 

* To test that everything works fine, run the following in a terminal window and use the teleop keys to make the Husky walk around:
 - FIRST WINDOW: roslaunch husky_gazebo husky_laser_tests.launch vertical_laser_enabled:=true


0.6.0 (2017-04-02 @ 4:29 PM)
-----------
* Added the laser Hokuyo UST-10LX to the simulation. This laser is on the top of the AGROB tower. Used the "hokuyo.dae" file from the "robot_description" package inside the "agrob_simulation-master" package for the geometry tag of the laser. Possible modifications to this file can happen.

* To test that everything works fine, run the following in a terminal window:
 - FIRST WINDOW: roslaunch husky_gazebo husky_empty_world.launch vertical_laser_enabled:=true


0.5.3 (2017-03-30 @ 3:45 PM)
-----------
* Reconfigured the MoveIt! package for the Husky + Manipulator-H integration in order to be fully compatible with the Husky Kinetic package. 

* To test that everything works fine, run the oak tree tests again:
 - FIRST WINDOW: roslaunch husky_gazebo husky_empty_world.launch manipulator_h_enabled:=true
 - SECOND WINDOW: rosrun gazebo_ros spawn_model -file /home/josebrito/catkin_ws/src/brito_thesis/manipulator_h_path_planning/meshes/oak_tree/model.sdf -sdf -model oak_tree -y 1
 - THIRD WINDOW: roslaunch husky_manipulator_h_moveit_config husky_manipulator_h_planning_execution.launch
 - FOURTH WINDOW: roslaunch manipulator_h_path_planning oak_tree_tests.launch


0.5.2 (2017-03-30 @ 0:59 AM)
-----------
* Removed the Husky's top plate when spawning the Manipulator-H. In order to do that, disabled the top plate in the following file:
 - "decorations.urdf.xacro" inside the "husky_description" package.

* Keep in mind that the top plate is necessary when launching the kinect or the laser! If you want to launch both the laser and the Manipulator-H, go to the "decorations.urdf.xacro" file and enable to top plate. There will be a collision between the top plate and the box where the Manipulator-H stands but right now I still haven't figured out if that will cause any problems in MoveIt!, since I still haven't started to work with it.

* Integrated the UR5 with the Husky Kinetic package. Keep in mind that UR5 also needs the Husky's top plate. I altered the file "arm_controller_ur5.yaml" from the package "ur_gazebo" because the controller spawned for the UR5 arm needed a prefix added to its joints names for them to have specific joint names that won't be mistaken by the Husky's joint names.

* UR5 is now possible to visualize in both Gazebo and RViz (through the "view_model.launch" file in the "husky_viz" package). Manipulator-H was only visible on Gazebo in the last commit and now it's also visible in RViz.

* In order to run the simulation, run the following in the terminal:
 - FIRST WINDOW: roslaunch husky_gazebo husky_empty_world.launch ur5_enabled:=true


0.5.1 (2017-03-29 @ 12:30 AM)
-----------
* Integrated the Robotis Manipulator-H with the Husky Kinetic package. Right now only spawning the Husky and the Manipulator-H is working.

* In order to run the simulation, run the following in the terminal:
 - FIRST WINDOW: roslaunch husky_gazebo husky_empty_world.launch manipulator_h_enabled:=true


0.5.0 (2017-03-29 @ 10:12 AM)
-----------
* Started to substitute packages from Husky INDIGO to Husky KINETIC. Right now only deleted the indigo packages and downloaded the kinetic ones


0.4.3 (2017-03-28 @ 5:59 PM)
-----------
* Added a new tutorial were path planning is done taking in account an oak tree loaded by a SDF file. Keep in mind that you have to manually add and remove the oak tree from the gazebo simulation.

* In order to perform path planning with RViz and visualizing them both in RViz and Gazebo, run the following in the terminal:
 - FIRST WINDOW: roslaunch husky_gazebo husky_empty_world.launch manipulator_h_enabled:=true
 - SECOND WINDOW: rosrun gazebo_ros spawn_model -file /home/josebrito/catkin_ws/src/brito_thesis/manipulator_h_path_planning/meshes/oak_tree/model.sdf -sdf -model oak_tree -y 1
 - THIRD WINDOW: roslaunch husky_manipulator_h_moveit_config husky_manipulator_h_planning_execution.launch
 - FOURTH WINDOW: roslaunch manipulator_h_path_planning oak_tree_tests.launch


0.4.2 (2017-03-28 @ 11:12 AM)
-----------
* MoveIt! move group tutorials now working for the Husky + Manipulator-H integration. Watchout for the modifications in the node that send the MoveIt! controls. The base frame is now "base_link" instead of "world".

* In order to perform path planning with RViz and visualizing them both in RViz and Gazebo, run the following in the terminal:
 - FIRST WINDOW: roslaunch husky_gazebo husky_empty_world.launch manipulator_h_enabled:=true
 - SECOND WINDOW: roslaunch husky_manipulator_h_moveit_config husky_manipulator_h_planning_execution.launch
 - THIRD WINDOW: roslaunch manipulator_h_path_planning husky_manipulator_h_move_group_interface_tutorial.launch


0.4.1 (2017-03-27 @ 1:09 PM)
-----------
* Plans done in RViz can now be visualized in Gazebo. Altered files can be seen in Git Kraken.

* In order to perform path planning with RViz and visualizing them both in RViz and Gazebo, run the following in the terminal:
 - FIRST WINDOW: roslaunch husky_gazebo husky_empty_world.launch manipulator_h_enabled:=true
 - SECOND WINDOW: roslaunch husky_manipulator_h_moveit_config husky_manipulator_h_planning_execution.launch


0.4.0 (2017-03-27 @ 11:59 AM)
-----------
* Added package "husky_manipulator_h_moveit_config" to make it possible to perform path planning with the manipulator on top of the Husky robot. Right now path planning can be done with RViz, using the "demo.launch". Keep in mind that future changes to the configurations may be necessary because the AGROB V16's tower isn't modelled and the manipulator's base isn't 100% modelled.

* When launching the MoveIt! Setup Assistant use "--inorder manipulator_h_enabled:=true" in "xacro arguments" when loading the MoveIt! configuration to load the manipulator on top of the Husky robot.

* The following website links were useful to do the configuration, especially during the virtual joint setup:
 - http://wiki.ros.org/Industrial/Tutorials/Create_a_MoveIt_Pkg_for_an_Industrial_Robot
 - http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html#step-3-add-virtual-joints

* In order to perform path planning with RViz, run the following in the terminal (note that plans can't still be visualized in Gazebo):
 - FIRST WINDOW: roslaunch husky_manipulator_h_moveit_config demo.launch


0.3.2 (2017-03-26 @ 6:46 PM)
-----------
* Commit changes in the "husky" and "husky_simulator" indigo repositories. Don't really know why the commits on this repositories weren't automatically done. 


0.3.2 (2017-03-26 @ 6:37 PM)
-----------
* Created a box and a cylinder to simulate the base that sustains the manipulator. 

* Note that in boxes and cylinders, the Z axis starts to count from the middle of the object. Also fixed links and joints don't appear in Gazebo because they can't be moved so Gazebo decides to ignore them

* Try to move the robot by clicking on the keyboard while selecting the second terminal, after running the following in 2 different terminals:
 - FIRST WINDOW: roslaunch husky_gazebo husky_empty_world.launch manipulator_h_enabled:=true
 - SECOND WINDOW: rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=husky_velocity_controller/cmd_vel


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
 - https://github.com/AS4SR/general_info/wiki/ROS-MoveIt!-and-Gazebo-Integration-(WIP)
 - https://www.youtube.com/watch?v=j6bBxfD_bYs
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
