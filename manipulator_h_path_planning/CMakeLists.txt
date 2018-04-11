cmake_minimum_required(VERSION 2.8.3)
project(manipulator_h_path_planning)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    moveit_visual_tools
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# Only Manipulator-H
add_executable(manipulator_h_move_group_interface_tutorial src/manipulator_h_move_group_interface_tutorial.cpp)
target_link_libraries(manipulator_h_move_group_interface_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(manipulator_h_kinematic_model_tutorial src/manipulator_h_kinematic_model_tutorial.cpp)
target_link_libraries(manipulator_h_kinematic_model_tutorial ${catkin_LIBRARIES})

add_executable(manipulator_h_planning_scene_tutorial src/manipulator_h_planning_scene_tutorial.cpp)
target_link_libraries(manipulator_h_planning_scene_tutorial ${catkin_LIBRARIES})

add_executable(manipulator_h_planning_scene_ros_api_tutorial src/manipulator_h_planning_scene_ros_api_tutorial.cpp)
target_link_libraries(manipulator_h_planning_scene_ros_api_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES} interactive_markers)

add_executable(manipulator_h_planning_pipeline_tutorial src/manipulator_h_planning_pipeline_tutorial.cpp)
target_link_libraries(manipulator_h_planning_pipeline_tutorial ${catkin_LIBRARIES})

# Husky + Manipulator-H
add_executable(husky_manipulator_h_move_group_interface_tutorial src/husky_manipulator_h_move_group_interface_tutorial.cpp)
target_link_libraries(husky_manipulator_h_move_group_interface_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(oak_tree_tests src/oak_tree_tests.cpp)
target_link_libraries(oak_tree_tests ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(laser_tests src/laser_tests.cpp)
target_link_libraries(laser_tests ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(fake_vine_tree_tests src/fake_vine_tree_tests.cpp)
target_link_libraries(fake_vine_tree_tests ${catkin_LIBRARIES} ${Boost_LIBRARIES})
