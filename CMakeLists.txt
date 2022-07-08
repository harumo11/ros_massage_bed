cmake_minimum_required(VERSION 3.0.2)
project(ros_massage_bed)

add_definitions("-std=c++20")

find_package(catkin REQUIRED COMPONENTS
	actionlib
	geometry_msgs
	moveit_commander
	moveit_msgs
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_visual_tools
	geometric_shapes
	roscpp
	tf2
	trajectory_msgs
	tm_msgs
	std_msgs
	)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(simple_force_massage src/simple_force_massage.cpp)
add_executable(techman_control_bridge src/techman_control_bridge.cpp)
add_executable(test_techman_control_bridge src/test_techman_control_bridge.cpp)

target_link_libraries(simple_force_massage ${catkin_LIBRARIES})
target_link_libraries(techman_control_bridge PocoFoundation PocoNet ${catkin_LIBRARIES})
target_link_libraries(test_techman_control_bridge PocoFoundation PocoNet)

