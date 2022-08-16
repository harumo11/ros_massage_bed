cmake_minimum_required(VERSION 3.12)
project(ros_massage_bed)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions("-std=c++2a")

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

find_package(Boost REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(simple_force_massage src/simple_force_massage.cpp)
add_executable(techman_control_bridge src/techman_control_bridge.cpp)
add_executable(techman_sensor_bridge src/techman_sensor_bridge.cpp)
add_executable(leptrino_sensor_bridge src/leptrino_sensor_bridge.cpp)
add_executable(test_techman_control_bridge src/test_techman_control_bridge.cpp)
add_executable(test_techman_sensor_bridge src/test_techman_sensor_bridge.cpp)
add_executable(test_leptrino_sensor_bridge src/test_leptrino_sensor_bridge.cpp)
add_executable(tool_techman_control_bridge 
	src/tool_techman_control_bridge.cpp
	${CMAKE_SOURCE_DIR}/src/third_party/imgui.cpp
	${CMAKE_SOURCE_DIR}/src/third_party/imgui_demo.cpp
	${CMAKE_SOURCE_DIR}/src/third_party/imgui_draw.cpp
	${CMAKE_SOURCE_DIR}/src/third_party/imgui_tables.cpp
	${CMAKE_SOURCE_DIR}/src/third_party/imgui_widgets.cpp
	${CMAKE_SOURCE_DIR}/src/third_party/imgui_impl_opengl3.cpp
	${CMAKE_SOURCE_DIR}/src/third_party/imgui_impl_glfw.cpp
	)

target_link_libraries(simple_force_massage ${catkin_LIBRARIES})
target_link_libraries(techman_control_bridge glog ${catkin_LIBRARIES})
target_link_libraries(leptrino_sensor_bridge ${catkin_LIBRARIES})
target_link_libraries(test_techman_control_bridge PocoFoundation PocoNet)
target_link_libraries(tool_techman_control_bridge pthread glfw GLEW OpenGL::GL ${CMAKE_DL_LIBS})
target_link_libraries(techman_sensor_bridge ${catkin_LIBRARIES})
target_link_libraries(test_techman_sensor_bridge ${catkin_LIBRARIES})
target_link_libraries(test_leptrino_sensor_bridge ${catkin_LIBRARIES})

