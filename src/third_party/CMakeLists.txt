cmake_minimum_required(VERSION 3.12)
project(imgui)

find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(dashboard
	dashboard.cpp
	${CMAKE_SOURCE_DIR}/imgui.cpp
	${CMAKE_SOURCE_DIR}/imgui_demo.cpp
	${CMAKE_SOURCE_DIR}/imgui_draw.cpp
	${CMAKE_SOURCE_DIR}/imgui_tables.cpp
	${CMAKE_SOURCE_DIR}/imgui_widgets.cpp
	${CMAKE_SOURCE_DIR}/imgui_impl_opengl3.cpp
	${CMAKE_SOURCE_DIR}/imgui_impl_glfw.cpp
	)


target_link_libraries(dashboard
	glfw
	GLEW 
	GL
	${CMAKE_DL_LIBS}
	)




