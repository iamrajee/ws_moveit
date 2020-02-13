cmake_minimum_required(VERSION 2.8.3)
project(mtc_pour)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  moveit_task_constructor_core
  rviz_marker_tools
  roscpp
  moveit_ros_planning_interface
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES mtc_pour_into mtc_pour_into2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(mtc_pour_into src/pour_into.cpp)
add_dependencies(mtc_pour_into ${catkin_EXPORTED_TARGETS})
target_link_libraries(mtc_pour_into ${catkin_LIBRARIES})

add_library(mtc_pour_into2 src/pour_into2.cpp)                                              #<==== new
add_dependencies(mtc_pour_into2 ${catkin_EXPORTED_TARGETS})
target_link_libraries(mtc_pour_into2 ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_tams_ur5_demo src/mtc_pour_tams_ur5_demo.cpp)
add_dependencies(${PROJECT_NAME}_tams_ur5_demo ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_tams_ur5_demo
	mtc_pour_into
	${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_tams_ur5_demo2 src/mtc_pour_tams_ur5_demo2.cpp)                #<==== new
add_dependencies(${PROJECT_NAME}_tams_ur5_demo2 ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_tams_ur5_demo2
	mtc_pour_into2
	${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_baxter_demo src/mtc_pour_baxter_demo.cpp)
add_dependencies(${PROJECT_NAME}_baxter_demo ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_baxter_demo
	mtc_pour_into
	${catkin_LIBRARIES}
)
