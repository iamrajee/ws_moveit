cmake_minimum_required(VERSION 2.8.3)
project(moveit_task_constructor_demo)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	moveit_core
	moveit_task_constructor_core
	moveit_ros_planning_interface
	rosparam_shortcuts
  mtc_pour
  std_msgs
) 

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cartesian src/cartesian.cpp)
target_link_libraries(cartesian ${catkin_LIBRARIES})

add_executable(cartesian2 src/cartesian2.cpp)
target_link_libraries(cartesian2 ${catkin_LIBRARIES})

add_executable(cartesian3 src/cartesian3.cpp)
target_link_libraries(cartesian3 ${catkin_LIBRARIES})

add_executable(cartesian4 src/cartesian4.cpp)
target_link_libraries(cartesian4 ${catkin_LIBRARIES})

add_executable(modular src/modular.cpp)
target_link_libraries(modular ${catkin_LIBRARIES})

add_executable(modular2 src/modular2.cpp)
target_link_libraries(modular2 ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_lib src/pick_place_task.cpp)                                                       # <= why not directly change here
set_target_properties(${PROJECT_NAME}_lib PROPERTIES OUTPUT_NAME moveit_task_constructor_demo_pick_place)      # <= useless??
add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_lib2 src/pick_place_task2.cpp)
set_target_properties(${PROJECT_NAME}_lib2 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo_pick_place2)
add_dependencies(${PROJECT_NAME}_lib2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_lib3 src/pick_place_task3.cpp)
set_target_properties(${PROJECT_NAME}_lib3 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo_pick_place3)
add_dependencies(${PROJECT_NAME}_lib3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_lib4 src/pick_place_task4.cpp)
set_target_properties(${PROJECT_NAME}_lib4 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo_pick_place4)
add_dependencies(${PROJECT_NAME}_lib4 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_lib5 src/pick_place_task5.cpp)
set_target_properties(${PROJECT_NAME}_lib5 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo_pick_place5)
add_dependencies(${PROJECT_NAME}_lib5 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_lib6 src/pick_place_task6.cpp)
set_target_properties(${PROJECT_NAME}_lib6 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo_pick_place6)
add_dependencies(${PROJECT_NAME}_lib6 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_lib7 src/pick_place_task7.cpp)
set_target_properties(${PROJECT_NAME}_lib7 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo_pick_place7)
add_dependencies(${PROJECT_NAME}_lib7 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_lib8 src/pick_place_task8.cpp)
set_target_properties(${PROJECT_NAME}_lib8 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo_pick_place8)
add_dependencies(${PROJECT_NAME}_lib8 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_lib9 src/pick_place_task9.cpp)
set_target_properties(${PROJECT_NAME}_lib9 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo_pick_place9)
add_dependencies(${PROJECT_NAME}_lib9 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_lib10 src/pick_place_task10.cpp)
set_target_properties(${PROJECT_NAME}_lib10 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo_pick_place10)
add_dependencies(${PROJECT_NAME}_lib10 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_lib11 src/pick_place_task11.cpp)
set_target_properties(${PROJECT_NAME}_lib11 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo_pick_place11)
add_dependencies(${PROJECT_NAME}_lib11 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_lib12 src/pick_place_task12.cpp)
set_target_properties(${PROJECT_NAME}_lib12 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo_pick_place12)
add_dependencies(${PROJECT_NAME}_lib12 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_lib13 src/pick_place_task13.cpp)
set_target_properties(${PROJECT_NAME}_lib13 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo_pick_place13)
add_dependencies(${PROJECT_NAME}_lib13 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_lib14 src/pick_place_task14.cpp)
set_target_properties(${PROJECT_NAME}_lib14 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo_pick_place14)
add_dependencies(${PROJECT_NAME}_lib14 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_lib15 src/pick_place_task15.cpp)
set_target_properties(${PROJECT_NAME}_lib15 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo_pick_place15)
add_dependencies(${PROJECT_NAME}_lib15 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})







add_executable(${PROJECT_NAME}_demo src/moveit_task_constructor_demo.cpp)
target_link_libraries(${PROJECT_NAME}_demo ${PROJECT_NAME}_lib ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_demo PROPERTIES OUTPUT_NAME moveit_task_constructor_demo)
add_dependencies(${PROJECT_NAME}_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_demo2 src/moveit_task_constructor_demo2.cpp)
target_link_libraries(${PROJECT_NAME}_demo2 ${PROJECT_NAME}_lib2 ${catkin_LIBRARIES})              # <= important lib2 !!!
set_target_properties(${PROJECT_NAME}_demo2 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo2)
add_dependencies(${PROJECT_NAME}_demo2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(${PROJECT_NAME}_demo3 src/moveit_task_constructor_demo3.cpp)
target_link_libraries(${PROJECT_NAME}_demo3 ${PROJECT_NAME}_lib3 ${catkin_LIBRARIES})              # <= important lib3 !!!
set_target_properties(${PROJECT_NAME}_demo3 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo3)
add_dependencies(${PROJECT_NAME}_demo3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_demo4 src/moveit_task_constructor_demo4.cpp)
target_link_libraries(${PROJECT_NAME}_demo4 ${PROJECT_NAME}_lib4 ${catkin_LIBRARIES})              # <= important lib4 !!!
set_target_properties(${PROJECT_NAME}_demo4 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo4)
add_dependencies(${PROJECT_NAME}_demo4 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_demo5 src/moveit_task_constructor_demo5.cpp)
target_link_libraries(${PROJECT_NAME}_demo5 ${PROJECT_NAME}_lib5 ${catkin_LIBRARIES})              # <= important lib5 !!!
set_target_properties(${PROJECT_NAME}_demo5 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo5)
add_dependencies(${PROJECT_NAME}_demo5 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_demo6 src/moveit_task_constructor_demo6.cpp)
target_link_libraries(${PROJECT_NAME}_demo6 ${PROJECT_NAME}_lib6 ${catkin_LIBRARIES})              # <= important lib6 !!!
set_target_properties(${PROJECT_NAME}_demo6 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo6)
add_dependencies(${PROJECT_NAME}_demo6 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_demo7 src/moveit_task_constructor_demo7.cpp)
target_link_libraries(${PROJECT_NAME}_demo7 ${PROJECT_NAME}_lib7 ${catkin_LIBRARIES})              # <= important lib7 !!!
set_target_properties(${PROJECT_NAME}_demo7 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo7)
add_dependencies(${PROJECT_NAME}_demo7 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_demo8 src/moveit_task_constructor_demo8.cpp)
target_link_libraries(${PROJECT_NAME}_demo8 ${PROJECT_NAME}_lib8 ${catkin_LIBRARIES})              # <= important lib8 !!!
set_target_properties(${PROJECT_NAME}_demo8 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo8)
add_dependencies(${PROJECT_NAME}_demo8 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_demo9 src/moveit_task_constructor_demo9.cpp)
target_link_libraries(${PROJECT_NAME}_demo9 ${PROJECT_NAME}_lib9 ${catkin_LIBRARIES})              # <= important lib9 !!!
set_target_properties(${PROJECT_NAME}_demo9 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo9)
add_dependencies(${PROJECT_NAME}_demo9 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_demo10 src/moveit_task_constructor_demo10.cpp)
target_link_libraries(${PROJECT_NAME}_demo10 ${PROJECT_NAME}_lib10 ${catkin_LIBRARIES})              # <= important lib10 !!!
set_target_properties(${PROJECT_NAME}_demo10 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo10)
add_dependencies(${PROJECT_NAME}_demo10 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_demo11 src/moveit_task_constructor_demo11.cpp)
target_link_libraries(${PROJECT_NAME}_demo11 ${PROJECT_NAME}_lib11 ${catkin_LIBRARIES})              # <= important lib11 !!!
set_target_properties(${PROJECT_NAME}_demo11 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo11)
add_dependencies(${PROJECT_NAME}_demo11 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_demo12 src/moveit_task_constructor_demo12.cpp)
target_link_libraries(${PROJECT_NAME}_demo12 ${PROJECT_NAME}_lib12 ${catkin_LIBRARIES})              # <= important lib12 !!!
set_target_properties(${PROJECT_NAME}_demo12 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo12)
add_dependencies(${PROJECT_NAME}_demo12 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_demo13 src/moveit_task_constructor_demo13.cpp)
target_link_libraries(${PROJECT_NAME}_demo13 ${PROJECT_NAME}_lib13 ${catkin_LIBRARIES})              # <= important lib13 !!!
set_target_properties(${PROJECT_NAME}_demo13 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo13)
add_dependencies(${PROJECT_NAME}_demo13 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_demo14 src/moveit_task_constructor_demo14.cpp)
target_link_libraries(${PROJECT_NAME}_demo14 ${PROJECT_NAME}_lib14 ${catkin_LIBRARIES})              # <= important lib14 !!!
set_target_properties(${PROJECT_NAME}_demo14 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo14)
add_dependencies(${PROJECT_NAME}_demo14 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_demo15 src/moveit_task_constructor_demo15.cpp)
target_link_libraries(${PROJECT_NAME}_demo15 ${PROJECT_NAME}_lib15 ${catkin_LIBRARIES})              # <= important lib15 !!!
set_target_properties(${PROJECT_NAME}_demo15 PROPERTIES OUTPUT_NAME moveit_task_constructor_demo15)
add_dependencies(${PROJECT_NAME}_demo15 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_demo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_lib2 ${PROJECT_NAME}_demo2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_lib3 ${PROJECT_NAME}_demo3
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_lib4 ${PROJECT_NAME}_demo4
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_lib5 ${PROJECT_NAME}_demo5
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_lib6 ${PROJECT_NAME}_demo6
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_lib7 ${PROJECT_NAME}_demo7
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_lib8 ${PROJECT_NAME}_demo8
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_lib9 ${PROJECT_NAME}_demo9
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_lib10 ${PROJECT_NAME}_demo10
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_lib11 ${PROJECT_NAME}_demo11
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_lib12 ${PROJECT_NAME}_demo12
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_lib13 ${PROJECT_NAME}_demo13
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_lib14 ${PROJECT_NAME}_demo14
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_lib15 ${PROJECT_NAME}_demo15
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
