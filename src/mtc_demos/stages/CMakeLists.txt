set(SOURCES
	bimanual_grasp_pose.cpp
	generate_touch_pose.cpp
)
add_library(${PROJECT_NAME}_stages ${SOURCES})
target_link_libraries(${PROJECT_NAME}_stages
	PRIVATE ${catkin_LIBRARIES}
)
target_include_directories(${PROJECT_NAME}_stages INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
)

install(TARGETS ${PROJECT_NAME}_stages
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
