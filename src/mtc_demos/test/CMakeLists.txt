#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)

	find_package(pr2_moveit_config QUIET)
	if (pr2_moveit_config_FOUND)
		add_rostest(pr2_bimodal.test)
	endif()

	find_package(pa10_shadow_moveit_config QUIET)
	if (pa10_shadow_moveit_config_FOUND)
		add_rostest(pa10_bimodal.test)
	endif()

	find_package(baxter_moveit_config QUIET)
	if (baxter_moveit_config_FOUND)
		add_rostest(baxter_bimodal.test)
	endif()

	find_package(pepper_moveit_config QUIET)
	if (pepper_moveit_config_FOUND)
		add_rostest(pepper_bimodal.test)
		add_rostest(pepper_bimanual.test)
	endif()

	find_package(yumi_moveit_config QUIET)
	if (yumi_moveit_config_FOUND)
		add_rostest(yumi_bimanual.test)
		add_rostest(yumi_handover.test)
	endif()
endif()
