add_executable(baxter baxter.cpp)
target_link_libraries(baxter ${catkin_LIBRARIES})

add_executable(pr2 pr2.cpp)
target_link_libraries(pr2 ${catkin_LIBRARIES})

add_executable(pa10 pa10.cpp)
target_link_libraries(pa10 ${catkin_LIBRARIES})

add_executable(pepper pepper.cpp)
target_link_libraries(pepper ${catkin_LIBRARIES})


add_executable(baxter_bimodal baxter_bimodal.cpp)
target_link_libraries(baxter_bimodal ${catkin_LIBRARIES} gtest)

add_executable(pr2_bimodal pr2_bimodal.cpp)
target_link_libraries(pr2_bimodal ${catkin_LIBRARIES} gtest)

add_executable(pa10_bimodal pa10_bimodal.cpp)
target_link_libraries(pa10_bimodal ${catkin_LIBRARIES} gtest)

add_executable(pepper_bimodal pepper_bimodal.cpp)
target_link_libraries(pepper_bimodal ${catkin_LIBRARIES} gtest)


add_executable(pepper_bimanual pepper_bimanual.cpp)
target_link_libraries(pepper_bimanual ${PROJECT_NAME}_stages ${catkin_LIBRARIES} gtest)


add_executable(yumi_handover yumi_handover.cpp)
target_link_libraries(yumi_handover ${catkin_LIBRARIES} gtest)

add_executable(yumi_bimanual yumi_bimanual.cpp)
target_link_libraries(yumi_bimanual ${PROJECT_NAME}_stages ${catkin_LIBRARIES} gtest)
