cmake_minimum_required(VERSION 2.8.3)
project(enpm808x_final_inspection_robot)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
    actionlib
    std_msgs
    geometry_msgs
    sensor_msgs
    control_msgs
    move_base_msgs
    tf
)

add_service_files(
    DIRECTORY srv
    FILES InspectCan.srv LocalizeCan.srv
)
add_message_files(
    DIRECTORY msg
    FILES InspectionMetrics.msg
)
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs actionlib control_msgs move_base_msgs tf
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(inspection_controller src/InspectionControllerNode.cpp src/InspectionController.cpp)
target_link_libraries(inspection_controller ${catkin_LIBRARIES})
add_dependencies(inspection_controller enpm808x_final_inspection_robot_generate_messages_cpp)

add_executable(can_characterizer src/CanCharacterizer.cpp src/CanCharacterizerNode.cpp)
target_link_libraries(can_characterizer ${catkin_LIBRARIES})
add_dependencies(can_characterizer enpm808x_final_inspection_robot_generate_messages_cpp)

install(
    DIRECTORY include
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
    TARGETS inspection_controller can_characterizer
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
    DIRECTORY urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
    DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    add_rostest_gtest(
        test_main
        test/launch/test_main.launch
        test/TestMain.cpp
    )
    target_link_libraries(
        test_main
        ${catkin_LIBRARIES}
    )
#    add_dependencies(
#        test_main
#        inspection_controller
#        can_characterizer
#    )
    install(
        DIRECTORY test
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
endif(CATKIN_ENABLE_TESTING)
