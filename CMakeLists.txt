cmake_minimum_required(VERSION 2.8.3)
project(enpm808x_final_inspection_robot)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
    actionlib
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
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs actionlib control_msgs move_base_msgs tf
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(inspection_controller src/InspectionControllerNode.cpp src/InspectionController.cpp)
target_link_libraries(inspection_controller ${catkin_LIBRARIES})

install(
    DIRECTORY include
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
    TARGETS inspection_controller
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
