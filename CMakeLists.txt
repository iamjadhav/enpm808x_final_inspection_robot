cmake_minimum_required(VERSION 2.8.3)
project(enpm808x_final_inspection_robot)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
    geometry_msgs
    sensor_msgs
)

add_service_files(
    DIRECTORY srv
    FILES InspectCan.srv LocalizeCan.srv
)
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs)

catkin_package(
    CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

install(
    DIRECTORY urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
    DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
