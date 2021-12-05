cmake_minimum_required(VERSION 2.8.3)
project(enpm808x_final_inspection_robot)

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

catkin_package(
    CATKIN_DEPENDS roscpp
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
