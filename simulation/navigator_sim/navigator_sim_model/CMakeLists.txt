
cmake_minimum_required(VERSION 2.8.3)

project(navigator_sim_model)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()
catkin_package(
    DEPENDS  # TODO
    CATKIN_DEPENDS navigator_sim_rendering
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

include_directories( ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

install(PROGRAMS scripts/thruster_conn scripts/imu_conn scripts/sim DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
