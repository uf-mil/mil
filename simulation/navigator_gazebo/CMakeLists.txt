cmake_minimum_required(VERSION 2.8.3)
project(navigator_gazebo)

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -Wall")
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin
    REQUIRED COMPONENTS
    std_msgs
    nav_msgs
    rospy
    gazebo_ros
    roscpp
    message_generation
    roboteq_msgs
    xacro
)


find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    include
)

add_library(
    navigator_state_set
        src/navigator_state_set.cpp
)

set_target_properties(navigator_state_set PROPERTIES COMPILE_FLAGS "-std=c++11 -Wall")


target_link_libraries(
    navigator_state_set
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
)



add_library(
    navigator_buoyancy
        src/navigator_buoyancy.cpp
)

set_target_properties(navigator_buoyancy PROPERTIES COMPILE_FLAGS "-std=c++11 -Wall")


target_link_libraries(
    navigator_buoyancy
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
)

add_library(
    navigator_thrusters
        src/navigator_thrusters.cpp
)

set_target_properties(navigator_thrusters PROPERTIES COMPILE_FLAGS "-std=c++11 -Wall")


target_link_libraries(
    navigator_thrusters
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
)

add_library(
    sylphase_gazebo
        src/sylphase_gazebo.cpp
)

set_target_properties(sylphase_gazebo PROPERTIES COMPILE_FLAGS "-std=c++11 -Wall")
target_link_libraries(
    sylphase_gazebo
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
)

add_library(gazebo_ros_velodyne_laser src/GazeboRosVelodyneLaser.cpp)

set_target_properties(gazebo_ros_velodyne_laser PROPERTIES COMPILE_FLAGS "-std=c++11 -Wall")

target_link_libraries(gazebo_ros_velodyne_laser
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  RayPlugin
)

add_library(animated_box SHARED src/animated_box.cc)
target_link_libraries(animated_box ${GAZEBO_LIBRARIES})

set_target_properties(animated_box PROPERTIES COMPILE_FLAGS "-std=c++11 -Wall")

add_dependencies(navigator_thrusters
    navigator_msgs_generate_messages_cpp
    navigator_msgs_generate_messages_py
    ${catkin_EXPORTED_TARGETS}
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS gazebo_ros roscpp rospy std_msgs nav_msgs message_runtime tf gazebo_plugins sensor_msgs
)

xacro_add_files(
  urdf/navigator.urdf.xacro
  INORDER INSTALL DESTINATION urdf
)

# if(CATKIN_ENABLE_TESTING)
#     find_package(rostest)
#     add_subdirectory(test)
# endif()
