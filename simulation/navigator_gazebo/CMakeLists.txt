cmake_minimum_required(VERSION 2.8.3)
project(navigator_gazebo)

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
    navigator_thrusters
        src/navigator_thrusters.cpp
)
set_target_properties(navigator_thrusters PROPERTIES COMPILE_FLAGS "-std=c++11 -Wall")
target_link_libraries(navigator_thrusters
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(navigator_thrusters
  ${catkin_EXPORTED_TARGETS}
)

add_library(sylphase_gazebo
  src/sylphase_gazebo.cpp
)
set_target_properties(sylphase_gazebo PROPERTIES COMPILE_FLAGS "-std=c++11 -Wall")
target_link_libraries(sylphase_gazebo
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(sylphase_gazebo
    ${catkin_EXPORTED_TARGETS}
)

add_library(gazebo_ros_velodyne_laser src/GazeboRosVelodyneLaser.cpp)
set_target_properties(gazebo_ros_velodyne_laser PROPERTIES COMPILE_FLAGS "-std=c++11 -Wall")
target_link_libraries(gazebo_ros_velodyne_laser
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  RayPlugin
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
