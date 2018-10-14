cmake_minimum_required(VERSION 2.8.3)
project(mil_gazebo)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  gazebo_ros
  mil_msgs
)

catkin_package(INCLUDE_DIRS include)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    include
)


add_library(
    mil_model_heading
        src/mil_model_heading.cpp
)
target_link_libraries(mil_model_heading
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(mil_model_heading ${catkin_EXPORTED_TARGETS})
