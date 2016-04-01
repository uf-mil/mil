cmake_minimum_required(VERSION 2.8.3)
project(duck_gazebo)

find_package(catkin REQUIRED COMPONENTS gazebo_ros roscpp)

# find_package(Boost REQUIRED COMPONENTS system)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    include
)

add_library(sub8_buoyancy src/sub8_buoyancy.cc)
target_link_libraries(sub8_buoyancy ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(sub8_test src/sub8_test.cc)
target_link_libraries(sub8_test ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES duck_gazebo
  CATKIN_DEPENDS gazebo_ros roscpp
  DEPENDS system_lib
)
