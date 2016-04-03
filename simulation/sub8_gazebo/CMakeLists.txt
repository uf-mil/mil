cmake_minimum_required(VERSION 2.8.3)
project(sub8_gazebo)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -Wall")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

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

add_library(
    sub8_buoyancy
        src/sub8_buoyancy.cc
)

target_link_libraries(
    sub8_buoyancy
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
)

add_library(
    sub8_thrusters
        src/sub8_thrusters.cc
)
target_link_libraries(
    sub8_thrusters
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
)

add_library(
    sub8_test
        src/sub8_test.cc
)

target_link_libraries(
    sub8_test
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
)


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES duck_gazebo
    CATKIN_DEPENDS gazebo_ros roscpp
    DEPENDS system_lib
)
