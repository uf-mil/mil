cmake_minimum_required(VERSION 2.8.3)
project(sub8_controller)

# Require C++11
# [1] http://answers.ros.org/question/152276/is-there-a-way-to-enable-c11-support-for-catkin-packages/
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
add_definitions ("-Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  eigen_conversions
  dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)

include_directories(EIGEN3_INCLUDE_DIR)

set(catkin_LIBRARIES
    ${catkin_LIBRARIES}
)

generate_dynamic_reconfigure_options(
  cfg/pd_controller.cfg
)

catkin_package(
  CATKIN_DEPENDS roscpp sub8_msgs dynamic_reconfigure
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  include
)

add_executable(pd_controller nodes/pd_controller.cc)
target_link_libraries(pd_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(pd_controller
    sub8_msgs_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
)

# add_executable(mpc nodes/mpc.cc)
# target_link_libraries(mpc ${catkin_LIBRARIES} ${Boost_LIBRARIES})