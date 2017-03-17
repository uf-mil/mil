cmake_minimum_required(VERSION 2.8.3)
project(navigator_tools)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -pedantic -Wall")

find_package(catkin
  REQUIRED COMPONENTS
    rostest
    std_msgs
    roscpp
    dynamic_reconfigure
)
catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/BoundsConfig.cfg
)

# make sure configure headers are built before any node using them
#add_dependencies(lidar ${PROJECT_NAME}_gencfg)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    navigator_tools_lib
  CATKIN_DEPENDS
    roscpp
)

# Add folders to be run by python nosetests
if(CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(tests)
endif()

add_library(navigator_tools_lib src/navigator_tools.cpp)
target_include_directories(navigator_tools_lib PUBLIC include)
target_link_libraries(navigator_tools_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_compile_options(navigator_tools_lib PUBLIC -std=c++11)