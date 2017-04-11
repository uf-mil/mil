cmake_minimum_required(VERSION 2.8.3)
project(mil_tools)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -pedantic -Wall -std=c++11")

find_package(catkin REQUIRED COMPONENTS 
  rostest
  std_msgs
  rospy
  roscpp
  cv_bridge
  cmake_modules
)

find_package(Eigen 3 REQUIRED)

catkin_package(
  DEPENDS Eigen
  INCLUDE_DIRS
    include
  LIBRARIES
    mil_tools
  CATKIN_DEPENDS
    roscpp
    cv_bridge
)

# Add folders to be run by python nosetests
if(CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(test)
endif()

catkin_python_setup()

include_directories(
    include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

add_library(mil_tools
  src/mil_tools/mil_tools.cpp
)
target_include_directories(mil_tools PUBLIC include)
target_link_libraries(mil_tools ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

