cmake_minimum_required(VERSION 3.0.2)
project(mil_missions)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -pedantic -Wall -std=c++11")

find_package(catkin REQUIRED rostest genmsg actionlib_msgs roscpp actionlib)
catkin_python_setup()
add_action_files(
  FILES
    DoMission.action
)

generate_messages(
  DEPENDENCIES
    std_msgs
    actionlib_msgs
)
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    mil_missions
  CATKIN_DEPENDS
    actionlib_msgs
    roscpp
)

include_directories(
    include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/import_mil_missions_examples.test)
    add_rostest(test/run_mission_py.test)
    add_rostest_gtest(run_test_mission_cpp
                      test/run_mission_cpp.test
                      test/test_run_mission.cpp
                      src/client.cpp)
    target_link_libraries(run_test_mission_cpp ${catkin_LIBRARIES})
endif()

add_library(mil_missions
  src/client.cpp
)
target_include_directories(mil_missions PUBLIC include)
target_link_libraries(mil_missions ${catkin_LIBRARIES})
