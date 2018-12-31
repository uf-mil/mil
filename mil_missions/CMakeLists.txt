cmake_minimum_required(VERSION 2.8.3)
project(mil_missions)
find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)
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
catkin_package()

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/import_mil_missions_examples.test)
    add_rostest(test/run_mission.test)
endif()
