cmake_minimum_required(VERSION 2.8.3)
project(navigator_mission_planner)

find_package(catkin REQUIRED
	nav_msgs
	actionlib_msgs
	navigator_msgs
  	rostest
	)

catkin_python_setup()

add_action_files(
	  FILES
	  current_goal.action
	)

generate_messages(DEPENDENCIES 
	actionlib_msgs 
	navigator_msgs
)

catkin_package(CATKIN_DEPENDS 
	actionlib_msgs 
	nav_msgs
	navigator_msgs
	)



if(CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(tests)
endif()

