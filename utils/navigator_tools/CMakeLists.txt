cmake_minimum_required(VERSION 2.8.3)
project(navigator_tools)
find_package(catkin REQUIRED COMPONENTS rostest std_msgs dynamic_reconfigure)
catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/BoundsConfig.cfg
)

# make sure configure headers are built before any node using them
#add_dependencies(lidar ${PROJECT_NAME}_gencfg)

catkin_package()

# Add folders to be run by python nosetests
if(CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(tests)
endif()
