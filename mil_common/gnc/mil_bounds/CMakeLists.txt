cmake_minimum_required(VERSION 2.8.3)
project(mil_bounds)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/BoundsConfig.cfg
)

catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/bounds.test DEPENDENCIES ${PROJECT_NAME}_EXPORTED_TARGETS)
endif()
