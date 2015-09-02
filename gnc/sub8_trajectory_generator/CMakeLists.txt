cmake_minimum_required(VERSION 2.8.3)
project(sub8_trajectory_generator)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  message_generation
)

## Boost 
# find_package(Boost REQUIRED COMPONENTS system)

# add_message_files(
# )

# add_service_files(
# )

# add_action_files(
# )

# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
# )

##############
## INCLUDES ##
##############

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

############
## BUILD  ##
############

# add_executable(sub8_trajectory_generator_node src/sub8_trajectory_generator_node.cpp)
# add_dependencies(sub8_trajectory_generator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(sub8_trajectory_generator_node
#   ${catkin_LIBRARIES}
# )

#############
## Testing ##
#############

# catkin_add_gtest(${PROJECT_NAME}-test test/test_sub8_trajectory_generator.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
