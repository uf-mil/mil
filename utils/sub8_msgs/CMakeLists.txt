cmake_minimum_required(VERSION 2.8.3)
project(sub8_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  message_runtime
  rospy
  std_msgs
)

# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
#   std_msgs
# )

catkin_package(
    LIBRARIES sub8_msgs
    CATKIN_DEPENDS geometry_msgs message_generation rospy std_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)