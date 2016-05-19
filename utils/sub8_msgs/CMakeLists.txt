cmake_minimum_required(VERSION 2.8.3)
project(sub8_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  message_runtime
  rospy
  std_msgs
  sensor_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  Alarm.msg
  Thrust.msg
  ThrusterCmd.msg
  ThrusterStatus.msg
  Waypoint.msg
  Trajectory.msg
  VelocityMeasurement.msg
  VelocityMeasurements.msg
  Path.msg
  PathPoint.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  ThrusterInfo.srv
  MotionPlan.srv
  UpdateThrusterLayout.srv
  FailThruster.srv
  VisionRequest.srv
  VisionRequest2D.srv
  BMatrix.srv
  PathPlan.srv
  SearchPose.srv
  SetValve.srv
)

# add_action_files(
#   FILES
#   Action1.action
# )

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS geometry_msgs message_generation rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
