cmake_minimum_required(VERSION 2.8.3)
project(mil_msgs)
find_package(catkin
  REQUIRED COMPONENTS
    message_generation
    message_runtime
    geometry_msgs
    sensor_msgs
    actionlib
    interactive_markers
    std_msgs
    actionlib_msgs
   )

add_action_files(FILES
  MoveTo.action
  BagOnline.action
)

add_message_files(FILES
  DepthStamped.msg
  LabeledObject.msg
  LabeledObjects.msg
  RangeStamped.msg
  Rectangle3D.msg
  PerceptionObject.msg
  PerceptionObjectArray.msg
  PoseTwist.msg
  PoseTwistStamped.msg
  VelocityMeasurement.msg
  VelocityMeasurements.msg
)

add_service_files(FILES
  Bounds.srv
  CameraToLidarTransform.srv
  ObjectDBQuery.srv
  SetGeometry.srv
  ObjectDBQuery.srv
)

generate_messages(
    DEPENDENCIES std_msgs actionlib_msgs geometry_msgs sensor_msgs
)
catkin_package(
    CATKIN_DEPENDS message_generation message_runtime geometry_msgs actionlib std_msgs actionlib_msgs
)

