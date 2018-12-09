cmake_minimum_required(VERSION 2.8.3)
project(navigator_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  geometry_msgs
  genmsg
  actionlib_msgs
  actionlib
  sensor_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  Acceleration.msg
  PoseTwist.msg
  Coordinates.msg
  PoseTwistStamped.msg
  PerceptionObject.msg
  PerceptionObjectArray.msg
  DockShape.msg
  DockShapes.msg
  KillStatus.msg
  Host.msg
  Hosts.msg
  ColoramaDebug.msg
  ScanTheCode.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  ShooterManual.srv
  VisionRequest.srv
  GetDockShape.srv
  GetDockShapes.srv
  StartGate.srv
  SetROI.srv
  CoordinateConversion.srv
  StereoShapeDetector.srv
  ScanTheCodeMission.srv
  ObjectDBQuery.srv
  CameraDBQuery.srv
  CameraToLidarTransform.srv
  KeyboardControl.srv
  ColorRequest.srv
  FindPinger.srv
  SetFrequency.srv
  GetDockBays.srv
  MessageDetectDeliver.srv
  MessageExtranceExitGate.srv
  MessageIdentifySymbolsDock.srv
)

add_action_files(
  DIRECTORY action
  FILES
  ClassifyUnknown.action
  ShooterDo.action
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  sensor_msgs
)

#catkin_package(
	#CATKIN_DEPENDS
	#std_msgs
	#geometry_msgs
  #message_runtime
  #actionlib_msgs
#)
