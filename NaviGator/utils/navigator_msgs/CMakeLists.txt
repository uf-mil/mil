cmake_minimum_required(VERSION 3.0.2)
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
  DroneTarget.msg
  DroneTin.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  AcousticBeacon.srv
  ChooseAnimal.srv
  MoveToWaypoint.srv
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
  MessageDetectDock.srv
  MessageEntranceExitGate.srv
  MessageFindFling.srv
  MessageFollowPath.srv
  MessageReactReport.srv
  MessageUAVReplenishment.srv
  MessageUAVSearchReport.srv
  TwoClosestCones.srv
  # 2024 RobotX service files
  BallLauncherDrops.srv
  DroneMission.srv
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
