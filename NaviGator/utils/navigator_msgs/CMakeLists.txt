cmake_minimum_required(VERSION 3.5)
project(navigator_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(genmsg REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(actionlib REQUIRED)
find_package(sensor_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 14)

add_action_files(
  DIRECTORY action
  FILES
  ClassifyUnknown.action
  ShooterDo.action
  )
rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/Hosts.msg
        msg/ColoramaDebug.msg
        msg/PerceptionObject.msg
        msg/Acceleration.msg
        msg/PerceptionObjectArray.msg
        msg/PoseTwistStamped.msg
        msg/Host.msg
        msg/DockShape.msg
        msg/PoseTwist.msg
        msg/ScanTheCode.msg
        msg/KillStatus.msg
        msg/DockShapes.msg
        msg/Coordinates.msg
        action/ShooterDo.action
        action/ClassifyUnknown.action
        srv/KeyboardControl.srv
        srv/MessageReactReport.srv
        srv/ObjectDBQuery.srv
        srv/CameraDBQuery.srv
        srv/ChooseAnimal.srv
        srv/GetDockShape.srv
        srv/ShooterManual.srv
        srv/MessageUAVSearchReport.srv
        srv/VisionRequest.srv
        srv/MoveToWaypoint.srv
        srv/ScanTheCodeMission.srv
        srv/MessageDetectDock.srv
        srv/StereoShapeDetector.srv
        srv/SetFrequency.srv
        srv/CameraToLidarTransform.srv
        srv/GetDockBays.srv
        srv/MessageFollowPath.srv
        srv/FindPinger.srv
        srv/ColorRequest.srv
        srv/TwoClosestCones.srv
        srv/StartGate.srv
        srv/CoordinateConversion.srv
        srv/MessageEntranceExitGate.srv
        srv/MessageUAVReplenishment.srv
        srv/SetROI.srv
        srv/MessageFindFling.srv
        srv/GetDockShapes.srv
        srv/AcousticBeacon.srv
    DEPENDENCIES
        geometry_msgs
        std_msgs
        actionlib_msgs
        sensor_msgs
)

#catkin_package(
	#CATKIN_DEPENDS
	#std_msgs
	#geometry_msgs
  #message_runtime
  #actionlib_msgs
#)
ament_export_dependencies(std_msgs geometry_msgs sensor_msgs)
ament_package()
