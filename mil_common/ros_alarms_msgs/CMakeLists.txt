#File updated for MIL ROS2 Conversion (Spring 2024)
#Referencing:
# - https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html
# - https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1

cmake_minimum_required(VERSION 3.5)
project(ros_alarms_msgs)

# If no C++ standard is provided, compile as C++17 (supported in ROS2 Humble and newer).
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Alarm.msg"
  "msg/Alarms.msg"
  "srv/AlarmGet.srv"
  "srv/AlarmSet.srv"
  DEPENDENCIES std_msgs
)

ament_package()
