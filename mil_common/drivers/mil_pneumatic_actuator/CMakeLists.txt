cmake_minimum_required(VERSION 3.5)
project(mil_pneumatic_actuator)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)

# Generate ROS interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetValve.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
