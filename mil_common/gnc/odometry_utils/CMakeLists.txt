cmake_minimum_required(VERSION 3.5)
project(odometry_utils)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(eigen_conversions REQUIRED)
find_package(odom_estimator REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package()
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include ${Boost_INCLUDE_DIRS})

add_library(transform_odometry src/transform_odometry.cpp)
ament_target_dependencies(transform_odometry
   rclcpp
   geometry_msgs
   odom_estimator
   eigen_conversions
   nav_msgs
   tf2
   tf2_ros
   tf2_eigen
   tf2_geometry_msgs)
rclcpp_components_register_nodes(transform_odometry "odometry_utils::TransformOdometry")
set_target_properties(transform_odometry PROPERTIES COMPILE_FLAGS "-O3 -std=c++17")

add_library(odometry_to_tf src/odometry_to_tf.cpp)
ament_target_dependencies(odometry_to_tf
   rclcpp
   geometry_msgs
   odom_estimator
   eigen_conversions
   nav_msgs
   tf2
   tf2_ros
   tf2_eigen
   tf2_geometry_msgs)
rclcpp_components_register_nodes(odometry_to_tf "odometry_utils::OdometryToTf")
set_target_properties(odometry_to_tf PROPERTIES COMPILE_FLAGS "-O3 -std=c++17")

# Install libraries
install(TARGETS transform_odometry odometry_to_tf
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, config files, etc.
# install(DIRECTORY
#   launch
#   config
#   DESTINATION share/${PROJECT_NAME}
# )

# Export dependencies
ament_export_dependencies(rclcpp geometry_msgs nav_msgs odom_estimator eigen_conversions tf2 tf2_ros tf2_eigen tf2_geometry_msgs)
ament_export_include_directories(include)
ament_export_libraries(transform_odometry odometry_to_tf)

# Add linters if desired
# find_package(ament_lint_auto REQUIRED)
# ament_lint_auto_find_test_dependencies()

ament_package()
