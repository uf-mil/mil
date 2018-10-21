cmake_minimum_required(VERSION 2.8.3)
project(point_stamped_and_odom_to_navsatfix)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
)


catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(point_stamped_and_odom_to_navsatfix src/point_stamped_and_odom_to_navsatfix.cpp)
target_link_libraries(point_stamped_and_odom_to_navsatfix ${catkin_LIBRARIES})
