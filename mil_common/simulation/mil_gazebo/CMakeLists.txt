cmake_minimum_required(VERSION 3.0.2)
project(mil_gazebo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  gazebo_ros
  mil_msgs
  point_cloud_object_detection_and_recognition
  mil_passive_sonar
)

catkin_package(INCLUDE_DIRS include)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    include
)

# Shared Utilities
add_library(
    mil_gazebo_utils
        src/mil_gazebo_utils.cpp
)
target_link_libraries(mil_gazebo_utils
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(mil_gazebo_utils ${catkin_EXPORTED_TARGETS})


add_library(
    mil_passive_sonar_gazebo
        src/mil_passive_sonar_gazebo.cpp
)
target_link_libraries(mil_passive_sonar_gazebo
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(mil_passive_sonar_gazebo ${catkin_EXPORTED_TARGETS})

# Add buoyancy plugin
add_library(
    mil_buoyancy_gazebo
        src/mil_buoyancy_gazebo.cpp
)
target_link_libraries(mil_buoyancy_gazebo
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  mil_gazebo_utils
)
add_dependencies(mil_buoyancy_gazebo mil_gazebo_utils ${catkin_EXPORTED_TARGETS})

# Add DVL plugin
add_library(
    mil_dvl_gazebo
        src/mil_dvl_gazebo.cpp
)
target_link_libraries(mil_dvl_gazebo
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  mil_gazebo_utils
)
add_dependencies(mil_dvl_gazebo mil_gazebo_utils ${catkin_EXPORTED_TARGETS})

# Add Magnetometer plugin
add_library(
    mil_magnetometer_gazebo
        src/mil_magnetometer_gazebo.cpp
)
target_link_libraries(mil_magnetometer_gazebo
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  mil_gazebo_utils
)
add_dependencies(mil_magnetometer_gazebo mil_gazebo_utils ${catkin_EXPORTED_TARGETS})

# Add Depth plugin
add_library(
    mil_depth_gazebo
        src/mil_depth_gazebo.cpp
)
target_link_libraries(mil_depth_gazebo
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  mil_gazebo_utils
)
add_dependencies(mil_depth_gazebo mil_gazebo_utils ${catkin_EXPORTED_TARGETS})

# Add IMU Plugin
add_library(
    mil_imu_gazebo
        src/mil_imu_gazebo.cpp
)
target_link_libraries(mil_imu_gazebo
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  mil_gazebo_utils
)
add_dependencies(mil_imu_gazebo mil_gazebo_utils ${catkin_EXPORTED_TARGETS})

#Add Inclinometer Plugin
add_library(
    mil_inclinometer_gazebo
        src/mil_inclinometer_gazebo.cpp
)
target_link_libraries(mil_inclinometer_gazebo
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  mil_gazebo_utils
)
add_dependencies(mil_inclinometer_gazebo mil_gazebo_utils ${catkin_EXPORTED_TARGETS})

# Add Drag plugin
add_library(
    mil_drag_gazebo
        src/mil_drag_gazebo.cpp
)
target_link_libraries(mil_drag_gazebo
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  mil_gazebo_utils
)
add_dependencies(mil_drag_gazebo mil_gazebo_utils ${catkin_EXPORTED_TARGETS})


# Add pcodar_gazebo library
add_library(
    pcodar_gazebo
        src/pcodar_gazebo.cpp
)
target_link_libraries(pcodar_gazebo
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(pcodar_gazebo
  ${catkin_EXPORTED_TARGETS}
)
