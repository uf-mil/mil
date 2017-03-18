cmake_minimum_required(VERSION 2.8.3)
project(mil_vision)

# set c++11 as default, overide with set_target_properties()
# if needed for specific nodes (cough..cough... PCL)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -g -pedantic -Wall -std=c++11 ")

find_package(catkin
  REQUIRED COMPONENTS
    roscpp
    rospy
    eigen_conversions
    rostime
    image_transport
    image_geometry
    cv_bridge
    message_generation
    mil_msgs
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    tf
    pcl_ros
    tf2_sensor_msgs
    tf2_geometry_msgs
)

find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED date_time filesystem)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    mil_vision_lib
    mil_sparsestereo
  CATKIN_DEPENDS
    roscpp
    rospy
    message_runtime
    std_msgs
    geometry_msgs
    sensor_msgs
    mil_msgs
    mil_tools
    pcl_ros
  DEPENDS
    system_lib
    image_transport
    image_geometry
    cv_bridge
    mil_msgs
)

include_directories(
    include
  SYSTEM
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

FILE(GLOB EXFAST_SOURCES exFAST_SparseStereo/src/sparsestereo/*)
add_library( mil_sparsestereo SHARED ${EXFAST_SOURCES})
target_include_directories(mil_sparsestereo PUBLIC exFAST_SparseStereo/src)
target_link_libraries(mil_sparsestereo ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_INCLUDE_DIRS})
set_target_properties(mil_sparsestereo PROPERTIES COMPILE_FLAGS "-O3 -DNDEBUG -fopenmp -g -Wall -march=native -msse -msse2 -msse3 -mssse3 -msse4 -ffast-math -mfpmath=sse")

add_library(
  mil_vision_lib
    src/mil_vision_lib/visualization.cc
    src/mil_vision_lib/cv_utils.cc
    src/mil_vision_lib/image_filtering.cpp
    src/mil_vision_lib/active_contours.cpp
    src/mil_vision_lib/colorizer/pcd_colorizer.cpp
    src/mil_vision_lib/colorizer/single_cloud_processor.cpp
    src/mil_vision_lib/colorizer/camera_observer.cpp
    src/mil_vision_lib/colorizer/color_observation.cpp
)
target_include_directories(mil_vision_lib PUBLIC include/mil_vision_lib)
target_link_libraries(mil_vision_lib mil_tools_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_COMMON_LIBRARIES})

add_executable(camera_stream_demo nodes/camera_stream_demo.cpp)
add_dependencies(camera_stream_demo mil_vision_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  camera_stream_demo
    mil_vision_lib
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_INCLUDE_DIRS}
)

