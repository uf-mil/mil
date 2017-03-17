cmake_minimum_required(VERSION 2.8.3)
project(navigator_vision)

# SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake;${CMAKE_MODULE_PATH}")
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
    navigator_msgs
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
    navigator_vision_lib
    navigator_sparsestereo
  CATKIN_DEPENDS
    roscpp
    rospy
    message_runtime
    std_msgs
    geometry_msgs
    sensor_msgs
    navigator_msgs
    navigator_tools
    pcl_ros
  DEPENDS
    system_lib
    image_transport
    image_geometry
    cv_bridge
    navigator_msgs
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
add_library( navigator_sparsestereo SHARED ${EXFAST_SOURCES})
target_include_directories(navigator_sparsestereo PUBLIC exFAST_SparseStereo/src)
target_link_libraries(navigator_sparsestereo ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_INCLUDE_DIRS})
set_target_properties(navigator_sparsestereo PROPERTIES COMPILE_FLAGS "-O3 -DNDEBUG -fopenmp -g -Wall -march=native -msse -msse2 -msse3 -mssse3 -msse4 -ffast-math -mfpmath=sse")

add_library(
  navigator_vision_lib
    src/navigator_vision_lib/visualization.cc
    src/navigator_vision_lib/cv_utils.cc
    src/navigator_vision_lib/image_filtering.cpp
    src/navigator_vision_lib/active_contours.cpp
    src/navigator_vision_lib/colorizer/pcd_colorizer.cpp
    src/navigator_vision_lib/colorizer/single_cloud_processor.cpp
    src/navigator_vision_lib/colorizer/camera_observer.cpp
    src/navigator_vision_lib/colorizer/color_observation.cpp
)
target_include_directories(navigator_vision_lib PUBLIC include/navigator_vision_lib)
target_link_libraries(navigator_vision_lib navigator_tools_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_COMMON_LIBRARIES})

add_executable(stereo_point_cloud_driver nodes/stereo_point_cloud_driver.cpp)
add_dependencies(stereo_point_cloud_driver navigator_vision_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  stereo_point_cloud_driver
    navigator_sparsestereo
    navigator_vision_lib
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_INCLUDE_DIRS}
)
set_target_properties(stereo_point_cloud_driver PROPERTIES COMPILE_FLAGS "-O3 -DNDEBUG -fopenmp -g -Wall -march=native -msse -msse2 -msse3 -mssse3 -msse4 -ffast-math -mfpmath=sse")


add_executable(camera_stream_demo nodes/camera_stream_demo.cpp)
add_dependencies(camera_stream_demo navigator_vision_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  camera_stream_demo
    navigator_vision_lib
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(underwater_shape_identification_vision 
  nodes/underwater_shape_identification_vision.cpp
  src/missions/underwater_shape_identification.cpp
)
add_dependencies(underwater_shape_identification_vision navigator_vision_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  underwater_shape_identification_vision
    navigator_vision_lib
    navigator_tools_lib
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(A_C_test_vision 
  nodes/A_C_test_vision.cpp
)
add_dependencies(A_C_test_vision navigator_vision_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  A_C_test_vision
    navigator_vision_lib
    navigator_tools_lib
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(shape_identification
  nodes/shape_identification/main.cpp
  nodes/shape_identification/DockShapeVision.cpp
  nodes/shape_identification/GrayscaleContour/GrayscaleContour.cpp
  nodes/shape_identification/TrackedShape.cpp
  nodes/shape_identification/ShapeTracker.cpp
)
add_dependencies(shape_identification
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(shape_identification
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
add_executable(camera_to_lidar
  nodes/camera_to_lidar/main.cpp
  nodes/camera_to_lidar/CameraLidarTransformer.cpp
)
add_dependencies(camera_to_lidar
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(camera_to_lidar
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(velodyne_pcd_colorizer nodes/velodyne_pcd_colorizer.cpp)
add_dependencies(velodyne_pcd_colorizer navigator_vision_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  velodyne_pcd_colorizer
    navigator_vision_lib
    navigator_tools_lib
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_INCLUDE_DIRS}
)

install(TARGETS navigator_vision_lib navigator_sparsestereo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
install(
  TARGETS
)
