cmake_minimum_required(VERSION 2.8.3)
project(navigator_perception)

# SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake;${CMAKE_MODULE_PATH}")
# set c++11 as default, overide with set_target_properties()
# if needed for specific nodes (cough..cough... PCL)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -g -Wall -std=c++11 ")

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
)

find_package(PCL 1.7 REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
    include
    exFAST_SparseStereo/src/sparsestereo
  LIBRARIES
    navigator_vision_lib
    sparsestereo
  CATKIN_DEPENDS
    roscpp
    rospy
    message_runtime
    std_msgs
    geometry_msgs
    sensor_msgs
    navigator_msgs
  DEPENDS
    system_lib
    image_transport
    image_geometry
    cv_bridge
    navigator_msgs
)

include_directories(
    include
    exFAST_SparseStereo/src
  SYSTEM
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(EXFAST_SRC_DIR "exFAST_SparseStereo/src/sparsestereo")
add_library( navigator_sparsestereo SHARED
  ${EXFAST_SRC_DIR}/calibrationresult.h
  ${EXFAST_SRC_DIR}/calibrationresult.cpp
  ${EXFAST_SRC_DIR}/censuswindow.h
  ${EXFAST_SRC_DIR}/exception.h
  ${EXFAST_SRC_DIR}/extendedfast.cpp
  ${EXFAST_SRC_DIR}/extendedfast.h
  ${EXFAST_SRC_DIR}/fast9.h
  ${EXFAST_SRC_DIR}/fast9-inl.h
  ${EXFAST_SRC_DIR}/hammingdistance.cpp
  ${EXFAST_SRC_DIR}/hammingdistance.h
  ${EXFAST_SRC_DIR}/simd.h
  ${EXFAST_SRC_DIR}/simd.cpp
  ${EXFAST_SRC_DIR}/sparsematch.h
  ${EXFAST_SRC_DIR}/sparserectification.cpp
  ${EXFAST_SRC_DIR}/sparserectification.h
  ${EXFAST_SRC_DIR}/sparsestereo.h
  ${EXFAST_SRC_DIR}/sparsestereo-inl.h
  ${EXFAST_SRC_DIR}/stereorectification.cpp
  ${EXFAST_SRC_DIR}/stereorectification.h
  ${EXFAST_SRC_DIR}/imageconversion.h
  ${EXFAST_SRC_DIR}/imageconversion.cpp
  ${EXFAST_SRC_DIR}/census.h
  ${EXFAST_SRC_DIR}/census-inl.h
  ${EXFAST_SRC_DIR}/census.cpp
)

target_link_libraries(
  navigator_sparsestereo
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_INCLUDE_DIRS}
)

set_target_properties(navigator_sparsestereo PROPERTIES COMPILE_FLAGS "-O3 -DNDEBUG -fopenmp -g -Wall -march=native -msse -msse2 -msse3 -mssse3 -msse4 -ffast-math -mfpmath=sse")

install(TARGETS navigator_sparsestereo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(
  navigator_vision_lib
    src/navigator_vision_lib/visualization.cc
    src/navigator_vision_lib/cv_utils.cc
    src/navigator_vision_lib/point_cloud_algorithms.cc
)

target_link_libraries(

  navigator_vision_lib
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

install(TARGETS navigator_vision_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)


add_executable(
  stereo_point_cloud_driver
    nodes/stereo_point_cloud_driver.cpp
)

add_dependencies(
  stereo_point_cloud_driver
   navigator_msgs_generate_messages_cpp
   ${catkin_EXPORTED_TARGETS}
)

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


install(
  TARGETS
)
