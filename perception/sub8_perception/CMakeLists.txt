cmake_minimum_required(VERSION 2.8.3)
project(sub8_perception)

SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake;${CMAKE_MODULE_PATH}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -g -rdynamic -Wall -std=c++11 ")

find_package(catkin
  REQUIRED COMPONENTS
    roscpp
    rospy
    eigen_conversions
    rostime
    image_transport
    image_geometry
    cv_bridge
    sub8_msgs
    message_generation
    std_msgs
    geometry_msgs
)

catkin_python_setup()

# find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    # exFAST_SparseStereo/src/sparsestereo
  LIBRARIES
    sub8_vision_lib
    sparsestereo
  CATKIN_DEPENDS
    roscpp
    rospy
    sub8_msgs
    message_runtime
    std_msgs
    geometry_msgs
  DEPENDS
    image_transport
    image_geometry
    cv_bridge
    sub8_msgs
)

include_directories(
    include
    # exFAST_SparseStereo/src
  SYSTEM
    # ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

add_library(sub8_vision_lib
    # src/sub8_vision_lib/align.cc
    src/sub8_vision_lib/cv_utils.cc
    # src/sub8_vision_lib/cv_param_helpers.cc
    src/sub8_vision_lib/visualization.cc
    # src/sub8_vision_lib/object_finder.cc
)

target_link_libraries(
  sub8_vision_lib
    ${catkin_LIBRARIES}
    # ${PCL_COMMON_LIBRARIES}
    # ${PCL_IO_LIBRARIES}
    # ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    # bfd
    # dl
)

# add_library( sparsestereo SHARED
#   # exFAST_SparseStereo/src/sparsestereo/calibrationresult.h
#   exFAST_SparseStereo/src/sparsestereo/calibrationresult.cpp
#   # exFAST_SparseStereo/src/sparsestereo/censuswindow.h
#   # exFAST_SparseStereo/src/sparsestereo/exception.h
#   exFAST_SparseStereo/src/sparsestereo/extendedfast.cpp
#   # exFAST_SparseStereo/src/sparsestereo/extendedfast.h
#   # exFAST_SparseStereo/src/sparsestereo/fast9.h
#   # exFAST_SparseStereo/src/sparsestereo/fast9-inl.h
#   exFAST_SparseStereo/src/sparsestereo/hammingdistance.cpp
#   # exFAST_SparseStereo/src/sparsestereo/hammingdistance.h
#   # exFAST_SparseStereo/src/sparsestereo/simd.h
#   exFAST_SparseStereo/src/sparsestereo/simd.cpp
#   # exFAST_SparseStereo/src/sparsestereo/sparsematch.h
#   exFAST_SparseStereo/src/sparsestereo/sparserectification.cpp
#   # exFAST_SparseStereo/src/sparsestereo/sparserectification.h
#   # exFAST_SparseStereo/src/sparsestereo/sparsestereo.h
#   # exFAST_SparseStereo/src/sparsestereo/sparsestereo-inl.h
#   exFAST_SparseStereo/src/sparsestereo/stereorectification.cpp
#   # exFAST_SparseStereo/src/sparsestereo/stereorectification.h
#   # exFAST_SparseStereo/src/sparsestereo/imageconversion.h
#   exFAST_SparseStereo/src/sparsestereo/imageconversion.cpp
#   # exFAST_SparseStereo/src/sparsestereo/census.h
#   # exFAST_SparseStereo/src/sparsestereo/census-inl.h
#   exFAST_SparseStereo/src/sparsestereo/census.cpp
# )

# target_link_libraries(
#   sparsestereo
#     ${catkin_LIBRARIES}
#     ${Boost_LIBRARIES}
#     ${OpenCV_INCLUDE_DIRS}
# )

# set_target_properties(sparsestereo PROPERTIES COMPILE_FLAGS "-O3 -DNDEBUG -fopenmp -g -Wall -march=native -msse -msse2 -msse3 -mssse3 -msse4 -ffast-math -mfpmath=sse")

install(TARGETS sub8_vision_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(TARGETS sparsestereo
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

# file(GLOB BUOY_SOURCES
#     "src/sub8_perception/*.cc"
# )

MESSAGE( STATUS "Include search path: " ${INCLUDE_DIRECTORIES} )

add_executable(
  torpedos_cpp
    nodes/torpedo_board.cc
)

add_dependencies(
  torpedos_cpp
   sub8_msgs_generate_messages_cpp
   ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  torpedos_cpp
    sub8_vision_lib
    # sparsestereo
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_INCLUDE_DIRS}
)

#add_executable(
#  perception_server
#    nodes/perception_server.cc
#)

#add_dependencies(
# perception_server
#   sub8_msgs_generate_messages_cpp
#   ${catkin_EXPORTED_TARGETS}
#)

#target_link_libraries(
#  perception_server
#    sub8_vision_lib
#    ${catkin_LIBRARIES}
#    ${Boost_LIBRARIES}
#    ${OpenCV_INCLUDE_DIRS}
#)

# add_executable(
#   pcl_buoy
#     nodes/buoy_node.cc
#     ${BUOY_SOURCES}
# )

# add_dependencies(
#   pcl_buoy
#    sub8_msgs_generate_messages_cpp
#    ${catkin_EXPORTED_TARGETS}
# )

# target_link_libraries(
#   pcl_buoy
#     sub8_vision_lib
#     ${catkin_LIBRARIES}
#     ${PCL_COMMON_LIBRARIES}
#     ${PCL_IO_LIBRARIES}
#     ${PCL_LIBRARIES}
#     ${Boost_LIBRARIES}
#     ${OpenCV_INCLUDE_DIRS}
#     bfd
#     dl
# )

install(
  TARGETS
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/path_marker.test)
  
  catkin_download_test_data(
    ${PROJECT_NAME}_pool_path2.bag
    https://ci.mil.ufl.edu/unit_test_data/subjugator/follow_orange_pipes/pool_path2.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests/follow_orange_pipes
    MD5 5b376a5c4687a5b44e9c67959aa4b570)

  catkin_download_test_data(
    ${PROJECT_NAME}_pool_path3.bag
    https://ci.mil.ufl.edu/unit_test_data/subjugator/follow_orange_pipes/pool_path3.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests/follow_orange_pipes
    MD5 da78e11db71101b5f5a9ae004aaa083c)

  catkin_download_test_data(
    ${PROJECT_NAME}_transdec_path1.bag
    https://ci.mil.ufl.edu/unit_test_data/subjugator/follow_orange_pipes/transdec_path1.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests/follow_orange_pipes
    MD5 cf69477332c0b16cba5d1c9cea9acadd)

  catkin_download_test_data(
    ${PROJECT_NAME}_transdec_path3.bag
    https://ci.mil.ufl.edu/unit_test_data/subjugator/follow_orange_pipes/transdec_path3.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests/follow_orange_pipes
    MD5 5af392f1636a8fe038be865e5796e2a4)
endif()
