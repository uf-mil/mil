cmake_minimum_required(VERSION 2.8.3)
project(navigator_perception)

SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake;${CMAKE_MODULE_PATH}")
# set c++11 as default, overide with set_target_properties()
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
    sub8_build_tools
    sub8_msgs
    message_generation
    std_msgs
    geometry_msgs
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
    include
    exFAST_SparseStereo/src/sparsestereo
  LIBRARIES
    sparsestereo
  CATKIN_DEPENDS
    roscpp
    rospy
    sub8_msgs
    message_runtime
    std_msgs
    geometry_msgs
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
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)


add_library( sparsestereo SHARED
  exFAST_SparseStereo/src/sparsestereo/calibrationresult.h
  exFAST_SparseStereo/src/sparsestereo/calibrationresult.cpp
  exFAST_SparseStereo/src/sparsestereo/censuswindow.h
  exFAST_SparseStereo/src/sparsestereo/exception.h
  exFAST_SparseStereo/src/sparsestereo/extendedfast.cpp
  exFAST_SparseStereo/src/sparsestereo/extendedfast.h
  exFAST_SparseStereo/src/sparsestereo/fast9.h
  exFAST_SparseStereo/src/sparsestereo/fast9-inl.h
  exFAST_SparseStereo/src/sparsestereo/hammingdistance.cpp
  exFAST_SparseStereo/src/sparsestereo/hammingdistance.h
  exFAST_SparseStereo/src/sparsestereo/simd.h
  exFAST_SparseStereo/src/sparsestereo/simd.cpp
  exFAST_SparseStereo/src/sparsestereo/sparsematch.h
  exFAST_SparseStereo/src/sparsestereo/sparserectification.cpp
  exFAST_SparseStereo/src/sparsestereo/sparserectification.h
  exFAST_SparseStereo/src/sparsestereo/sparsestereo.h
  exFAST_SparseStereo/src/sparsestereo/sparsestereo-inl.h
  exFAST_SparseStereo/src/sparsestereo/stereorectification.cpp
  exFAST_SparseStereo/src/sparsestereo/stereorectification.h
  exFAST_SparseStereo/src/sparsestereo/imageconversion.h
  exFAST_SparseStereo/src/sparsestereo/imageconversion.cpp
  exFAST_SparseStereo/src/sparsestereo/census.h
  exFAST_SparseStereo/src/sparsestereo/census-inl.h
  exFAST_SparseStereo/src/sparsestereo/census.cpp
)

target_link_libraries(
  sparsestereo
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_INCLUDE_DIRS}
)

set_target_properties(sparsestereo PROPERTIES COMPILE_FLAGS "-O3 -DNDEBUG -fopenmp -g -Wall -march=native -msse -msse2 -msse3 -mssse3 -msse4 -ffast-math -mfpmath=sse")

install(TARGETS sparsestereo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS
)
