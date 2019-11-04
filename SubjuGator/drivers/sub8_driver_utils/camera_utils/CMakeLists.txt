cmake_minimum_required(VERSION 2.8.3)
project(camera_utils)

# This builds a utility to reset the camera bus using the DC1394 camera libraries
# Since we are moving away from dc1394 towards pointgrey drivers. This utility is 
# no longer applicable. HoWever, we may need to create something similar using the
# flycapture SDK

#find_package(DC1394 REQUIRED)
#catkin_package(DEPENDS system_lib)

#FIND_PATH( DC1394_INCLUDE_DIR dc1394/dc1394.h)
#FIND_LIBRARY( DC1394_LIBRARIES dc1394)
#INCLUDE(FindPackageHandleStandardArgs)
#FIND_PACKAGE_HANDLE_STANDARD_ARGS(DC1394 DEFAULT_MSG DC1394_LIBRARIES DC1394_INCLUDE_DIR)
#MARK_AS_ADVANCED(DC1394_INCLUDE_DIR DC1394_LIBRARIES)

#include_directories(
#  ${catkin_INCLUDE_DIRS}
#  ${DC1394_INCLUDE_DIR}
#)

#add_executable(dc1394_reset src/dc1394_reset_bus.c)
#target_link_libraries(dc1394_reset ${catkin_LIBRARIES} ${DC1394_LIBRARIES})
#set_target_properties(dc1394_reset PROPERTIES COMPILE_FLAGS "-std=c99")
