# common source files for both node and nodelet driver interfaces
set(CAMERA1394_SOURCES 
  driver1394.cpp
  dev_camera1394.cpp
  features.cpp
  format7.cpp
  modes.cpp
  registers.cpp
  trigger.cpp)

## The following used to be necessary rosbuild on Mac OSX.
## No telling what, if anything, is needed with catkin.

# Mac OSX requires extra linker parameters
#if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
#  set(DC1394 dc1394 "-framework CoreServices")
#else(CMAKE_SYSTEM_NAME MATCHES "Darwin")
#  set(DC1394 dc1394)
#endif(CMAKE_SYSTEM_NAME MATCHES "Darwin")

# build and install the node
add_executable(camera1394_node camera1394_node.cpp ${CAMERA1394_SOURCES})
add_dependencies(camera1394_node ${camera1394_EXPORTED_TARGETS})
target_link_libraries(camera1394_node
  ${catkin_LIBRARIES} ${DC1394_LIBRARY})
install(TARGETS camera1394_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# build and install the nodelet
add_library(camera1394_nodelet nodelet.cpp ${CAMERA1394_SOURCES})
add_dependencies(camera1394_nodelet ${camera1394_EXPORTED_TARGETS})
target_link_libraries(camera1394_nodelet
  ${catkin_LIBRARIES} ${DC1394_LIBRARY})
install(TARGETS camera1394_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
