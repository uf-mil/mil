cmake_minimum_required(VERSION 2.8)
project(camera1394)

# Find catkin macros and libraries, also other catkin packages.
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  diagnostic_updater
  driver_base
  dynamic_reconfigure
  image_transport
  message_generation
  nodelet
  roscpp
  rostest
  sensor_msgs
  tf)

# This driver uses Boost threads
find_package(Boost REQUIRED COMPONENTS thread)

# Resolve system dependency on libdc1394, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(DC1394_PKGCONF REQUIRED libdc1394-2)
find_path(DC1394_INCLUDE_DIR
  NAMES dc1394.h
  PATHS ${DC1394_PKGCONF_INCLUDE_DIRS}
)
find_library(DC1394_LIBRARY
  NAMES dc1394
  PATHS ${DC1394_PKGCONF_LIBRARY_DIRS}
)

add_service_files(FILES
  GetCameraRegisters.srv
  SetCameraRegisters.srv)

generate_messages(DEPENDENCIES std_msgs)

# auto-generate dynamic reconfiguration GUI before catkin_package()
generate_dynamic_reconfigure_options(cfg/Camera1394.cfg)

# nothing builds against this package
catkin_package(CATKIN_DEPENDS
                 camera_info_manager
                 diagnostic_updater
                 dynamic_reconfigure
                 image_transport
                 message_runtime
                 nodelet
                 roscpp
                 sensor_msgs
                 tf)

include_directories(${catkin_INCLUDE_DIRS}
  ${DC1394_PKGCONF_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR})

# build package sources
add_subdirectory(src/nodes)

# install pluginlib definition for the nodelet
install(FILES camera1394_nodelet.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# driver unit tests
if (CATKIN_ENABLE_TESTING)
  #
  #  * When -DUSE_DEVICE=1 is configured: run tests that require some
  #    real 1394 camera to be present.
  #
  #  * Otherwise, run only tests that require no attached camera, as is
  #    done on the ROS build machines.
  #
  if (${USE_DEVICE})
    add_rostest(tests/camera_node_hz.test)
    add_rostest(tests/camera_nodelet_hz.test)
  endif()

  if (${USE_TRIGGER})
    # build the software trigger node, but do not install it
    add_executable(camera1394_trigger_node tests/trigger_node.cpp)
    target_link_libraries(camera1394_trigger_node
                          ${catkin_LIBRARIES}
                          ${DC1394_LIBRARY})
    add_rostest(tests/trigger_node.test)
    add_rostest(tests/trigger_nodelet.test)
  endif()
  
  add_rostest(tests/no_device_node.test)
  add_rostest(tests/no_device_nodelet.test)
endif()
