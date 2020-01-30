# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(navigator_emergency_control)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS sensor_msgs arm_bootloader libopencm3 uf_subbus_protocol kill_handling message_generation)
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory

# CATKIN_MIGRATION: removed during catkin migration
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory

# CATKIN_MIGRATION: removed during catkin migration
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_find_ros_package(libopencm3)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_find_ros_package(uf_subbus_protocol)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_find_ros_package(arm_bootloader)


## Generate added messages and services with any dependencies listed here
#generate_messages(
#    DEPENDENCIES 
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS  # TODO
    CATKIN_DEPENDS sensor_msgs arm_bootloader libopencm3 uf_subbus_protocol kill_handling message_runtime
    INCLUDE_DIRS include# TODO include
    LIBRARIES # TODO
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
add_custom_target(navigator_emergency_control_bootloader ALL
    COMMAND $(MAKE) -C ${PROJECT_SOURCE_DIR}/bootloader BUILD_DIR=${CMAKE_CURRENT_BINARY_DIR}/bootloader_out TOOLCHAIN_DIR=${libopencm3_TOOLCHAIN_DIR} UF_SUBBUS_PROTOCOL_DIR=${uf_subbus_protocol_PACKAGE_PATH} ARM_BOOTLOADER_DIR=${arm_bootloader_PACKAGE_PATH}
)
add_dependencies(navigator_emergency_control_bootloader ${catkin_EXPORTED_TARGETS})

add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/firmware_out/firmware.bin.dummy
    COMMAND $(MAKE) -C ${PROJECT_SOURCE_DIR}/firmware BUILD_DIR=${CMAKE_CURRENT_BINARY_DIR}/firmware_out TOOLCHAIN_DIR=${libopencm3_TOOLCHAIN_DIR} UF_SUBBUS_PROTOCOL_DIR=${uf_subbus_protocol_PACKAGE_PATH} ARM_BOOTLOADER_DIR=${arm_bootloader_PACKAGE_PATH}
)
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/firmware_out/firmware.bin.dummy APPEND DEPENDS ${catkin_EXPORTED_TARGETS})
add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/firmware_bin.cpp
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/firmware_out/firmware.bin.dummy
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/firmware_out
    COMMAND xxd -i firmware.bin > ../firmware_bin.cpp.tmp
    COMMAND cmp -s ../firmware_bin.cpp.tmp ../firmware_bin.cpp || cp ../firmware_bin.cpp.tmp ../firmware_bin.cpp
)
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/firmware_bin.cpp APPEND DEPENDS ${catkin_EXPORTED_TARGETS})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
add_executable(navigator_emergency_control src/navigator_emergency.cpp firmware_bin.cpp)
target_link_libraries(navigator_emergency_control ${catkin_LIBRARIES})
add_dependencies(navigator_emergency_control ${catkin_EXPORTED_TARGETS})
add_dependencies(navigator_emergency_control kill_handling_generate_messages_cpp)
find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
set_target_properties(navigator_emergency_control PROPERTIES COMPILE_FLAGS -std=c++0x)
#target_link_libraries(example ${PROJECT_NAME})
