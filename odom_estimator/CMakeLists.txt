# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(odom_estimator)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp pluginlib rawgps_common sensor_msgs eigen_conversions geometry_msgs tf nodelet tf_conversions std_msgs message_generation)
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

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
## Generate added messages and services with any dependencies listed here
add_message_files(
    FILES
    Info.msg
)
generate_messages(
    DEPENDENCIES std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS Eigen # TODO
    CATKIN_DEPENDS roscpp pluginlib rawgps_common sensor_msgs eigen_conversions geometry_msgs tf nodelet tf_conversions std_msgs message_runtime
    INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS} include# TODO include
    LIBRARIES # TODO
)

include_directories(${EIGEN_INCLUDE_DIRS} include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
add_library(odom_estimator_nodelet src/nodelet.cpp)
target_link_libraries(odom_estimator_nodelet ${catkin_LIBRARIES})
add_dependencies(odom_estimator_nodelet ${catkin_EXPORTED_TARGETS})
add_dependencies(odom_estimator_nodelet ${PROJECT_NAME}_generate_messages_cpp)
set_target_properties(odom_estimator_nodelet PROPERTIES COMPILE_FLAGS -std=c++0x)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
add_executable(test_unscented_transform src/test_unscented_transform.cpp)
target_link_libraries(test_unscented_transform ${catkin_LIBRARIES})
add_dependencies(test_unscented_transform ${catkin_EXPORTED_TARGETS})
set_target_properties(test_unscented_transform PROPERTIES COMPILE_FLAGS -std=c++0x)
add_executable(test_gps src/test_gps.cpp)
target_link_libraries(test_gps ${catkin_LIBRARIES})
add_dependencies(test_gps ${catkin_EXPORTED_TARGETS})
set_target_properties(test_gps PROPERTIES COMPILE_FLAGS -std=c++0x)
add_executable(test_dual_gps src/test_dual_gps.cpp)
target_link_libraries(test_dual_gps ${catkin_LIBRARIES})
add_dependencies(test_dual_gps ${catkin_EXPORTED_TARGETS})
set_target_properties(test_dual_gps PROPERTIES COMPILE_FLAGS -std=c++0x)
add_executable(test_mag src/test_mag.cpp)
target_link_libraries(test_mag ${catkin_LIBRARIES})
add_dependencies(test_mag ${catkin_EXPORTED_TARGETS})
set_target_properties(test_mag PROPERTIES COMPILE_FLAGS -std=c++0x)
#target_link_libraries(example ${PROJECT_NAME})

install(PROGRAMS scripts/test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
