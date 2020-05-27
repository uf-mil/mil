cmake_minimum_required(VERSION 2.8.3)
project(indyav_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin
    REQUIRED COMPONENTS
    roscpp
    indyav_control
    gazebo_ros
    xacro
    )

## System dependencies are found with CMake's conventions
find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  include
)



## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES backwheel_plugin
  CATKIN_DEPENDS indyav_control_generate_message gazebo_ros roscpp
)

###########
## Build ##
###########


add_library(
  backwheel_plugin
  src/backwheel_plugin.cpp
)
target_link_libraries(
  backwheel_plugin
  ${catkin_LIBRARIES}
)
add_dependencies(backwheel_plugin
    include/wheel_plugin.hpp
    indyav_control_generate_message
    ${catkin_EXPORTED_TARGETS}
)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/indyav_gazebo_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )


# Add new_smyrna speedway generation
add_custom_target(new_smyrna
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_racetrack.py ${CMAKE_CURRENT_SOURCE_DIR}/worlds/new_smyrna.yaml ${CMAKE_CURRENT_SOURCE_DIR}/worlds/racetracks/new_smyrna_speedway.xacro
  BYPRODUCTS
    ${CMAKE_CURRENT_SOURCE_DIR}/worlds/racetracks/new_smyrna_speedway.xacro
  DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_racetrack.py
    ${CMAKE_CURRENT_SOURCE_DIR}/worlds/new_smyrna.yaml
  USES_TERMINAL
)
xacro_add_files(
  worlds/new_smyrna.world.xacro
  INORDER INSTALL DESTINATION ${CMAKE_CURRENT_DEVEL_DIR}/worlds
  DEPENDS new_smyrna
)


# add car urdf
xacro_add_files(
  urdf/indyav_car.urdf.xacro
  INORDER INSTALL DESTINATION urdf
  TARGET xacro_urdf
)

