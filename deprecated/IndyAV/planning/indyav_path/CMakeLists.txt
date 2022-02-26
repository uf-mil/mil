# cmake_minimum_required(VERSION 2.8.3)
# project(indyav_path)
# 
# ## Find catkin macros and libraries
# ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# ## is used, also find other catkin packages
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   std_msgs
#   nav_msgs
#   mil_tools
# )
# 
# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")
# 
# # System dependencies are found with CMake's conventions
# #find_package(REQUIRED COMPONENTS system)
# 
# ## Generate messages in the 'msg' folder
# # add_message_files(
# #   FILES
# #   Message1.msg
# #   Message2.msg
# # )
# 
# ## Generate services in the 'srv' folder
# # add_service_files(
# #   FILES
# #   Service1.srv
# #   Service2.srv
# # )
# 
# ## Generate actions in the 'action' folder
# # add_action_files(
# #   FILES
# #   Action1.action
# #   Action2.action
# # )
# 
# ## Generate added messages and services with any dependencies listed here
# # generate_messages(
# #   DEPENDENCIES
# #   std_msgs
# # )
# 
# ################################################
# ## Declare ROS dynamic reconfigure parameters ##
# ################################################
# 
# ## To declare and build dynamic reconfigure parameters within this
# ## package, follow these steps:
# ## * In the file package.xml:
# ##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
# ## * In this file (CMakeLists.txt):
# ##   * add "dynamic_reconfigure" to
# ##     find_package(catkin REQUIRED COMPONENTS ...)
# ##   * uncomment the "generate_dynamic_reconfigure_options" section below
# ##     and list every .cfg file to be processed
# 
# ## Generate dynamic reconfigure parameters in the 'cfg' folder
# # generate_dynamic_reconfigure_options(
# #   cfg/DynReconf1.cfg
# #   cfg/DynReconf2.cfg
# # )
# 
# ###################################
# ## catkin specific configuration ##
# ###################################
# ## The catkin_package macro generates cmake config files for your package
# ## Declare things to be passed to dependent projects
# ## INCLUDE_DIRS: uncomment this if your package contains header files
# ## LIBRARIES: libraries you create in this project that dependent projects also need
# ## CATKIN_DEPENDS: catkin_packages dependent projects also need
# ## DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
#   INCLUDE_DIRS include
#   CATKIN_DEPENDS roscpp std_msgs nav_msgs mil_tools
# #  LIBRARIES
# )
# 
# ###########
# ## Build ##
# ###########
# 
# ## Specify additional locations of header files
# ## Your package locations should be listed before other locations
# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
# )
# 
# ## Declare a C++ library
# # add_library(${PROJECT_NAME}
# #   src/${PROJECT_NAME}/planning.cpp
# # )
# 
# ## Add cmake target dependencies of the library
# ## as an example, code may need to be generated before libraries
# ## either from message generation or dynamic reconfigure
# # add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# 
# # Path Player
# add_executable(path_player src/path_player.cpp)
# target_link_libraries(path_player ${catkin_LIBRARIES})
# add_dependencies (path_player ${catkin_EXPORTED_TARGETS})
# 
# # Path Recorder
# add_executable(path_recorder src/path_recorder.cpp)
# target_link_libraries(path_recorder ${catkin_LIBRARIES})
# add_dependencies (path_recorder ${catkin_EXPORTED_TARGETS})
