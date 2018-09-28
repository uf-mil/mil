cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_object_detection_and_recognition)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2
  mil_msgs
  interactive_markers
  eigen_conversions
  pcl_ros
  cv_bridge
  dynamic_reconfigure
  mil_bounds
)

###################################################
# 				     BASE CODE 					  #
###################################################

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/PCODAR.cfg
)

# C++ Flags
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")

# PCL stuff
find_package(PCL 1.2 REQUIRED)

# OPENCV stuff
find_package(OpenCV 3.0 REQUIRED)

# Include directories
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# Get all relevant files
file(GLOB SOURCES src/*.cpp)

# Create Library
add_library(pcodar ${SOURCES})

# Link all the relevant libs
target_link_libraries(pcodar ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Weird catkin stuff, We will never know what it does.
add_dependencies(pcodar ${catkin_EXPORTED_TARGETS})


catkin_package(INCLUDE_DIRS include
               LIBRARIES pcodar
               CATKIN_DEPENDS roscpp sensor_msgs std_msgs mil_msgs tf tf2 eigen_conversions pcl_ros)

# executable that actually runs everything
add_executable(pcodar_node nodes/main.cpp)
target_link_libraries(pcodar_node pcodar)


###################################################
# 				  LAUNCH CODE 					  #
###################################################

roslaunch_add_file_check(launch/pcodar.launch)

###################################################
# 						TOOLS 					  #
###################################################

#find_package(Qt5Widgets)
#set(CMAKE_AUTOMOC ON)
#set(CMAKE_INCLUDE_CURRENT_DIR ON)

#QT5_WRAP_CPP(tst_hdr_moc tools/labeling_tool/main_window.hpp)
#QT5_WRAP_UI(tst_form_hdr tools/labeling_tool/main_window.ui)

#add_library(main_window ${tst_hdr_moc} ${tst_form_hdr})
#qt5_use_modules(main_window Widgets)

#add_library(label_model tools/labeling_tool/label_model.hpp tools/labeling_tool/label_model.cpp)
#target_link_libraries(label_model pcodar main_window ${catkin_LIBRARIES})

#include_directories(${catkin_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS})

#add_executable(pcodar_labeling_tool tools/labeling_tool/main.cpp)
#qt5_use_modules(pcodar_labeling_tool Core Gui Widgets)

#target_link_libraries(pcodar_labeling_tool pcodar main_window label_model ${catkin_LIBRARIES} ${Qt5Widgets_LIBRARIES})
