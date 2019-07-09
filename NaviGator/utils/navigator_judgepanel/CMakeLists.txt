cmake_minimum_required(VERSION 2.8.3)
project(navigator_judgespanel)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs  
  navigator_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES qt
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

find_package (Qt5Widgets)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set (JudgesPanelLib_src ${PROJECT_SOURCE_DIR}/src/UI.cpp)
set (JudgesPanelLib_hdr ${PROJECT_SOURCE_DIR}/src/UI.h)
set (JudgesPanelLib_ui  ${PROJECT_SOURCE_DIR}/src/UI.ui)
set (JudgesPanelBin_src ${PROJECT_SOURCE_DIR}/src/main.cpp)

qt5_wrap_cpp(JudgesPanelLib_hdr_moc ${JudgesPanelLib_hdr})
qt5_wrap_ui (JudgesPanelLib_ui_moc  ${JudgesPanelLib_ui})

include_directories (${PROJECT_SOURCE_DIR})
include_directories (${PROJECT_BINARY_DIR})

add_library (JudgesPanelLib SHARED 
    ${JudgesPanelLib_src}
    ${JudgesPanelLib_hdr_moc}
    ${JudgesPanelLib_ui_moc}
)
target_link_libraries (JudgesPanelLib Qt5::Widgets)

add_executable(judges_panel ${JudgesPanelBin_src})
target_link_libraries (judges_panel JudgesPanelLib ${catkin_LIBRARIES})
add_dependencies(JudgesPanelLib ${catkin_EXPORTED_TARGETS})
