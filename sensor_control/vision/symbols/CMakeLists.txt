cmake_minimum_required(VERSION 2.8.3)
project(navigator_shoot_vision)
  
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  message_generation 
  sensor_msgs
  cv_bridge
  image_transport
  geometry_msgs
  navigator_msgs
)
find_package( OpenCV REQUIRED )

catkin_package(
  CATKIN_DEPENDS 
	std_msgs 
    std_srvs
    message_runtime
)


include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(shape_filter src/smart.cpp)
target_link_libraries(shape_filter ${catkin_LIBRARIES})
add_dependencies(shape_filter navigator_msgs_generate_messages)

set(GRAYSCALE_CONTOUR_FILES src/GrayscaleContour/GrayscaleContour.cpp)
add_executable(shape_finder src/DebugWindow.cpp src/DockShapeVision.cpp src/main.cpp  ${GRAYSCALE_CONTOUR_FILES} src/ContourMethod/ContourMethod.cpp src/ContourMethod/FrameProc.cpp src/ContourMethod/ContourMethod.cpp src/ContourMethod/ShapeFind.cpp  src/ContourMethod/ShapeDetector.cpp )
add_dependencies(shape_finder navigator_msgs_generate_messages)
target_link_libraries(shape_finder ${catkin_LIBRARIES})

target_link_libraries(shape_finder ${OpenCV_LIBS} )

#add_executable(shore_debug src/ShoreDebug.cpp src/PoseEstimator.cpp)
add_executable(shore_debug src/ShoreDebug.cpp)
add_dependencies(shore_debug navigator_msgs_generate_messages)
target_link_libraries(shore_debug ${catkin_LIBRARIES})
target_link_libraries(shore_debug ${OpenCV_LIBS} )
