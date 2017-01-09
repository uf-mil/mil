cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")
project(sonar)

find_package(bvtsdkpkg)
find_package(catkin
			REQUIRED COMPONENTS
			roslib
			image_transport
			cv_bridge)
find_package(OpenCV REQUIRED)
if(bvtsdkpkg_FOUND)
	catkin_python_setup()
	catkin_package(
		INCLUDE_DIRS
		include 
		LIBRARIES
		sonar_tools
		CATKIN_DEPENDS
		bvtsdkpkg
		roslib
		image_transport
		cv_bridge
		DEPENDS 
		OpenCV
		)
	include_directories(include ${bvtsdkpkg_INCLUDE_DIRS} ${roslib_INCLUDE_DIRS})

	add_library(sonar_tools
				src/BVWrapper.cpp)

	target_link_libraries(sonar_tools
						${catkin_LIBRARIES}
						${OpenCV_LIBRARIES}
						${roslib_LIBRARIES}
						${bvtsdkpkg_LIBRARIES}
						)

	add_executable(test_sdk test/test_sdk.cpp)

	target_link_libraries(test_sdk
						${catkin_LIBRARIES}
						${OpenCV_LIBRARIES}
						${bvtsdkpkg_LIBRARIES}
						sonar_tools
						)


	add_executable(image_pub nodes/image_pub.cpp)

	target_link_libraries(image_pub
						sonar_tools
						)
else()
	message("Blue View SDK not found, not building sonar package")
endif()



