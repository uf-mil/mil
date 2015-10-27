cmake_minimum_required(VERSION 2.8.3)
set(PROJECT_NAME sub8_trajectory_generator)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
project(${PROJECT_NAME})

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sub8_msgs cmake_modules)
find_package(OMPL REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)


set(catkin_LIBRARIES
	${catkin_LIBRARIES}
)
catkin_package(
   	CATKIN_DEPENDS roscpp sub8_msgs
   	DEPENDS system_lib 
)

include_directories(
  	include/${PROJECT_NAME}
 	${catkin_INCLUDE_DIRS}
  	${OMPL_INCLUDE_DIRS}
  	${Boost_INCLUDE_DIRS}
  	${Eigen_INCLUDE_DIRS}
)


add_executable(
	trajectory_generator 
	src/trajectory_generator.cc 
	src/sub8_state_space.cc
	src/space_information_generator.cc
	src/sub8_state_validity_checker.cc
	src/tgen_manager.cc
	src/tgen_thruster_info.cc
	src/sub_dynamics.cc
)

target_link_libraries(trajectory_generator
	${catkin_LIBRARIES}
	${OMPL_LIBRARIES}
	${Eigen_LIBRARIES}
)

catkin_add_gtest(${PROJECT_NAME}_unit_tests 
	test/sub8_state_space_test.cc
	test/sub8_state_validity_checker_test.cc
	test/sub_dynamics_test.cc
	test/space_information_test.cc
	test/tgen_manager_test.cc
	test/test_driver.cc
	src/sub8_state_space.cc
	src/space_information_generator.cc
	src/sub8_state_validity_checker.cc
	src/tgen_manager.cc
	src/tgen_thruster_info.cc
	src/sub_dynamics.cc
)

target_link_libraries(
	${PROJECT_NAME}_unit_tests
	${catkin_LIBRARIES}
	${OMPL_LIBRARIES}
	${Eigen_LIBRARIES}
)
