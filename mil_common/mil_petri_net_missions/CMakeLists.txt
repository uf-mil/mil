cmake_minimum_required(VERSION 3.0.2)
project(mil_petri_net_missions)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy)

find_package(Boost REQUIRED COMPONENTS thread system)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mil_petri_net_missions
  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/petri_net.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${Boost_LIBREARIES}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test test/petri_net_basic.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()


if(CATKIN_ENABLE_TESTING)
  foreach(T
      test/petri_net_basic.test)
     add_rostest(${T})
  endforeach()
endif()

