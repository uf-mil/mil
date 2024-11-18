cmake_minimum_required(VERSION 3.0.2)
project(electrical_protocol)
find_package(catkin REQUIRED COMPONENTS
  rospy
  serial
  axros
)

catkin_python_setup()
catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # add_rostest(test/adder_device_sub9.test)
  # add_rostest(test/echo_device_sub9.test)
  # add_rostest(test/basic.test)
endif()
