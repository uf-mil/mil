cmake_minimum_required(VERSION 2.8.3)
project(mil_usb_to_can)
find_package(catkin REQUIRED COMPONENTS
  rospy
  serial
)
catkin_python_setup()
catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/adder_device.test)
endif()
