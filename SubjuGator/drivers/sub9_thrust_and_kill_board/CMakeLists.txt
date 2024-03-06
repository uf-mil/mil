cmake_minimum_required(VERSION 2.8.3)
project(sub9_thrust_and_kill_board)
find_package(catkin REQUIRED COMPONENTS
  mil_usb_to_can
)
add_rostest(test/simulated_board.test)
catkin_python_setup()
catkin_package()
