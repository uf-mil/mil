cmake_minimum_required(VERSION 3.0.2)
project(sub_actuator_board)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  mil_usb_to_can
)
catkin_python_setup()
add_service_files(
  FILES
  SetValve.srv
  GetValve.srv
)
add_rostest(test/simulated_board.test)
generate_messages()
catkin_package(CATKIN_DEPENDS message_runtime)
