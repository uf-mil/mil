cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)
# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)
message( ${ROS_LIB_DIR} )
#print_board_list()
#print_programmer_list()

include_directories(${Boost_INCLUDE_DIRS})
generate_arduino_firmware(arduino
  SRCS shooter.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD atmega328
  PORT /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A603UWY5-if00-port0
)
