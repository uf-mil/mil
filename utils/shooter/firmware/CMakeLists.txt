cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

#print_board_list()
#print_programmer_list()

generate_arduino_firmware(arduino
  SRCS shooter.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD atmega328
  PORT /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A603UWY5-if00-port0
)
