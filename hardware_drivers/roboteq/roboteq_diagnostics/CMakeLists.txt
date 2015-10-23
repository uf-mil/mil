cmake_minimum_required(VERSION 2.8.3)
project(roboteq_diagnostics)

find_package(catkin REQUIRED)

# catkin_python_setup()

catkin_package()

install(PROGRAMS
  nodes/diagnostic_publisher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
