# Declare a cpp executable
add_executable(${PROJECT_NAME}_node driver controller channel)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_script roboteq_msgs_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_script ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME driver_node PREFIX "")

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
