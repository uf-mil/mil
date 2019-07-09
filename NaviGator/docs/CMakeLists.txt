cmake_minimum_required(VERSION 2.8.3)
project(navigator_documentation)
find_package(catkin REQUIRED)
catkin_package()

# Function to generate an svg image from
function(dot_to_svg input target_name)
  get_filename_component(_INPUT_BASE ${input} NAME_WE)
  set(_INPUT ${CMAKE_CURRENT_SOURCE_DIR}/${input})
  set(_OUTPUT ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/${_INPUT_BASE}.svg)
  add_custom_command(OUTPUT ${_OUTPUT}
    COMMAND dot -Tsvg ${_INPUT} > ${_OUTPUT}
    DEPENDS ${_INPUT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generating svg from graphviz file ${_INPUT}"
  )
  add_custom_target(${PROJECT_NAME}_${target_name} DEPENDS ${_OUTPUT})
endfunction(dot_to_svg)

# Generate SVG for the high level graph
dot_to_svg(high_level_architecture.dot high_level_architecture)
