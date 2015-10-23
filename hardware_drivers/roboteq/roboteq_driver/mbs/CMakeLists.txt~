
set(${PROJECT_NAME}_SCRIPT_HEX_CPP ${CMAKE_CURRENT_BINARY_DIR}/genc_script_hex.cpp)
add_custom_command(OUTPUT ${${PROJECT_NAME}_SCRIPT_HEX_CPP}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  DEPENDS genc script.mbs script.hex
  COMMAND ./genc script_lines script.hex > ${${PROJECT_NAME}_SCRIPT_HEX_CPP}
)
add_library(${PROJECT_NAME}_script STATIC ${${PROJECT_NAME}_SCRIPT_HEX_CPP})
