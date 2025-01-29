cmake_minimum_required(VERSION 3.14)
project(autoware_perception_architect)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

ament_package(
  CONFIG_EXTRAS "autoware_architect-extras.cmake"
)

# Copy the script to the build directory, all of them
# also template files
file(GLOB_RECURSE SCRIPT_FILES ${CMAKE_SOURCE_DIR}/script/*.py)
file(GLOB_RECURSE TEMPLATE_FILES ${CMAKE_SOURCE_DIR}/script/*.jinja2)

foreach(SCRIPT_FILE ${SCRIPT_FILES})
  get_filename_component(SCRIPT_FILE_NAME ${SCRIPT_FILE} NAME)
  configure_file(${SCRIPT_FILE} ${CMAKE_BINARY_DIR}/script/${SCRIPT_FILE_NAME} COPYONLY)
endforeach()
foreach(TEMPLATE_FILE ${TEMPLATE_FILES})
  get_filename_component(TEMPLATE_FILE_NAME ${TEMPLATE_FILE} NAME)
  configure_file(${TEMPLATE_FILE} ${CMAKE_BINARY_DIR}/script/${TEMPLATE_FILE_NAME} COPYONLY)
endforeach()

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
