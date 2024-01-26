cmake_minimum_required(VERSION 3.8)
project(vision_roi_object_detector)

# find dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

# Targets
ament_auto_add_library(vision_roi_object_detector_node_component SHARED
  src/vision_roi_object_detector/node.cpp
)

rclcpp_components_register_node(vision_roi_object_detector_node_component
  PLUGIN "vision_roi_object_detector::RoiObjectDetectorNode"
  EXECUTABLE vision_roi_object_detector_node)


if(BUILD_TESTING)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
