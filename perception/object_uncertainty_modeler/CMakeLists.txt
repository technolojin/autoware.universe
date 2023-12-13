cmake_minimum_required(VERSION 3.5)
project(object_uncertainty_modeler)

# Dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

# Targets
ament_auto_add_library(object_uncertainty_modeler_radar_node_component SHARED
  src/object_uncertainty_modeler_node/object_uncertainty_modeler_radar_node.cpp
)

rclcpp_components_register_node(object_uncertainty_modeler_radar_node_component
  PLUGIN "object_uncertainty_modeler::ObjectUncertaintyModelerRadarNode"
  EXECUTABLE object_uncertainty_modeler_radar_node
)

# Tests
if(BUILD_TESTING)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package
ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
