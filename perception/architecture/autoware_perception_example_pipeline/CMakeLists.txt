cmake_minimum_required(VERSION 3.14)
project(autoware_perception_example_pipeline)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
)

autoware_architect_configure()

autoware_architect_generate_launcher(
  module/DetectorA.module
  detector_a_node
)

autoware_architect_generate_launcher(
  module/FilterA.module
  filter_a_node
)

autoware_architect_generate_launcher(
  module/Tracker.module
  tracker_node
)
