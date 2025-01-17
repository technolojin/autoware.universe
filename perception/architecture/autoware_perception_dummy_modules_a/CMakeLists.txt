cmake_minimum_required(VERSION 3.14)
project(autoware_perception_dummy_modules_a)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
)

autoware_architect_configure()

autoware_architect_generate_launcher(
  DetectorA.module
  detector_a_node
)

autoware_architect_generate_launcher(
  FilterA.module
  filter_a_node
)

autoware_architect_generate_launcher(
  SensorA.module
  sensor_a_node
)
