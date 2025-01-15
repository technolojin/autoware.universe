cmake_minimum_required(VERSION 3.14)
project(autoware_perception_dummy_modules_b)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_package(
  INSTALL_TO_SHARE
  architecture
)

autoware_architect_configure()
