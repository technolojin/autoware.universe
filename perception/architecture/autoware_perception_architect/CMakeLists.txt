cmake_minimum_required(VERSION 3.14)
project(autoware_perception_architect)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

ament_package(
  CONFIG_EXTRAS "autoware_architect-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)