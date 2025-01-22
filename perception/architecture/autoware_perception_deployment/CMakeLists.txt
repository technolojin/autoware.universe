cmake_minimum_required(VERSION 3.14)
project(autoware_perception_deployment)

find_package(autoware_cmake REQUIRED)
find_package(autoware_perception_architect REQUIRED)

autoware_package()

ament_auto_package(
  INSTALL_TO_SHARE
    architecture
    deployment
)

autoware_architect_configure()

# Call the function to get the final list of YAML files
# autoware_architect_get_module_list()

# Add a dummy library to make a target
add_library(${PROJECT_NAME} SHARED src/empty.cpp)

# Call the function to build and deploy the modules
autoware_architect_build_deploy(${PROJECT_NAME} vehicle_x.deployment)
