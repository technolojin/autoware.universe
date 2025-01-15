# Copyright 2025 TIER IV, inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# list of yaml files collected over multiple packages
set(autoware_architect_module_yaml_files "")


# load architecture configuration yaml file for module
# and store the list of the yaml files collected over multiple packages

macro(autoware_architect_add_module module_name)
  
  # create a list of yaml files
  set(autoware_architect_module_yaml_files "")
  set(autoware_architect_module_yaml_files ${autoware_architect_module_yaml_files} PARENT_SCOPE)

  # get the path to the module configuration yaml file
  get_filename_component(autoware_architect_module_yaml_file "${CMAKE_CURRENT_SOURCE_DIR}/../architecture/${module_name}.yaml" ABSOLUTE)

  # add the path to the list of yaml files
  set(autoware_architect_module_yaml_files ${autoware_architect_module_yaml_files} ${autoware_architect_module_yaml_file} PARENT_SCOPE)

  # print the current list of yaml files
  message(STATUS "autoware_architect_module_yaml_files: ${autoware_architect_module_yaml_files}")

endmacro()