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


# collect all yaml files in the architecture directory
# store the list in a shared file
macro(autoware_architect_configure)
  file(GLOB_RECURSE YAML_FILES "${CMAKE_CURRENT_SOURCE_DIR}/architecture/*.yaml")
  # define the path to the shared file
  set(shared_file "${CMAKE_BINARY_DIR}/../autoware_perception_architect/autoware_architect_yaml_filelist.txt")

  foreach(YAML_FILE ${YAML_FILES})
    # read the current content of the shared file
    if(EXISTS ${shared_file})
      file(READ ${shared_file} yaml_files_content)
      string(REPLACE "\n" ";" yaml_files_list ${yaml_files_content})
    else()
      set(yaml_files_list "")
    endif()

    # check if the yaml file is already in the list
    list(FIND yaml_files_list ${YAML_FILE} index)
    if(index EQUAL -1)
      # append the new yaml file to the shared file
      file(APPEND ${shared_file} "${YAML_FILE}\n")
      message(STATUS "Added to autoware_architect_yaml_file: ${YAML_FILE}")
    else()
      message(STATUS "Already registered: ${YAML_FILE}")
    endif()

  endforeach()
endmacro()
