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

macro(autoware_architect_generate_launcher module_yaml_file_name executable_name)
  set(GENERATE_LAUNCHER_PY_SCRIPT "${CMAKE_BINARY_DIR}/../autoware_perception_architect/script/generate_launcher.py")
  set(MODULE_YAML_FILE "${CMAKE_SOURCE_DIR}/architecture/${module_yaml_file_name}.yaml")
  set(LAUNCHER_FILE_DIR "${CMAKE_INSTALL_PREFIX}/share/${CMAKE_PROJECT_NAME}/launcher/")

  # run build.py script
  add_custom_target(${executable_name}_generate_launcher ALL
    COMMAND ${CMAKE_COMMAND} -E env python3 ${GENERATE_LAUNCHER_PY_SCRIPT} ${MODULE_YAML_FILE} ${executable_name} ${LAUNCHER_FILE_DIR}
    COMMENT "Running generate_launcher.py script to generate ${module_yaml_file_name}"
  )

  # # add dependencies to trigger the build.py script when building the project
  # add_dependencies(${project_name} run_build_py)
endmacro()
