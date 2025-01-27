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

import sys

from classes import ElementList
from instance import Deployment

# global variables
debug_mode = True


# build the deployment
# search and connect the connections between the modules
def build(deployment_file: str, architecture_yaml_list_file: str, output_root_dir: str):
    print("autoware architect: Building deployment...")

    # Inputs:
    #   deployment_file: a yaml file that contains the deployment configuration
    #   architecture_yaml_list: a list of yaml file directories that contain the architecture configuration

    # parse the architecture yaml configuration list
    # the list is a text file that contains directories of the yaml files
    with open(architecture_yaml_list_file, "r") as file:
        architecture_yaml_list = file.read().splitlines()

    # load the architecture yaml files
    element_list = ElementList(architecture_yaml_list)

    # load the deployment yaml file
    deployment = Deployment(deployment_file, element_list, output_root_dir)


if __name__ == "__main__":
    build(sys.argv[1], sys.argv[2], sys.argv[3])
