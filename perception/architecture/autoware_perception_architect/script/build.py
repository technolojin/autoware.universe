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


import os
from pathlib import Path
import sys

import yaml

debug_mode = True


list_of_element_types = ["module", "pipeline", "parameter_set"]

list_module = []
list_pipeline = []
list_parameter_set = []

list_output = []
list_input = []


class ModuleConfig:
    def __init__(self, name, launch, if_input, if_output, parameter, configuration, process):
        self.name = name
        self.launch = launch
        self.if_input = if_input
        self.if_output = if_output
        self.parameter = parameter
        self.configuration = configuration
        self.process = process


def load_module(module_dir):
    # parse the architecture yaml configuration
    with open(module_dir, "r") as stream:
        try:
            module_yaml = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return
    if "name" not in module_yaml:
        print(f"Field 'name' is required in module configuration.")
        return
    module_name = module_yaml.get("name")
    # check the name is [name].module
    if module_name.split(".")[1] != "module":
        raise ValueError(f"Invalid module name: '{module_name}'. Please check file {module_dir}")

    required_field = [
        "name",
        "launch",
        "input",
        "output",
        "parameter",
        "configuration",
        "process",
    ]
    for field in required_field:
        if field not in module_yaml:
            raise ValueError(
                f"Field '{field}' is required in module configuration. Please check file {module_dir}"
            )
    try:
        module_config = ModuleConfig(
            name=module_yaml.get("name"),
            launch=module_yaml.get("launch"),
            if_input=module_yaml.get("input", []),
            if_output=module_yaml.get("output", []),
            parameter=module_yaml.get("parameter", []),
            configuration=module_yaml.get("configuration", []),
            process=module_yaml.get("process", []),
        )
        list_module.append(module_config)
    except yaml.YAMLError as exc:
        print(exc)
    if debug_mode:
        print(module_config.__dict__)
    return


def load_pipeline(pipeline_dir):
    # parse the pipeline yaml configuration
    with open(pipeline_dir, "r") as stream:
        try:
            pipeline_yaml = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return

    if debug_mode:
        print(pipeline_yaml)

    return


def load_parameter_set(parameter_set_dir):
    # parse the parameter set yaml configuration
    with open(parameter_set_dir, "r") as stream:
        try:
            parameter_set_yaml = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return
    if debug_mode:
        print(parameter_set_yaml)

    return


# load architecture files
def load_architecture_element(element_yaml_dir):

    # the type can be distinguished by the file name
    # consist of: name.[type].yaml
    file_name = Path(element_yaml_dir).stem
    element_type = file_name.split(".")[1]
    if debug_mode:
        print(f"Architecture element type: {element_type}")
    if element_type not in list_of_element_types:
        if debug_mode:
            print(f"Invalid architecture element type: {element_type}")
        return

    # parse the architecture yaml configuration
    # possible elements: module, pipeline, parameter set
    # the type can be distinguished by the key
    if element_type == "module":
        load_module(element_yaml_dir)
    elif element_type == "pipeline":
        load_pipeline(element_yaml_dir)
    elif element_type == "parameter_set":
        load_parameter_set(element_yaml_dir)
    else:
        if debug_mode:
            print(f"Invalid architecture element type: {element_type}")
    return


# load deployment file
def load_deployment(deployment_yaml):
    # parse the deployment yaml configuration
    # the deployment yaml configuration should contain the following

    if debug_mode:
        print(deployment_yaml)

    return


# search and connect the connections between the modules


def build(deployment_file, architecture_yaml_list_file):
    print("autoware architect: Building deployment...")

    # deployment_file: a yaml file that contains the deployment configuration
    # architecture_yaml_list: a list of yaml file directories that contain the architecture configuration

    # parse the architecture yaml configuration list
    # the list is a text file that contains directories of the yaml files
    with open(architecture_yaml_list_file, "r") as file:
        architecture_yaml_list = file.read().splitlines()

    # iterate through the architecture yaml configuration list
    for architecture_yaml_dir in architecture_yaml_list:
        # check if the file exists
        if not os.path.exists(architecture_yaml_dir):
            if debug_mode:
                print(f"File {architecture_yaml_dir} does not exist.")
            # if the file does not exist, remove the file from the list
            architecture_yaml_list.remove(architecture_yaml_dir)
            continue

        # load the architecture elements
        load_architecture_element(architecture_yaml_dir)

    # parse the deployment file
    with open(deployment_file, "r") as stream:
        try:
            deployment = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return
    # load the deployment
    load_deployment(deployment)

    # search and connect the connections between the modules


if __name__ == "__main__":
    build(sys.argv[1], sys.argv[2])
