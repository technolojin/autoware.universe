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
import sys
from typing import List

import yaml

debug_mode = True

list_of_element_types = ["module", "pipeline", "parameter_set"]


def load_config_yaml(config_yaml_dir):
    with open(config_yaml_dir, "r") as stream:
        try:
            config_yaml = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return
    return config_yaml


def element_name_decode(element_name) -> (str, str):
    # example of full_name: "ObjectDetector.module"
    if "." not in element_name:
        raise ValueError(f"Invalid element name: '{element_name}'")

    splitted = element_name.split(".")
    if len(splitted) < 2:
        raise ValueError(f"Invalid element name: '{element_name}'")

    element_name = splitted[0]  # example: "ObjectDetector"
    element_type = splitted[1]  # example: "module"

    # Check the element type
    if element_type not in list_of_element_types:
        raise ValueError(f"Invalid element type: '{element_type}'")

    return element_name, element_type


# classes for architecture configuration


class Element:
    def __init__(self, config_yaml_dir):
        self.config_yaml_dir = config_yaml_dir
        self.config_yaml = load_config_yaml(config_yaml_dir)

        # Check the name field
        if "name" not in self.config_yaml:
            print(f"Field 'name' is required in element configuration. File {self.config_yaml_dir}")
            return False

        self.full_name = self.config_yaml.get("name")
        self.name, self.type = element_name_decode(self.full_name)

        # Check the element
        if self.type not in list_of_element_types:
            raise ValueError(f"Invalid element type: '{self.type}'. File {self.config_yaml_dir}")
        self.check_config()

    def check_config(self) -> bool:
        return True


class ModuleElement(Element):
    def check_config(self) -> bool:
        # Check the name is [name].module
        if self.type != "module":
            raise ValueError(
                f"Invalid element name: '{self.full_name}'. File {self.config_yaml_dir}"
            )
            return False

        # Check the launch field
        required_field = [
            "name",
            "launch",
            "inputs",
            "outputs",
            "parameters",
            "configurations",
            "processes",
        ]
        for field in required_field:
            if field not in self.config_yaml:
                raise ValueError(
                    f"Field '{field}' is required in module configuration. Please check file {self.config_yaml_dir}"
                )
                return False

        # All checks passed
        return True


class PipelineElement(Element):
    def check_config(self) -> bool:
        # Check the name is [name].pipeline
        if self.type != "pipeline":
            raise ValueError(
                f"Invalid element name: '{self.full_name}'. File {self.config_yaml_dir}"
            )
            return False

        # Check the launch field
        required_field = [
            "name",
            "depends",
            "nodes",
            "external_interfaces",
            "connections",
            "parameters",
            "configurations",
        ]

        for field in required_field:
            if field not in self.config_yaml:
                raise ValueError(
                    f"Field '{field}' is required in pipeline configuration. Please check file {self.config_yaml_dir}"
                )
                return False

        # All checks passed
        return True


class ParameterSetElement(Element):
    def check_config(self) -> bool:
        # Check the name is [name].parameter_set
        if self.type != "parameter_set":
            raise ValueError(
                f"Invalid element name: '{self.full_name}'. File {self.config_yaml_dir}"
            )
            return False

        # Check the launch field
        required_field = [
            "name",
            "parameters",
        ]

        for field in required_field:
            if field not in self.config_yaml:
                raise ValueError(
                    f"Field '{field}' is required in parameter_set configuration. Please check file {self.config_yaml_dir}"
                )
                return False

        # All checks passed
        return True


# classes for deployment
class InPort:
    def __init__(self, name, msg_type, namespace: List[str] = []):
        self.name = name
        self.msg_type = msg_type
        self.namespace: List[str] = namespace
        self.topic = None
        # to enable/disable connection checker
        self.is_required = True

    def set_topic(self, topic_name, namespace):
        # input topic is given
        self.topic = "/".join(namespace) + "/" + topic_name


class OutPort:
    def __init__(self, name, msg_type, namespace: List[str] = []):
        self.name = name
        self.msg_type = msg_type
        self.namespace: List[str] = namespace
        self.topic = None

        # for topic monitor
        self.period = 0.0
        self.is_monitored = False

    def set_topic(self):
        self.topic = "/".join(self.namespace) + "/" + self.name
        if debug_mode:
            print(f"  output port: {self.topic}")


class Link:
    def __init__(self, msg_type):
        self.msg_type = msg_type
        # from-port and to-port connection
        self.from_port: [InPort, OutPort] = None
        #   from-port type is InPort: from an pipeline external input interface
        #   from-port type is OutPort: from an module output
        self.to_port: [OutPort, InPort] = None
        #   to-port type is OutPort: to an pipeline external output interface
        #   to-port type is InPort: to an module input

    def set_from_port(self, from_port: OutPort):
        if from_port.msg_type != self.msg_type:
            raise ValueError("From port message type does not match")
        self.from_port = from_port

    def set_to_port(self, to_port: InPort):
        if to_port.msg_type != self.msg_type:
            raise ValueError("To port message type does not match")
        self.to_port = to_port


class ModuleList:
    def __init__(self, module_list: List[ModuleElement]):
        self.list: List[ModuleElement] = []
        for module in module_list:
            self.list.append(module)

    def get(self, module_name):
        for module in self.list:
            if module.name == module_name:
                return module
        raise ValueError(f"ModuleList: Module not found: {module_name}")


class PipelineList:
    def __init__(self, pipeline_list: List[PipelineElement]):
        self.list: List[PipelineElement] = []
        for pipeline in pipeline_list:
            self.list.append(pipeline)

    def get(self, pipeline_name):
        for pipeline in self.list:
            if pipeline.name == pipeline_name:
                return pipeline
        raise ValueError(f"PipelineList: Pipeline not found: {pipeline_name}")


class Instance:
    # Common attributes for node hierarch instance
    def __init__(self, name: str, compute_unit: str, namespace: list[str], layer: int = 0):
        self.name: str = name
        self.namespace: List[str] = namespace.copy()
        # add the instance name to the namespace
        self.namespace.append(name)

        self.compute_unit: str = compute_unit
        self.layer: int = layer
        LAYER_LIMIT = 50
        if self.layer > LAYER_LIMIT:
            raise ValueError("Instance layer is too deep")

        # element
        self.element: [ModuleElement, PipelineElement] = None
        self.element_type: str = None
        self.parent: Instance = None
        self.children: List[Instance] = []
        self.parent_pipeline_list: List[str] = []

        # interface
        self.in_ports: List[InPort] = []
        self.out_ports: List[OutPort] = []
        self.links: List[Link] = []

        # status
        self.is_initialized = False

    def set_element(self, element_id, module_list, pipeline_list):
        element_name, element_type = element_name_decode(element_id)

        if element_type == "pipeline":
            if debug_mode:
                namespace_str = "/" + "/".join(self.namespace)
                print(f"Instance set_element: Setting {element_id} instance {namespace_str}")
            self.element = pipeline_list.get(element_name)
            self.element_type = element_type

            # check if the pipeline is already set
            if element_id in self.parent_pipeline_list:
                raise ValueError(f"Element is already set: {element_id}, avoid circular reference")
            self.parent_pipeline_list.append(element_id)

            # set children
            node_list = self.element.config_yaml.get("nodes")
            for node in node_list:
                instance = Instance(
                    node.get("node"), self.compute_unit, self.namespace, self.layer + 1
                )
                instance.parent = self
                instance.parent_pipeline_list = self.parent_pipeline_list.copy()
                # recursive call of set_element
                instance.set_element(node.get("element"), module_list, pipeline_list)
                self.children.append(instance)

            # run the pipeline configuration
            self.run_pipeline_configuration()

            # recursive call is finished
            self.is_initialized = True

        elif element_type == "module":
            if debug_mode:
                namespace_str = "/" + "/".join(self.namespace)
                print(f"Instance set_element: Setting {element_id} instance {namespace_str}")
            if self.layer == 0:
                raise ValueError(
                    "Module is not supported in the top level of the deployment, {element_id}"
                )
            self.element = module_list.get(element_name)
            self.element_type = element_type

            # run the module configuration
            self.run_module_configuration()

            # recursive call is finished
            self.is_initialized = True

        else:
            raise ValueError(f"Invalid element type: {element_type}")

    def run_module_configuration(self):
        if self.element_type != "module":
            raise ValueError("run_module_configuration is only supported for module")

        # set in_ports
        for in_port in self.element.config_yaml.get("inputs"):
            in_port_name = in_port.get("name")
            in_port_msg_type = in_port.get("message_type")
            in_port_instance = InPort(in_port_name, in_port_msg_type, self.namespace)
            self.in_ports.append(in_port_instance)

        # set out_ports
        for out_port in self.element.config_yaml.get("outputs"):
            out_port_name = out_port.get("name")
            out_port_msg_type = out_port.get("message_type")
            out_port_instance = OutPort(out_port_name, out_port_msg_type, self.namespace)
            self.out_ports.append(out_port_instance)

    def run_pipeline_configuration(self):
        if self.element_type != "pipeline":
            raise ValueError("run_pipeline_configuration is only supported for pipeline")

        # collect internal in/out ports from children
        internal_in_ports: List[InPort] = []
        internal_out_ports: List[OutPort] = []
        for child in self.children:
            if not child.is_initialized:
                raise ValueError("Child instance is not initialized")
            internal_in_ports += child.in_ports
            internal_out_ports += child.out_ports

        # set external interfaces
        external_input = self.element.config_yaml.get("external_interfaces").get("input")

        # set links


class Deployment:
    def __init__(
        self,
        config_yaml_dir: str,
        module_list: List[ModuleElement],
        pipeline_list: List[PipelineElement],
        parameter_set_list: List[ParameterSetElement],
    ):
        # load yaml file
        self.config_yaml_dir = config_yaml_dir
        self.config_yaml = load_config_yaml(config_yaml_dir)

        self.module_list: ModuleList = ModuleList(module_list)
        self.pipeline_list: PipelineList = PipelineList(pipeline_list)
        self.parameter_set_list: List[ParameterSetElement] = parameter_set_list

        # Check the configuration
        self.check_config()

        # member variables
        self.instances: List[Instance] = []
        # self.connections: List[Link] = []

        # build the deployment
        self.build()

    def check_config(self) -> bool:
        return True

    def find_parameter_set(self, parameter_set_name):
        parameter_set = [
            parameter_set
            for parameter_set in self.parameter_set_list
            if parameter_set.name == parameter_set_name
        ]
        if len(parameter_set) == 0:
            raise ValueError(f"Parameter set not found: {parameter_set_name}")
        return parameter_set[0]  # if multiple parameter sets have the same name, pick the first one

    def build(self):
        # 1. build the node tree, recursive algorithm
        for units in self.config_yaml.get("compute_units"):
            compute_unit_name = units.get("unit")
            for components in units.get("components"):
                # instance
                instance_name = components.get("component")
                element_id = components.get("element")
                namespace = components.get("namespace")

                instance = Instance(instance_name, compute_unit_name, [namespace])
                instance.set_element(element_id, self.module_list, self.pipeline_list)

                self.instances.append(instance)

        # 2. build the connection tree

        # 3. set message topics

        # 4. build the parameter connections

        # 5. generate system monitor configuration

        # 6. build the launcher

        # 7. visualize the deployment diagram via plantuml


class ListElement:
    def __init__(self, config_yaml_file_dirs: List[str]):
        self.elements: List[Element] = []
        self.fill_list(config_yaml_file_dirs)

    def fill_list(self, config_yaml_file_dirs: List[str]):
        for config_yaml_file_dir in config_yaml_file_dirs:
            if debug_mode:
                print(f"ListElement fill_list: Loading {config_yaml_file_dir}")
            element = Element(config_yaml_file_dir)
            self.elements.append(element)

    def get_module_list(self):
        return [element.config_yaml_dir for element in self.elements if element.type == "module"]

    def get_pipeline_list(self):
        return [element.config_yaml_dir for element in self.elements if element.type == "pipeline"]

    def get_parameter_set_list(self):
        return [
            element.config_yaml_dir for element in self.elements if element.type == "parameter_set"
        ]
