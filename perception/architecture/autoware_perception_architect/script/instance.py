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


import re
from typing import List

from classes import ArchitectureElement
from classes import ArchitectureList
from classes import Connection
from classes import ElementList
from classes import InPort
from classes import Link
from classes import ModuleElement
from classes import ModuleList
from classes import OutPort
from classes import ParameterSetList
from classes import PipelineElement
from classes import PipelineList
from classes import element_name_decode
from classes import load_config_yaml

debug_mode = True


class Instance:
    # Common attributes for node hierarch instance
    def __init__(
        self, name: str, compute_unit: str = "", namespace: list[str] = [], layer: int = 0
    ):
        self.name: str = name
        self.namespace: List[str] = namespace.copy()
        # add the instance name to the namespace
        self.namespace.append(name)
        self.namespace_str: str = "/" + "/".join(self.namespace)

        self.compute_unit: str = compute_unit
        self.layer: int = layer
        LAYER_LIMIT = 50
        if self.layer > LAYER_LIMIT:
            raise ValueError("Instance layer is too deep")

        # element
        self.element: [ModuleElement, PipelineElement, ArchitectureElement] = None
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

    def set_architecture(self, architecture: ArchitectureElement, module_list, pipeline_list):
        if debug_mode:
            print(
                f"Instance set_architecture: Setting {architecture.full_name} instance {self.name}"
            )
        # set component instances
        for component in architecture.config_yaml.get("components"):
            compute_unit_name = component.get("unit")

            instance_name = component.get("component")
            element_id = component.get("element")
            namespace = component.get("namespace")

            instance = Instance(instance_name, compute_unit_name, [namespace])
            instance.parent = self
            instance.set_element(element_id, module_list, pipeline_list)

            self.children.append(instance)
        # all children are initialized
        self.is_initialized = True

    def set_element(self, element_id, module_list, pipeline_list):
        element_name, element_type = element_name_decode(element_id)

        if element_type == "pipeline":
            if debug_mode:
                print(f"Instance set_element: Setting {element_id} instance {self.namespace_str}")
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
                print(f"Instance set_element: Setting {element_id} instance {self.namespace_str}")
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

        # # collect internal in/out ports from children
        # internal_in_ports: List[InPort] = []
        # internal_out_ports: List[OutPort] = []
        # for child in self.children:
        #     if not child.is_initialized:
        #         raise ValueError("Child instance is not initialized")
        #     internal_in_ports += child.in_ports
        #     internal_out_ports += child.out_ports

        # set external interfaces
        external_input_list = self.element.config_yaml.get("external_interfaces").get("input")
        external_output_list = self.element.config_yaml.get("external_interfaces").get("output")

        # set connections
        connection_list_yaml = self.element.config_yaml.get("connections")
        if len(connection_list_yaml) == 0:
            raise ValueError("No connections found in the pipeline configuration")

        connection_list: List[Connection] = []
        for connection in connection_list_yaml:
            connection_instance = Connection(connection)
            connection_list.append(connection_instance)

        # establish links
        link_list: List[Link] = []

        for connection in connection_list:
            # case 1. from external input to internal input
            if connection.type == 1:
                print(
                    f"Connection: external/{connection.from_port_name}-> {connection.to_instance}/{connection.to_port_name}"
                )
                # find the to_instance from children
                for child in self.children:
                    if child.name == connection.to_instance:
                        to_instance = child
                        break
                # match the port name
                for in_port in to_instance.in_ports:
                    if in_port.name == connection.to_port_name:
                        to_port = in_port
                        break
                # create link
                from_port = InPort(connection.from_port_name, to_port.msg_type, self.namespace)
                link_list.append(Link(to_port.msg_type, from_port, to_port))

            # case 2. from internal output to internal input
            if connection.type == 2:
                print(
                    f"Connection: {connection.from_instance}/{connection.from_port_name}-> {connection.to_instance}/{connection.to_port_name}"
                )
                # find the from_instance and to_instance from children
                from_instance = None
                to_instance = None
                for child in self.children:
                    if child.name == connection.from_instance:
                        from_instance = child
                    if child.name == connection.to_instance:
                        to_instance = child
                    if from_instance and to_instance:
                        break
                # find the from_port and to_port
                from_port = None
                to_port = None
                for out_port in from_instance.out_ports:
                    if out_port.name == connection.from_port_name:
                        from_port = out_port
                        break
                for in_port in to_instance.in_ports:
                    if in_port.name == connection.to_port_name:
                        to_port = in_port
                        break
                # create link
                link_list.append(Link(from_port.msg_type, from_port, to_port))

            # case 3. from internal output to external output
            if connection.type == 3:
                print(
                    f"Connection: {connection.from_instance}/{connection.from_port_name}-> external/{connection.to_port_name}"
                )
                # find the from_instance from children
                for child in self.children:
                    if child.name == connection.from_instance:
                        from_instance = child
                        break
                # match the port name
                for out_port in from_instance.out_ports:
                    if out_port.name == connection.from_port_name:
                        from_port = out_port
                        break
                # create link
                to_port = OutPort(connection.to_port_name, from_port.msg_type, self.namespace)
                link_list.append(Link(from_port.msg_type, from_port, to_port))

        # create in ports based on the link_list
        for link in link_list:
            # create port only if the namespace is the same as the instance
            if link.from_port.namespace == self.namespace:
                # check if the out_port is already in the list
                if link.from_port not in self.in_ports:
                    self.in_ports.append(link.from_port)
            if link.to_port.namespace == self.namespace:
                # check if the in_port is already in the list
                if link.to_port not in self.out_ports:
                    self.out_ports.append(link.to_port)
        if debug_mode:
            print(f"Instance run_pipeline_configuration: {len(link_list)} links are established")
            for link in link_list:
                print(
                    f"Link: {link.from_port.namespace}/{link.from_port.name} -> {link.to_port.namespace}/{link.to_port.name}"
                )
            # new ports
            for in_port in self.in_ports:
                print(f"New in port: {in_port.namespace}/{in_port.name}")
            for out_port in self.out_ports:
                print(f"New out port: {out_port.namespace}/{out_port.name}")


class Deployment:
    def __init__(self, config_yaml_dir: str, element_list: ElementList):
        # load yaml file
        self.config_yaml_dir = config_yaml_dir
        self.config_yaml = load_config_yaml(config_yaml_dir)
        self.name = self.config_yaml.get("name")

        self.module_list: ModuleList = ModuleList(element_list.get_module_list())
        self.pipeline_list: PipelineList = PipelineList(element_list.get_pipeline_list())
        self.parameter_set_list: ParameterSetList = ParameterSetList(
            element_list.get_parameter_set_list()
        )
        self.architecture_list: ArchitectureList = ArchitectureList(
            element_list.get_architecture_list()
        )

        # Check the configuration
        self.check_config()

        # member variables
        self.architecture_instance: Instance = None
        # self.connections: List[Link] = []

        # build the deployment
        self.build()

    def check_config(self) -> bool:
        # Check the name field
        deployment_config_fields = [
            "name",
            "architecture",
            "additional_components",
            "individual_parameters",
        ]
        for field in deployment_config_fields:
            if field not in self.config_yaml:
                raise ValueError(
                    f"Field '{field}' is required in deployment configuration file {self.config_yaml_dir}"
                )
                return False
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
        # 0. find the architecture
        architecture = self.architecture_list.get(self.config_yaml.get("architecture"))

        if not architecture:
            raise ValueError(f"Architecture not found: {self.config_yaml.get('architecture')}")

        # 1. build the node tree, recursive algorithm
        architecture_instance = Instance(self.name)
        architecture_instance.set_architecture(architecture, self.module_list, self.pipeline_list)

        # 2. build the connection tree

        # 3. set message topics

        # 4. build the parameter connections

        # 5. generate system monitor configuration

        # 6. build the launcher

        # 7. visualize the deployment diagram via plantuml
