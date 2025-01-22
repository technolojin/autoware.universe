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

from typing import List

import yaml

debug_mode = True

list_of_element_types = ["module", "pipeline", "parameter_set", "architecture"]


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


class ElementList:
    def __init__(self, config_yaml_file_dirs: List[str]):
        self.elements: List[Element] = []
        self.fill_list(config_yaml_file_dirs)

    def fill_list(self, config_yaml_file_dirs: List[str]):
        for config_yaml_file_dir in config_yaml_file_dirs:
            if debug_mode:
                print(f"ElementList fill_list: Loading {config_yaml_file_dir}")
            element = Element(config_yaml_file_dir)
            # check if the element is already in the list
            for e in self.elements:
                if e.full_name == element.full_name:
                    raise ValueError(
                        f"Element is already in the list: \n to be added {element.config_yaml_dir}\n exist {e.config_yaml_dir}"
                    )
            # add the element to the list
            self.elements.append(element)

    def get_module_list(self):
        return [element.config_yaml_dir for element in self.elements if element.type == "module"]

    def get_pipeline_list(self):
        return [element.config_yaml_dir for element in self.elements if element.type == "pipeline"]

    def get_parameter_set_list(self):
        return [
            element.config_yaml_dir for element in self.elements if element.type == "parameter_set"
        ]

    def get_architecture_list(self):
        return [
            element.config_yaml_dir for element in self.elements if element.type == "architecture"
        ]


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


class ModuleList:
    def __init__(self, module_list: List[Element]):
        self.list: List[ModuleElement] = []
        for module in module_list:
            self.list.append(ModuleElement(module))

    def get(self, module_name):
        for module in self.list:
            if module.name == module_name:
                return module
        raise ValueError(f"ModuleList: Module not found: {module_name}")


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


class PipelineList:
    def __init__(self, pipeline_list: List[Element]):
        self.list: List[PipelineElement] = []
        for pipeline in pipeline_list:
            self.list.append(PipelineElement(pipeline))

    def get(self, pipeline_name):
        for pipeline in self.list:
            if pipeline.name == pipeline_name:
                return pipeline
        # if not found, print the list of pipelines
        list_text = ""
        for pipeline in self.list:
            list_text += f"{pipeline.name}\n"
        raise ValueError(
            f"PipelineList: Pipeline not found: {pipeline_name}, available pipelines are:\n{list_text}"
        )


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


class ParameterSetList:
    def __init__(self, parameter_set_list: List[Element]):
        self.list: List[ParameterSetElement] = []
        for parameter_set in parameter_set_list:
            self.list.append(ParameterSetElement(parameter_set))

    def get(self, parameter_set_name):
        for parameter_set in self.list:
            if parameter_set.name == parameter_set_name:
                return parameter_set
        # if not found, print the list of parameter sets
        list_text = ""
        for parameter_set in self.list:
            list_text += f"{parameter_set.name}\n"
        raise ValueError(
            f"ParameterSetList: Parameter set not found: {parameter_set_name}, available parameter sets are:\n{list_text}"
        )


class ArchitectureElement(Element):
    def check_config(self) -> bool:
        if self.type != "architecture":
            raise ValueError(
                f"Invalid element name: '{self.full_name}'. File {self.config_yaml_dir}"
            )
            return False

        # Check the launch field
        required_field = [
            "name",
            "components",
            "connections",
            "parameter_sets",
            "parameter_connections",
        ]

        for field in required_field:
            if field not in self.config_yaml:
                raise ValueError(
                    f"Field '{field}' is required in architecture configuration. Please check file {self.config_yaml_dir}"
                )
                return False

        # All checks passed
        return True


class ArchitectureList:
    def __init__(self, architecture_list: List[Element]):
        self.list: List[ArchitectureElement] = []
        for architecture in architecture_list:
            self.list.append(ArchitectureElement(architecture))

    def get(self, architecture_name):
        # if the name is full name, decode it
        if "." in architecture_name:
            architecture_name, _ = element_name_decode(architecture_name)

        for architecture in self.list:
            if architecture.name == architecture_name:
                return architecture
        # if not found, print the list of architectures
        list_text = ""
        for architecture in self.list:
            list_text += f"{architecture.name}\n"
        raise ValueError(
            f"ArchitectureList: Architecture not found: {architecture_name}, available architectures are:\n{list_text}"
        )


# classes for deployment
class InPort:
    def __init__(self, name, msg_type, namespace: List[str] = []):
        self.name = name
        self.msg_type = msg_type
        self.namespace: List[str] = namespace
        self.topic = None
        # to enable/disable connection checker
        self.is_required = True


class OutPort:
    def __init__(self, name, msg_type, namespace: List[str] = []):
        self.name = name
        self.msg_type = msg_type
        self.namespace: List[str] = namespace
        self.topic = None

        # for topic monitor
        self.period = 0.0
        self.is_monitored = False


class Link:
    def __init__(self, msg_type, from_port, to_port):
        self.msg_type: str = msg_type
        # from-port and to-port connection
        self.from_port: [InPort, OutPort] = from_port
        #   from-port type is InPort: from an pipeline external input interface
        #   from-port type is OutPort: from an module output
        self.to_port: [InPort, OutPort] = to_port
        #   to-port type is OutPort: to an pipeline external output interface
        #   to-port type is InPort: to an module input


class Connection:
    def __init__(self, connection_dict: dict):
        
        # connection type       
        #   0: undefined
        #   1: external input to internal input
        #   2: internal output to external output
        #   3: internal output to internal input
        self.type = 0
    
        from_instance, from_port_name = self.parse_port_name(connection_dict.get("from"))
        to_instance, to_port_name = self.parse_port_name(connection_dict.get("to"))

        if from_instance == "" and to_instance == "":
            raise ValueError(f"Invalid connection: {connection_dict}")
        elif from_instance == "" and to_instance != "":
            self.type = 1
        elif from_instance != "" and to_instance == "":
            self.type = 2
        elif from_instance != "" and to_instance != "":
            self.type = 3

        self.from_instance: str = from_instance
        self.from_port_name: str = from_port_name
        self.to_instance: str = to_instance
        self.to_port_name: str = to_port_name

    def parse_port_name(self, port_name:str)-> (str, str): # (instance_name, port_name)
        name_splited = port_name.split(".")
        if len(name_splited) == 2:
            if name_splited[0] == "input":
                return "", name_splited[1] # external input
            if name_splited[0] == "output":
                return "", name_splited[1] # external output
            raise ValueError(f"Invalid port name: {port_name}")
        elif len(name_splited) == 3:
            if name_splited[1] == "input":
                return name_splited[0], name_splited[2] # internal input
            if name_splited[1] == "output":
                return name_splited[0], name_splited[2] # internal output
            raise ValueError(f"Invalid port name: {port_name}")
        else:
            raise ValueError(f"Invalid port name: {port_name}")
    

class ConnectionGraph:
    def __init__(self, connection_list: List[dict]):
        self.connection_list: List[Connection] = []
        self.fill_list(connection_list)

    def fill_list(self, connection_list: List[dict]):
        for connection in connection_list:
            self.connection_list.append(Connection(connection))
    

            
            
