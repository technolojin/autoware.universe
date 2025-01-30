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
from typing import List

import classes as awa_cls
from classes import element_name_decode
from classes import load_config_yaml
import jinja2

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
        # create namespace string, FOR ERROR MESSAGE ONLY
        self.namespace_str: str = "/" + "/".join(self.namespace)
        self.id = "__".join(self.namespace) + "__" + name

        self.compute_unit: str = compute_unit
        self.layer: int = layer
        LAYER_LIMIT = 50
        if self.layer > LAYER_LIMIT:
            raise ValueError("Instance layer is too deep")

        # element
        self.element: [
            awa_cls.ModuleElement,
            awa_cls.PipelineElement,
            awa_cls.ArchitectureElement,
        ] = None
        self.element_type: str = None
        self.parent: Instance = None
        self.children: List[Instance] = []
        self.parent_pipeline_list: List[str] = []

        # interface
        self.in_ports: List[awa_cls.InPort] = []
        self.out_ports: List[awa_cls.OutPort] = []
        self.links: List[awa_cls.Link] = []

        # processes
        self.processes: List[awa_cls.Process] = []

        # parameters
        self.parameters: awa_cls.ParameterList = awa_cls.ParameterList()

        # status
        self.is_initialized = False

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
                try:
                    instance.set_element(node.get("element"), module_list, pipeline_list)
                except Exception as e:
                    # add the instance to the children list for debugging
                    self.children.append(instance)
                    raise ValueError(f"Error in setting child instance {instance.name} : {e}")
                self.children.append(instance)

            # run the pipeline configuration
            self.run_pipeline_configuration()

            # recursive call is finished
            self.is_initialized = True

        elif element_type == "module":
            if debug_mode:
                print(f"Instance set_element: Setting {element_id} instance {self.namespace_str}")
            self.element = module_list.get(element_name)
            self.element_type = element_type

            # run the module configuration
            self.run_module_configuration()

            # recursive call is finished
            self.is_initialized = True

        else:
            raise ValueError(f"Invalid element type: {element_type}")

    def run_pipeline_configuration(self):
        if self.element_type != "pipeline":
            raise ValueError("run_pipeline_configuration is only supported for pipeline")

        # set connections
        connection_list_yaml = self.element.config_yaml.get("connections")
        if len(connection_list_yaml) == 0:
            raise ValueError("No connections found in the pipeline configuration")

        connection_list: List[awa_cls.Connection] = []
        for connection in connection_list_yaml:
            connection_instance = awa_cls.Connection(connection)
            connection_list.append(connection_instance)

        # establish links
        for connection in connection_list:
            # case 1. from external input to internal input
            if connection.type == 1:
                link_list: List[awa_cls.Link] = []
                # find the to_instance from children
                to_instance = self.get_child(connection.to_instance)
                port_list = list(to_instance.in_ports)
                if len(port_list) == 0:
                    raise ValueError(f"No available port found in {to_instance.name}")
                # if the port name is wildcard, find available port from the to_instance
                if connection.to_port_name == "*":
                    for port in port_list:
                        from_port = awa_cls.InPort(port.name, port.msg_type, self.namespace)
                        link = awa_cls.Link(port.msg_type, from_port, port, self.namespace)
                        link_list.append(link)
                else:
                    # match the port name
                    to_port = to_instance.get_in_port(connection.to_port_name)
                    # create a link
                    from_port = awa_cls.InPort(
                        connection.from_port_name, to_port.msg_type, self.namespace
                    )
                    link = awa_cls.Link(to_port.msg_type, from_port, to_port, self.namespace)
                    link_list.append(link)

                for link in link_list:
                    self.links.append(link)
                    if debug_mode:
                        print(f"Link: {link.from_port.full_name}-> {link.to_port.full_name}")

            # case 2. from internal output to internal input
            if connection.type == 2:
                link_list: List[awa_cls.Link] = []
                # find the from_instance and to_instance from children
                from_instance = self.get_child(connection.from_instance)
                to_instance = self.get_child(connection.to_instance)
                # find the from_port and to_port
                from_port = from_instance.get_out_port(connection.from_port_name)
                to_port = to_instance.get_in_port(connection.to_port_name)
                # create link
                link = awa_cls.Link(from_port.msg_type, from_port, to_port, self.namespace)
                link_list.append(link)

                for link in link_list:
                    self.links.append(link)
                    if debug_mode:
                        print(f"Link: {link.from_port.full_name}-> {link.to_port.full_name}")

            # case 3. from internal output to external output
            if connection.type == 3:
                link_list: List[awa_cls.Link] = []

                # find the from_instance from children
                from_instance = self.get_child(connection.from_instance)
                port_list = list(from_instance.out_ports)
                if len(port_list) == 0:
                    raise ValueError(f"No available port found in {from_instance.name}")

                # if the port name is wildcard, find available port from the from_instance
                if connection.from_port_name == "*":
                    for port in port_list:
                        to_port = awa_cls.OutPort(port.name, port.msg_type, self.namespace)
                        link = awa_cls.Link(port.msg_type, port, to_port, self.namespace)
                        link_list.append(link)
                else:
                    # match the port name
                    from_port = from_instance.get_out_port(connection.from_port_name)
                    # create link
                    to_port = awa_cls.OutPort(
                        connection.to_port_name, from_port.msg_type, self.namespace
                    )
                    link = awa_cls.Link(from_port.msg_type, from_port, to_port, self.namespace)
                    link_list.append(link)

                for link in link_list:
                    self.links.append(link)
                    if debug_mode:
                        print(f"Connection: {link.from_port.full_name}-> {link.to_port.full_name}")

        # create external ports
        self.create_external_ports(self.links)

        if debug_mode:
            print(
                f"Instance {self.name} run_pipeline_configuration: {len(self.links)} links are established"
            )
            for link in self.links:
                print(f"  Link: {link.from_port.full_name} -> {link.to_port.full_name}")
            # new ports
            for in_port in self.in_ports:
                print(f"  New in port: {in_port.full_name}")
            for out_port in self.out_ports:
                print(f"  New out port: {out_port.full_name}")

    def run_module_configuration(self):
        if self.element_type != "module":
            raise ValueError("run_module_configuration is only supported for module")

        # set in_ports
        for in_port in self.element.config_yaml.get("inputs"):
            in_port_name = in_port.get("name")
            in_port_msg_type = in_port.get("message_type")
            in_port_instance = awa_cls.InPort(in_port_name, in_port_msg_type, self.namespace)
            self.in_ports.append(in_port_instance)

        # set out_ports
        for out_port in self.element.config_yaml.get("outputs"):
            out_port_name = out_port.get("name")
            out_port_msg_type = out_port.get("message_type")
            out_port_instance = awa_cls.OutPort(out_port_name, out_port_msg_type, self.namespace)
            self.out_ports.append(out_port_instance)

        # set parameters
        for param in self.element.config_yaml.get("parameters"):
            param_name = param.get("name")
            param_value = param.get("default")
            self.parameters.set_parameter(param_name, param_value)

        # parse processes and get trigger conditions and output conditions
        # self.processes = awa_cls.ProcessList(self.element.config_yaml.get("processes"))
        for process_config in self.element.config_yaml.get("processes"):
            name = process_config.get("name")
            self.processes.append(awa_cls.Process(name, process_config))

        # connect port events and the process events

    def get_child(self, name: str):
        for child in self.children:
            if child.name == name:
                return child
        raise ValueError(f"Child not found: child name {name}, instance of {self.name}")

    def get_in_port(self, name: str):
        for in_port in self.in_ports:
            if in_port.name == name:
                return in_port
        raise ValueError(f"In port not found: in-port name {name}, instance of {self.name}")

    def get_out_port(self, name: str):
        for out_port in self.out_ports:
            if out_port.name == name:
                return out_port
        raise ValueError(f"Out port not found: out-port name {name}, instance of {self.name}")

    def set_in_port(self, in_port: awa_cls.InPort):
        # check the external input is defined
        external_input_list = self.element.config_yaml.get("external_interfaces").get("input")
        external_input_list = [ext_input.get("name") for ext_input in external_input_list]
        if in_port.name not in external_input_list:
            raise ValueError(f"External input not found: {in_port.name} in {external_input_list}")

        # check if there is a port with the same name
        for port in self.in_ports:
            if port.name == in_port.name:
                # check if the message type is the same
                if port.msg_type != in_port.msg_type:
                    raise ValueError(
                        f"Message type mismatch: {port.full_name} {port.msg_type} != {in_port.msg_type}"
                    )
                # same port name is found, update reference
                port.set_references(in_port.reference)
                return
        # same port name is not found, add the port
        self.in_ports.append(in_port)

    def set_out_port(self, out_port: awa_cls.OutPort):
        # check the external output is defined
        external_output_list = self.element.config_yaml.get("external_interfaces").get("output")
        external_output_list = [ext_output.get("name") for ext_output in external_output_list]
        if out_port.name not in external_output_list:
            raise ValueError(
                f"External output not found: {out_port.name} in {external_output_list}"
            )

        # check if there is a port with the same name
        for port in self.out_ports:
            if port.name == out_port.name:
                # check if the message type is the same
                if port.msg_type != out_port.msg_type:
                    raise ValueError(
                        f"Message type mismatch: {port.full_name} {port.msg_type} != {out_port.msg_type}"
                    )
                # same port name is found, update reference
                port.set_references(out_port.reference)
                return
        # same port name is not found, add the port
        self.out_ports.append(out_port)

    def create_external_ports(self, link_list):
        # create in ports based on the link_list
        for link in link_list:
            # create port only if the namespace is the same as the instance
            if link.from_port.namespace == self.namespace:
                # set the in_port
                self.set_in_port(link.from_port)
            if link.to_port.namespace == self.namespace:
                # set the out_port
                self.set_out_port(link.to_port)

    def check_ports(self):
        # recursive call for children
        for child in self.children:
            child.check_ports()

        # check ports only for module. others are only for connection
        if self.element_type != "module":
            return

        # check ports
        for in_port in self.in_ports:
            print(f"  In port: {in_port.full_name}")
            print(f"    Subscribing topic: {in_port.topic}")
            server_port_list = in_port.servers
            if server_port_list == []:
                print("    Server port not found")
                continue
            if debug_mode:
                for server_port in server_port_list:
                    print(f"    server: {server_port.full_name}, topic: {server_port.topic}")

        for out_port in self.out_ports:
            print(f"  Out port: {out_port.full_name}")
            user_port_list = out_port.users
            if user_port_list == []:
                print("    User port not found")
                continue
            if debug_mode:
                for user_port in user_port_list:
                    print(f"    user: {user_port.full_name}")

    def set_parameter(self, param):
        # in case of pipeline, search parameter connection and call set_parameter for children
        if self.element_type == "pipeline":
            self.set_pipeline_parameter(param)
        # in case of module, set the parameter
        elif self.element_type == "module":
            self.set_module_parameter(param)
        else:
            raise ValueError(f"Invalid element type: {self.element_type}")

    def set_pipeline_parameter(self, param):
        if self.element_type != "pipeline":
            raise ValueError("set_pipeline_parameter is only supported for pipeline")
        param_name = param.get("name")

        # check external_interfaces/parameter
        pipeline_parameter_list = self.element.config_yaml.get("external_interfaces").get(
            "parameter"
        )
        # check if the param_list_yaml is superset of pipeline_parameter_list
        pipeline_parameter_list = [param.get("name") for param in pipeline_parameter_list]
        if param_name not in pipeline_parameter_list:
            raise ValueError(f"Parameter not found: {param_name} in {pipeline_parameter_list}")

        # check parameters to connect parameters to the children
        param_connection_list = self.element.config_yaml.get("parameters")
        for connection in param_connection_list:
            param_from = connection.get("from")
            param_from_name = param_from.split(".")[1]
            if param_from_name != param_name:
                continue

            param_to = connection.get("to")
            param_to_inst_name = param_to.split(".")[0]
            child_instance = self.get_child(param_to_inst_name)

            # set the parameter to the child instance
            if child_instance.element_type == "pipeline":
                param["name"] = param_to.split(".")[2]

            child_instance.set_parameter(param)

    def set_module_parameter(self, param):
        if self.element_type != "module":
            raise ValueError("set_module_parameter is only supported for module")
        param_path_list = param.get("parameter_paths")
        # get list of parameter paths, which comes in dictionary format
        for param_path in param_path_list:
            param_keys = param_path.keys()
            for param_key in param_keys:
                param_value = param_path.get(param_key)
                self.parameters.set_parameter(param_key, param_value)
        if debug_mode:
            for param in self.parameters.list:
                print(f"  Parameter: {param.name} = {param.value}")

    def collect_instance_data(self):
        data = {
            "name": self.name,
            "id": self.id,
            "element_type": self.element_type,
            "namespace": self.namespace,
            "in_ports": [{"name": port.name, "id": port.id} for port in self.in_ports],
            "out_ports": [
                {"name": port.name, "id": port.id, "topic": port.topic, "msg_type": port.msg_type}
                for port in self.out_ports
            ],
            "children": (
                [child.collect_instance_data() for child in self.children]
                if hasattr(self, "children")
                else []
            ),
            "links": (
                [
                    {
                        "from_port": link.from_port,
                        "to_port": link.to_port,
                        "msg_type": link.msg_type,
                    }
                    for link in self.links
                ]
                if hasattr(self, "links")
                else []
            ),
        }
        return data


class ArchitectureInstance(Instance):
    def __init__(self, name: str):
        super().__init__(name)

    def set_component_instances(self, module_list, pipeline_list, parameter_set_list):
        # set pipeline and module instances as 'components'
        for component in self.element.config_yaml.get("components"):
            compute_unit_name = component.get("compute_unit")

            instance_name = component.get("component")
            element_id = component.get("element")
            namespace = component.get("namespace")

            # parameter set
            parameter_set = component.get("parameter_set")
            param_list_yaml = None
            if parameter_set is not None:
                param_set_name, element_type = element_name_decode(parameter_set)
                if element_type != "parameter_set":
                    raise ValueError(f"Invalid parameter set type: {element_type}")
                param_set = parameter_set_list.get(param_set_name)
                param_list_yaml = param_set.config_yaml.get("parameters")

            # create instance
            instance = Instance(instance_name, compute_unit_name, [namespace])
            instance.parent = self
            try:
                instance.set_element(element_id, module_list, pipeline_list)
            except Exception as e:
                # add the instance to the children list for debugging
                self.children.append(instance)
                raise ValueError(f"Error in setting component instance {instance_name} : {e}")

            if param_list_yaml is not None:
                for param in param_list_yaml:
                    instance.set_parameter(param)

            self.children.append(instance)
        # all children are initialized
        self.is_initialized = True

    def set_connections(self):
        # 2. connect instances
        # set connections
        connection_list_yaml = self.element.config_yaml.get("connections")
        if len(connection_list_yaml) == 0:
            raise ValueError("No connections found in the pipeline configuration")

        connection_list: List[awa_cls.Connection] = []
        for connection in connection_list_yaml:
            connection_instance = awa_cls.Connection(connection)
            connection_list.append(connection_instance)

        # establish links. topics will be defined in this step
        link_list: List[awa_cls.Link] = []
        for connection in connection_list:
            # find the from_instance and to_instance from children
            from_instance = self.get_child(connection.from_instance)
            to_instance = self.get_child(connection.to_instance)
            # find the from_port and to_port
            from_port = from_instance.get_out_port(connection.from_port_name)
            to_port = to_instance.get_in_port(connection.to_port_name)
            # check if the port type
            if not isinstance(from_port, awa_cls.OutPort):
                raise ValueError(f"Invalid port type: {from_port.full_name}")
            if not isinstance(to_port, awa_cls.InPort):
                raise ValueError(f"Invalid port type: {to_port.full_name}")
            # create link
            link = awa_cls.Link(from_port.msg_type, from_port, to_port, self.namespace)
            link_list.append(link)

        for link in link_list:
            self.links.append(link)
            if debug_mode:
                print(f"Connection: {link.from_port.full_name}-> {link.to_port.full_name}")
        if debug_mode:
            print(f"Instance {self.name} set_architecture: {len(self.links)} links are established")
            for link in self.links:
                print(f"  Link: {link.from_port.full_name} -> {link.to_port.full_name}")

        # check ports
        if debug_mode:
            print(f"Instance {self.name}: checking ports")
        self.check_ports()

    def set_architecture(
        self,
        architecture: awa_cls.ArchitectureElement,
        module_list,
        pipeline_list,
        parameter_set_list,
    ):
        if debug_mode:
            print(
                f"Instance set_architecture: Setting {architecture.full_name} instance {self.name}"
            )
        self.element = architecture
        self.element_type = "architecture"

        # set component instances
        self.set_component_instances(module_list, pipeline_list, parameter_set_list)


class Deployment:
    def __init__(
        self, config_yaml_dir: str, element_list: awa_cls.ElementList, output_root_dir: str
    ):
        # load yaml file
        self.config_yaml_dir = config_yaml_dir
        self.config_yaml = load_config_yaml(config_yaml_dir)
        self.name = self.config_yaml.get("name")

        self.module_list: awa_cls.ModuleList = awa_cls.ModuleList(element_list.get_module_list())
        self.pipeline_list: awa_cls.PipelineList = awa_cls.PipelineList(
            element_list.get_pipeline_list()
        )
        self.parameter_set_list: awa_cls.ParameterSetList = awa_cls.ParameterSetList(
            element_list.get_parameter_set_list()
        )
        self.architecture_list: awa_cls.ArchitectureList = awa_cls.ArchitectureList(
            element_list.get_architecture_list()
        )

        # Check the configuration
        self.check_config()

        # member variables
        self.architecture_instance: Instance = None
        self.individual_parameters_yaml = None

        # output paths
        self.output_root_dir = output_root_dir
        self.launcher_dir = os.path.join(self.output_root_dir, "launcher/")
        self.system_monitor_dir = os.path.join(self.output_root_dir, "system_monitor/")
        self.visualization_dir = os.path.join(self.output_root_dir, "visualization/")

        # build the deployment
        self.build()

        # set the vehicle individual parameters
        #   sensor calibration, vehicle parameters, map, etc.

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

    def build(self):
        # 1. set architecture instance
        architecture = self.architecture_list.get(self.config_yaml.get("architecture"))

        if not architecture:
            raise ValueError(f"Architecture not found: {self.config_yaml.get('architecture')}")

        try:
            self.architecture_instance = ArchitectureInstance(self.name)
            # 1. set architecture instance
            self.architecture_instance.set_architecture(
                architecture, self.module_list, self.pipeline_list, self.parameter_set_list
            )
            # 2. set connections
            self.architecture_instance.set_connections()
        except Exception as e:
            # try to visualize the architecture to show error status
            self.visualize()
            raise ValueError(f"Error in setting architecture: {e}")

    def visualize(self):
        # 4. visualize the deployment diagram via plantuml
        # define the traverse_instance function

        # load the template file
        script_dir = os.path.dirname(__file__)
        template_path = os.path.join(script_dir, "node-diagram.puml.jinja2")
        with open(template_path, "r") as f:
            plantuml_template = f.read()

        # Collect data from the architecture instance
        # data = collect_instance_data(self.architecture_instance)
        data = self.architecture_instance.collect_instance_data()

        # Render the Jinja2 template with the collected data
        template = jinja2.Template(plantuml_template)
        plantuml_output = template.render(data)

        # write the plantuml file
        plantuml_file = os.path.join(self.visualization_dir, "architecture.puml")
        if os.path.exists(plantuml_file):
            os.remove(plantuml_file)
        if not os.path.exists(self.visualization_dir):
            os.makedirs(self.visualization_dir, exist_ok=True)
        with open(plantuml_file, "w") as f:
            # f.write("\n".join(lines))
            f.write(plantuml_output)

    def generate_system_monitor(self):
        # 2. generate system monitor configuration
        pass

    def generate_launcher(self):
        # 3. build the launcher
        pass
