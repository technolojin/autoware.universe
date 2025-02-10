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
        try:
            self._fill_list(config_yaml_file_dirs)
        except ValueError as e:
            print(e, config_yaml_file_dirs)
            raise

    def _fill_list(self, config_yaml_file_dirs: List[str]):
        for config_yaml_file_dir in config_yaml_file_dirs:
            if debug_mode:
                print(f"ElementList fill_list: Loading {config_yaml_file_dir}")
            element = Element(config_yaml_file_dir)
            # check if the element is already in the list
            for e in self.elements:
                if e.full_name == element.full_name:
                    raise ValueError(
                        f"Element {e.full_name} is already in the list: \n to be added {element.config_yaml_dir}\n exist {e.config_yaml_dir}"
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

        # Check the required field
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

        # Check the required field
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

        # Check the required fields
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

        # Check the required field
        required_field = [
            "name",
            "components",
            "connections",
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
class Event:
    def __init__(self, name: str, namespace: List[str], is_process_event=False):
        self.name = name
        self.namespace = namespace
        self.id = ("__".join(namespace) + "__" + name).replace("/", "__")
        self.type_list = [
            "on_input",
            "on_trigger",
            "once",
            "periodic",
            "to_trigger",
            "to_output",
        ]
        # on_input: activate the event when the input is received
        # on_trigger: activate the event when the trigger is activated
        # once: fulfill the condition if the input is received once
        # periodic: periodically activate this event
        self.type: str = None
        self.process_event: bool = is_process_event

        self.triggers: List["Event"] = []  # children triggers
        self.actions: List["Event"] = []  # event to trigger when this event is activated
        self.trigger_root_ids: List[str] = []  # trigger root ids

        self.frequency: float = None
        self.warn_rate: float = None
        self.error_rate: float = None
        self.timeout: float = None
        self.is_set: bool = False

    def set_type(self, type_str):
        if type_str not in self.type_list:
            raise ValueError(f"Invalid event type: {type_str}")
        self.type = type_str

    def check_trigger_root_ids(self, trigger_root_id):
        if trigger_root_id in self.trigger_root_ids:
            return True
        else:
            self.trigger_root_ids.append(trigger_root_id)
            return False

    def add_trigger_event(self, event, vise_versa=True):
        if event.id == self.id:
            raise ValueError(f"Event cannot trigger itself: {self.id}")
        # check if the event is already in the list
        for e in self.triggers:
            if e.id == event.id:
                return
        # relationship is established
        self.triggers.append(event)
        if vise_versa:
            event.add_action_event(self, False)

    def add_action_event(self, event, vise_versa=True):
        if event.id == self.id:
            raise ValueError(f"Event cannot trigger itself: {self.id}")
        # check if the event is already in the list
        for e in self.actions:
            if e.id == event.id:
                return
        # relationship is established
        self.actions.append(event)
        if vise_versa:
            event.add_trigger_event(self, False)

    def determine_type(self, config_yaml):
        if len(config_yaml) == 0:
            raise ValueError("Config is empty")
        # check the first element
        first_config = list(config_yaml)[0]
        if isinstance(first_config, str):
            # if the first element is string, it is the type
            type_key = first_config
            value = config_yaml.get(type_key)
        elif isinstance(first_config, dict):
            # in case of dict, the first key is the type
            type_key = list(first_config.keys())[0]
            value = first_config[type_key]
        return type_key, value

    def set_trigger(
        self,
        config_yaml,
        process_list: List["Event"],
        on_input_list: List["Event"],
    ):
        # get the config type
        config_key, config_value = self.determine_type(config_yaml)

        # convert to dict if the type is list and the size is 1
        if isinstance(config_yaml, list) and len(config_yaml) == 1:
            config_yaml = config_yaml[0]

        if config_key in self.type_list:
            # incoming event
            if config_key == "periodic":
                self.frequency = config_value
                self.is_set = True
            elif config_key == "once" and config_value is None:
                self.frequency = 0.0
                self.warn_rate = 0.0
                self.error_rate = 0.0
                self.timeout = 0.0
                self.is_set = True
            elif config_key == "on_input" or config_key == "once":
                self.condition_value = config_value
                # search the event in the on_input_list
                is_found = False
                for event in on_input_list:
                    if event.name == ("input_" + config_value):
                        self.add_trigger_event(event)
                        is_found = True
                        break
                # if not found, warn
                if not is_found:
                    raise ValueError(f"Input event not found: {config_value}")
            elif config_key == "on_trigger":
                self.condition_value = config_value
                # search the event in the process_list
                is_found = False
                for event in process_list:
                    if event.name == (config_value):
                        self.add_trigger_event(event)
                        is_found = True
                        break
                # if not found, warn
                if not is_found:
                    raise ValueError(f"Trigger event not found: {config_value}")
            else:
                raise ValueError(f"Invalid event type to set trigger: {config_key}")

            # set the topic monitor configurations, if available
            if "warn_rate" in config_yaml.keys():
                self.warn_rate = config_yaml.get("warn_rate")
            if "error_rate" in config_yaml.keys():
                self.error_rate = config_yaml.get("error_rate")
            if "timeout" in config_yaml.keys():
                self.timeout = config_yaml.get("timeout")

            # debug
            # print(f"Event '{self.name}' is set as {self.type}, {config_key}")
            # print(f" triggers: {[t.name for t in self.triggers]}, input_list: {[e.name for e in on_input_list]}, process_list: {[e.name for e in process_list]}")
        else:
            raise ValueError(f"Invalid event type: {config_key}")

    def set_event_frequency(
        self,
        trigger_root_id: str,
        frequency: float,
        warn_rate: float,
        error_rate: float,
        timeout: float,
    ):
        # debug
        # print(f"Event '{self.name}' set_event_frequency: {frequency}, {warn_rate}, {error_rate}, {timeout}")

        # if the event is already set, update the values and do not propagate
        # it is for loop prevention
        if self.check_trigger_root_ids(trigger_root_id):
            return

        # update the frequency, take higher value
        if frequency is not None:
            if self.frequency is None or frequency > self.frequency:
                self.frequency = frequency
        # update the warn_rate, take higher value
        if warn_rate is not None:
            if self.warn_rate is None or warn_rate > self.warn_rate:
                self.warn_rate = warn_rate
        # update the error_rate, take higher value
        if error_rate is not None:
            if self.error_rate is None or error_rate > self.error_rate:
                self.error_rate = error_rate
        # update the timeout, take higher value
        if timeout is not None:
            if self.timeout is None or timeout > self.timeout:
                self.timeout = timeout

        # set the flag
        self.is_set = True

        # propagate the frequency to the children
        for action in self.actions:
            action.set_event_frequency(trigger_root_id, frequency, warn_rate, error_rate, timeout)

    def set_frequency_tree(self):
        trigger_root_id = self.id
        if self.type == "periodic" and self.is_set:
            # propagate the frequency to the children
            for action in self.actions:
                action.set_event_frequency(
                    trigger_root_id, self.frequency, self.warn_rate, self.error_rate, self.timeout
                )
        elif self.type == "once" and self.is_set:
            # propagate the frequency to the children
            for action in self.actions:
                action.set_event_frequency(trigger_root_id, 0.0, 0.0, 0.0, 0.0)
        else:
            pass


class EventChain(Event):
    def __init__(self, name: str, namespace: List[str] = [], is_process_event=True):
        super().__init__(name, namespace, is_process_event)
        self.chain_list = [
            "and",
            "or",
        ]
        # and: all children triggers are activated, this event is activated
        # or: any of the children triggers are activated, this event is activated
        self.children: List[Event] = []

    def set_type(self, type_str):
        if type_str not in self.chain_list + self.type_list:
            raise ValueError(f"Invalid event chain type: {type_str}")
        self.type = type_str

    def set_chain(
        self,
        config_yaml,
        process_list: List[Event],
        on_input_list: List[Event],
        child_idx=0,
    ):
        # debug
        # print(f"EventChain '{self.name}' set_chain: {config_yaml}")

        if isinstance(config_yaml, dict):
            config_key, config_value = self.determine_type(config_yaml)
            if config_key in self.type_list:
                self.set_type(config_key)
                self.set_trigger(config_yaml, process_list, on_input_list)
            elif config_key in self.chain_list:
                if len(config_value) == 1:
                    # this is an event
                    self.set_chain(config_value[0], process_list, on_input_list)
                else:
                    self.set_type(config_key)  # and/or
                    # recursively set the triggers
                    for chain in config_value:
                        chain_key, chain_value = self.determine_type(chain)
                        if chain_key in ["periodic"] + self.chain_list:
                            event = EventChain(self.name + "_" + str(child_idx), self.namespace)
                            child_idx += 1
                            event.set_chain(chain, process_list, on_input_list)
                            self.add_trigger_event(event)
                            self.children.append(event)
                        elif chain_key in self.type_list:
                            self.set_trigger(chain, process_list, on_input_list)
                        else:
                            raise ValueError(f"Invalid trigger condition type: {chain_key}")
        elif isinstance(config_yaml, list):
            # check the length
            length = len(config_yaml)
            if length == 1:
                # this is a dictionary
                self.set_chain(config_yaml[0], process_list, on_input_list)
            else:
                # make a dictionary that the key is "or" and the value is the list
                config_dict = {"or": config_yaml}
                self.set_chain(config_dict, process_list, on_input_list)

    def get_children(self):
        # recursively get the children
        children_list = []
        for child in self.children:
            children_list += child.get_children()
        return children_list + self.children


class Process:
    def __init__(self, name: str, namespace: List[str], config_yaml: dict):
        self.name = name
        self.namespace = namespace
        self.config_yaml = config_yaml
        self.event: EventChain = EventChain(name, namespace)
        self.id = ("__".join(namespace) + "__process__" + name).replace("/", "__")

    def set_condition(self, process_list, on_input_list):
        trigger_condition_config = self.config_yaml.get("trigger_conditions")

        # debug
        # print(f"Process {self.name} trigger condition: {trigger_condition_config}")
        self.event.set_chain(trigger_condition_config, process_list, on_input_list)

    def set_outcomes(self, process_list, to_output_events):
        outcome_config = self.config_yaml.get("outcomes")
        for outcome in outcome_config:
            # parse the outcome type
            outcome_type = list(outcome.keys())[0]
            outcome_value = outcome[outcome_type]
            if outcome_type == "to_output":
                # search the event in the to_output_events
                for event in to_output_events:
                    if event.name == ("output_" + outcome_value):
                        self.event.add_action_event(event)
                        break
                # if not found, warn
                if self.event.actions == []:
                    raise ValueError(f"Output event not found: {outcome_value}")
            elif outcome_type == "to_trigger":
                # search the event in the process_list
                for event in process_list:
                    if event.name == outcome_value:
                        self.event.add_action_event(event)
                        break
                # if not found, warn
                if self.event.actions == []:
                    raise ValueError(f"Trigger event not found: {outcome_value}")

        # debug
        # print(f"Process '{self.name}' outcomes: {outcome_config}")
        # print(f"  actions: {[t.id for t in self.event.actions]}, to_output: {[e.name for e in to_output_events]}")

    def get_event_list(self):
        return self.event.get_children() + [self.event]


class Port:
    def __init__(self, name: str, msg_type: str, namespace: List[str] = []):
        self.name = name
        self.msg_type = msg_type
        self.namespace = namespace
        self.full_name = "/" + "/".join(namespace) + "/" + name
        self.reference: List["Port"] = []
        self.topic: List[str] = []
        self.event = None
        self.is_global = False

    def set_references(self, port_list: List["Port"]):
        # check if the port is already in the reference list
        reference_name_list = [p.full_name for p in self.reference]
        for port in port_list:
            if port.full_name not in reference_name_list:
                self.reference.append(port)

    def get_reference_list(self):
        if self.reference == []:
            return [self]
        return self.reference

    def set_topic(self, topic_namespace: List[str], topic_name: str):
        self.topic = topic_namespace + [topic_name]


class InPort(Port):
    def __init__(self, name, msg_type, namespace: List[str] = []):
        super().__init__(name, msg_type, namespace)
        self.full_name = "/" + "/".join(namespace) + "/input/" + name
        self.id = ("__".join(namespace) + "__input_" + name).replace("/", "__")
        # to enable/disable connection checker
        self.is_required = True
        # reference port
        self.servers: List[Port] = []
        # trigger event
        self.event = Event("input_" + name, namespace)
        self.event.set_type("on_input")

    def set_servers(self, port_list: List[Port]):
        # check if the port is already in the reference list
        server_name_list = [p.full_name for p in self.servers]
        for port in port_list:
            if port.full_name not in server_name_list:
                self.servers.append(port)


class OutPort(Port):
    def __init__(self, name, msg_type, namespace: List[str] = []):
        super().__init__(name, msg_type, namespace)
        self.full_name = "/" + "/".join(namespace) + "/output/" + name
        self.id = ("__".join(namespace) + "__output_" + name).replace("/", "__")
        # for topic monitor
        self.frequency = 0.0
        self.is_monitored = False
        # reference port
        self.users: List[Port] = []
        # action event
        self.event = Event("output_" + name, namespace)
        self.event.set_type("to_output")

    def set_users(self, port_list: List[Port]):
        # check if the port is already in the reference list
        user_name_list = [p.full_name for p in self.users]
        for port in port_list:
            if port.full_name not in user_name_list:
                self.users.append(port)


class Link:
    # Link is a connection between two ports
    def __init__(self, msg_type: str, from_port: Port, to_port: Port, namespace: List[str] = []):
        self.msg_type: str = msg_type
        # from-port and to-port connection
        self.from_port: Port = from_port
        self.to_port: Port = to_port
        # namespace
        self.namespace: List[str] = namespace

        self.check_connection()

    def check_connection(self):
        # if the from port is OutPort, it is internal port
        is_from_port_internal = isinstance(self.from_port, OutPort)
        # if the to port is InPort, it is internal port
        is_to_port_internal = isinstance(self.to_port, InPort)

        # case 1: from internal output to internal input
        if is_from_port_internal and is_to_port_internal:
            # propagate and finish the connection
            from_port_list = self.from_port.get_reference_list()
            to_port_list = self.to_port.get_reference_list()

            # check the message type is the same
            for from_port in from_port_list:
                if from_port.msg_type != self.msg_type:
                    raise ValueError(f"Invalid connection: {from_port.name} -> {self.to_port.name}")
            for to_port in to_port_list:
                if to_port.msg_type != self.msg_type:
                    raise ValueError(
                        f"Type mismatch: {self.msg_type}({self.from_port.name}) -> {to_port.msg_type}({to_port.name})"
                    )

            # link the ports
            for from_port_ref in from_port_list:
                from_port_ref.set_users(to_port_list)
            for to_port_ref in to_port_list:
                to_port_ref.set_servers(from_port_list)

            # determine the topic, set it to the from-ports to publish and to-ports to subscribe
            for from_port_ref in from_port_list:
                from_port_ref.set_topic(self.from_port.namespace, self.from_port.name)
            for to_port_ref in to_port_list:
                to_port_ref.set_topic(self.from_port.namespace, self.from_port.name)

            # set the trigger event of the to-port
            for to_port_ref in to_port_list:
                for server_port in to_port_ref.servers:
                    to_port_ref.event.add_trigger_event(server_port.event)

        # case 2: from internal output to external output
        elif is_from_port_internal and not is_to_port_internal:
            # bring the from-port reference to the to-port reference
            reference_port_list = self.from_port.get_reference_list()
            self.to_port.set_references(reference_port_list)
            # set the topic name to the external output, whether it is connected or not
            for reference_port in reference_port_list:
                reference_port.set_topic(self.to_port.namespace, self.to_port.name)

        # case 3: from external input to internal input
        elif not is_from_port_internal and is_to_port_internal:
            # bring the to-port reference to the from-port reference
            reference_port_list = self.to_port.get_reference_list()
            self.from_port.set_references(reference_port_list)

        # case 4: from-port is InPort and to-port is OutPort
        #   bypass connection, which is invalid
        else:
            raise ValueError(f"Invalid connection: {self.from_port.name} -> {self.to_port.name}")


class Connection:
    # Connection is a connection between two elements
    # In other words, it is a configuration to create link(s)
    def __init__(self, connection_dict: dict):

        # connection type
        #   0: undefined
        #   1: external input to internal input
        #   2: internal output to internal input
        #   3: internal output to external output
        self.type = 0

        connection_from = connection_dict.get("from")
        if connection_from is None:
            raise ValueError(f"Connection couldn't found : {connection_dict}")
        connection_to = connection_dict.get("to")
        if connection_to is None:
            raise ValueError(f"Connection couldn't found : {connection_dict}")

        from_instance, from_port_name = self.parse_port_name(connection_from)
        to_instance, to_port_name = self.parse_port_name(connection_to)

        if from_instance == "" and to_instance == "":
            raise ValueError(f"Invalid connection: {connection_dict}")
        elif from_instance == "" and to_instance != "":
            self.type = 1
        elif from_instance != "" and to_instance == "":
            self.type = 3
        elif from_instance != "" and to_instance != "":
            self.type = 2

        self.from_instance: str = from_instance
        self.from_port_name: str = from_port_name
        self.to_instance: str = to_instance
        self.to_port_name: str = to_port_name

    def parse_port_name(self, port_name: str) -> (str, str):  # (instance_name, port_name)
        name_splitted = port_name.split(".")
        if len(name_splitted) == 2:
            if name_splitted[0] == "input":
                return "", name_splitted[1]  # external input
            if name_splitted[0] == "output":
                return "", name_splitted[1]  # external output
            raise ValueError(f"Invalid port name: {port_name}")
        elif len(name_splitted) == 3:
            if name_splitted[1] == "input":
                return name_splitted[0], name_splitted[2]  # internal input
            if name_splitted[1] == "output":
                return name_splitted[0], name_splitted[2]  # internal output
            raise ValueError(f"Invalid port name: {port_name}")
        else:
            raise ValueError(f"Invalid port name: {port_name}")


class Parameter:
    def __init__(self, name: str, value: str):
        self.name = name
        self.value = value


class ParameterList:
    def __init__(self):
        self.list: List[Parameter] = []

    def get_parameter(self, parameter_name):
        for parameter in self.list:
            if parameter.name == parameter_name:
                return parameter.value
        # not found, return None
        return None

    def set_parameter(self, parameter_name, parameter_value):
        for parameter in self.list:
            if parameter.name == parameter_name:
                parameter.value = parameter_value
                return
        # if not found, add it
        self.list.append(Parameter(parameter_name, parameter_value))
