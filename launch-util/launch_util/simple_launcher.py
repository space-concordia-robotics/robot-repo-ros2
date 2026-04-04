from contextlib import contextmanager
from typing import Iterable, TypeVar, Generator, Callable, Sequence, Union, Text, Any, Tuple, List, Optional, Dict

import launch
from launch import LaunchDescription, Condition, Action, LaunchContext, SomeSubstitutionsType, Substitution
from launch.actions import IncludeLaunchDescription, LogInfo, OpaqueFunction, DeclareLaunchArgument, RegisterEventHandler, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handler import BaseEventHandler
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable, EqualsSubstitution, TextSubstitution, LaunchConfiguration
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities.type_utils import SomeValueType
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameters_type import SomeParameters, SomeParameterValue
from launch_ros.remap_rule_type import SomeRemapRules, SomeRemapRule
from launch_ros.substitutions import FindPackageShare

from .events import When
from .gazebo import GazeboType, GazeboBridge, ros_gz_prefix
from .group import Group
from .substitutions import SomeSubstitutionsValueTypeDict
from .substitutions import Xacro, WriteTempFile
from .util import flatten_substitutions

__all__ = [
    "SimpleLauncher",
]

A = TypeVar("A", bound=Action)

logger = launch.logging.get_logger(__name__)


# noinspection PyMethodMayBeStatic
class SimpleLauncher:
    __groups: List[Group]
    __cur_group: Group
    gz_axes: List[Text]
    __context: Optional[LaunchContext]
    scope_included_files: bool

    __mode_arg: SomeSubstitutionsType
    __control_arg: SomeSubstitutionsType
    __sim_time_param: Substitution

    def __init__(
            self,
            mode: Optional[SomeSubstitutionsType] = None,
            control: Optional[SomeSubstitutionsType] = None,
            namespace: SomeSubstitutionsType = None,
            scope_included_files=False,
    ):
        """
        Initializes entities in the given workspace
        scope_included_files will make this launch file include other ones with scoped arguments, so that modifying them does not override the ones of this launch file
        """
        # group tree
        self.__groups = [Group(namespace)]
        self.__cur_group = self.__groups[0]
        self.gz_axes = ["x", "y", "z", "yaw", "pitch", "roll"]
        self.__context = None
        self.scope_included_files = scope_included_files

        self.__mode_arg = self.declare_arg(
            "mode",
            default_value="production",
            choices=["production", "simulation"],
            description="The operation mode to use",
        ) if mode is None else mode

        self.__control_arg = self.declare_arg(
            "control",
            default_value="ros",
            choices=["ros", "gazebo"],
            description="The control mode to use",
        ) if control is None else control

        self.__sim_time_param = EqualsSubstitution(self.mode_arg, TextSubstitution(text="simulation"))

    # TODO 2026-02-07 (Will Free): automatically convert ints, floats, and booleans in default values to strings
    def declare_arg(
            self,
            name,
            *,
            default_value: Optional[SomeSubstitutionsType] = None,
            description: Optional[Text] = None,
            choices: Optional[List[Text]] = None,
            **kwargs,
    ) -> LaunchConfiguration:
        """
        Add an argument to the launch file
        """
        if self.__has_context():
            logger.error(f"declaring a launch argument \"{name}\" while inside an opaque function\nyou should declare the arguments before the function")

        def to_string_nested(elem):
            if isinstance(elem, str):
                return elem
            elif not isinstance(elem, Iterable):
                return str(elem)
            elem = list(elem)
            for i, item in enumerate(elem):
                elem[i] = to_string_nested(item)
            return elem

        self.__groups[0].add_action(
            DeclareLaunchArgument(
                name,
                default_value=default_value,
                description=description,
                choices=choices,
                **kwargs,
            ),
        )

        return self.arg(name)

    def arg(self, name: SomeSubstitutionsType, *, default: Optional[Union[Any, Iterable[Any]]] = None) -> LaunchConfiguration:
        """
        Retrieve an argument, should be a string otherwise -s will crash
        """
        return LaunchConfiguration(name, default=default)

    @property
    def mode_arg(self) -> SomeSubstitutionsType:
        return self.__mode_arg

    @property
    def control_arg(self) -> SomeSubstitutionsType:
        return self.__control_arg

    def arg_map(self, *names: SomeSubstitutionsType) -> Dict[SomeSubstitutionsType, Substitution]:
        """
        Retrieves several arguments as a dict
        """

        return {
            name: self.arg(name) for name in names
        }

    def ros_distro(self) -> Substitution:
        """
        Returns the name of the currently sourced ros distro (e.g. ``$ROS_VERSION``)
        """
        return EnvironmentVariable("ROS_DISTRO")

    def ros_version(self) -> Substitution:
        """
        Returns the name of the currently sourced ros version (e.g. ``$ROS_VERSION``)
        """
        return EnvironmentVariable("ROS_VERSION")

    def params(
            self,
            *,
            package: Optional[SomeSubstitutionsType] = None,
            directory: SomeSubstitutionsType = "config",
            file: Optional[SomeSubstitutionsType] = None,
    ) -> PathJoinSubstitution:
        return self.find(package, directory, file)

    def find(
            self,
            package: Optional[SomeSubstitutionsType] = None,
            directory: Optional[SomeSubstitutionsType] = None,
            file: Optional[SomeSubstitutionsType] = None,
    ) -> PathJoinSubstitution:
        return PathJoinSubstitution([FindPackageShare(package), directory, file])

    def launch_description(self, opaque_function=None) -> List[Action] | LaunchDescription | Callable[[], LaunchDescription] | Any:
        """
        Returns the setup launch description
        This value can be directly returned from generate_launch_description()
        It can also be stored in order to add custom entities

        If opaque_function is not None, then returns the generate_launch_description() wrapper around the passed function
        """

        if opaque_function is None:
            # classical call without opaque function
            main_actions = self.__cur_group.close()
            return main_actions if self.__has_context() else LaunchDescription(main_actions)

        if self.__has_context():
            # opaque function but we already have context
            logger.error("Calling SimpleLauncher.launch_description with opaque function within an opaque function makes it really opaque")

        # the actual generate_launch_description function, just wrapping the passed opaque function
        def generate_launch_description():
            def wrapped_opaque_function(context):
                self.__context = context
                return opaque_function()

            return LaunchDescription(self.__cur_group.close() + [OpaqueFunction(function=wrapped_opaque_function)])

        return generate_launch_description

    def __has_context(self) -> bool:
        return self.__context is not None

    @contextmanager
    def group(
            self,
            ns=None,
            if_arg: SomeSubstitutionsType = None,
            unless_arg: SomeSubstitutionsType = None,
            if_condition: SomeSubstitutionsType = None,
            unless_condition: SomeSubstitutionsType = None,
            when=None,
    ):
        """
        Group the next nodes / entities into
         - another namespace
         - a if / unless condition depending on some argument
         - a raw if / unless condition that may come out of an expression
         - an event condition (from simple_launch import OnExit, OnStart
        """
        if self.__cur_group.is_container():
            logger.error("Container \"%s\": Group blocks cannot be nested inside container blocks. Use OpaqueFunction or reverse the logic.",
                         self.__cur_group.is_container())

        # we can only deal with 1 condition
        conditions = (if_arg, unless_arg, if_condition, unless_condition)
        if len(conditions) - conditions.count(None) > 1:
            logger.error("group blocks cannot have more than 1 condition (has %i)", len(conditions) - conditions.count(None))

        # get condition and cast it
        condition = None
        if if_arg is not None:
            condition = IfCondition(self.arg(if_arg))
        elif unless_arg is not None:
            condition = UnlessCondition(self.arg(unless_arg))
        elif if_condition is not None:
            condition = IfCondition(if_condition)
        elif unless_condition is not None:
            condition = UnlessCondition(unless_condition)

        self.__groups.append(Group(ns=ns, condition=condition, parent=self.__cur_group, when=when))
        self.__cur_group = self.__groups[-1]

        try:
            yield self
        finally:
            self.__cur_group = self.__cur_group.close()

    @contextmanager
    def container(self, name, namespace="", existing=False, package="rclcpp_components", executable="component_container", **container_args):
        """
        Opens a Composition group to add nodes
        If existing is True, then loads nodes in the (supposely) existing container
        """
        # TODO 2026-02-07 (Will Free): uhh, should this group here have a namespace?
        self.__groups.append(Group(parent=self.__cur_group, container=name))
        self.__cur_group = self.__groups[-1]

        try:
            yield self
        finally:
            self.__cur_group, composed = self.__cur_group.close()

            # store ComposableNodes inside a Container
            if existing:
                self.add_action(
                    LoadComposableNodes(
                        composable_node_descriptions=composed,
                        target_container=name))
            else:
                self.add_action(
                    ComposableNodeContainer(
                        name=name,
                        namespace=namespace,
                        package=package,
                        executable=executable,
                        composable_node_descriptions=composed,
                        **container_args))

    def add_action(self, action: A) -> A:
        """
        Adds an Action (Node, ComposableNodes, Group, etc.) at the current group level
        """
        self.__cur_group.add_action(action)
        return action

    def add_event(self, event_handler: BaseEventHandler) -> RegisterEventHandler:
        return self.add_action(RegisterEventHandler(event_handler=event_handler))

    def node(
            self,
            package: Optional[SomeSubstitutionsType] = None,
            executable: Optional[SomeSubstitutionsType] = None,
            plugin: Optional[SomeSubstitutionsType] = None,
            name: Optional[SomeSubstitutionsType] = None,
            namespace: Optional[SomeSubstitutionsType] = None,
            exec_name: Optional[SomeSubstitutionsType] = None,
            parameters: Optional[SomeParameters] = None,
            remappings: Optional[SomeRemapRules] = None,
            ros_arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
            arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
            extra_arguments: Optional[SomeParameters] = None,
            condition: Optional[Condition] = None,
            exclude_default_params: bool = False,
            add: bool = True,
            **node_args,
    ) -> Node | ComposableNode:
        """
        Add a node to the launch tree. If use_sim_time was used then the use_sim_time parameter will be set if not explicitly given

        * package -- name of the package
        * executable (classical node) -- name of the node within the package, if None then assumes the node has the name of the package
        * plugin (inside a composition group) -- name of the composed node plugin within the package
        * node_args -- any other args passed to the node constructor
        """
        as_composable = self.__cur_group.is_container()

        if plugin is None and as_composable:
            logger.error("Indicate the plugin name when adding a composable node to a container")

        if executable is None:
            executable = package

        if not exclude_default_params:
            additional_params = {
                "mode": self.__mode_arg,
                "control": self.__control_arg,
                "use_sim_time": self.__sim_time_param,
            }

            if parameters is None:
                parameters = [additional_params]
            else:
                parameters += [additional_params]

        # TODO 2026-02-06 (Will Free): I don"t think this is right for composable nodes...
        if as_composable:
            # check plugin name - add package if needed
            if "::" not in plugin:
                plugin = "{}::{}".format(package, plugin)
            # noinspection PyTypeChecker
            node = ComposableNode(
                package=package,
                plugin=plugin,
                name=name,
                namespace=namespace,
                parameters=parameters,
                remappings=remappings,
                extra_arguments=extra_arguments,
                condition=condition,
            )
        else:
            node = Node(
                package=package,
                executable=executable,
                name=name,
                namespace=namespace,
                exec_name=exec_name,
                parameters=parameters,
                remappings=remappings,
                ros_arguments=ros_arguments,
                arguments=arguments,
                **node_args,
            )

        if add:
            return self.add_action(node)
        else:
            return node

    def log_info(self, msg):
        """
        Adds a LogInfo
        """
        return self.add_action(LogInfo(msg=msg))

    def include(
            self,
            *,
            package: Optional[SomeSubstitutionsType] = None,
            directory: SomeSubstitutionsType = "launch",
            launch_file: Optional[SomeSubstitutionsType] = None,
            launch_arguments: Optional[
                Iterable[Tuple[SomeSubstitutionsType, SomeSubstitutionsType]]
            ] = None,
            exclude_default_args: bool = False,
    ) -> GroupAction | IncludeLaunchDescription:
        """
        Include another launch file
        """
        paths = []
        if package is not None:
            paths += [FindPackageShare(package)]
        if directory is not None:
            paths += [directory]

        if launch_file is None:
            logger.error("The launch file cannot be None for an include()")
            # noinspection PyTypeChecker
            return None

        paths += [launch_file]

        if not exclude_default_args:
            if launch_arguments is None:
                launch_arguments: List[Tuple[SomeSubstitutionsType, SomeSubstitutionsType]] = []
            else:
                launch_arguments: List[Tuple[SomeSubstitutionsType, SomeSubstitutionsType]] = list(launch_arguments)

            additional_args = {
                "mode": self.__mode_arg,
                "control": self.__control_arg,
                "use_sim_time": self.__sim_time_param,
            }

            launch_arguments.extend(additional_args.items())

        launch_file = PathJoinSubstitution([path for path in paths if path is not None])

        inclusion = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(launch_file),
            launch_arguments=launch_arguments,
        )

        if self.scope_included_files:
            # run included launch in a group to avoid polluting scope
            return self.add_action(GroupAction([inclusion]))
        # just use default include behavior
        return self.add_action(inclusion)

    # TODO 2026-02-07 (Will Free): remove
    def call_service(self, server, request=None, verbosity="", **kwargs):
        """
        Calls the service at server address after checking its type.
        Request is a dictionary that is forwarded to service request fields, assuming they match
        verbosity is none or "req", "res" or "reqres" to get information on service call
        """
        params = {"simple_launch.server": server, "simple_launch.verbosity": verbosity}
        if request is not None:
            params.update(request)
        return self.node("simple_launch", "call_service", parameters=params, **kwargs)

    # TODO 2026-02-07 (Will Free): remove
    def set_parameters(self, node_name, parameters: dict = {}, verbosity="", **kwargs):
        """
        Sets the requested parameters for this node
        verbosity is none or "req", "res" or "reqres" to get information on service call
        """
        params = {"simple_launch.node": node_name, "simple_launch.keys": list(parameters.keys()), "simple_launch.verbosity": verbosity}
        params.update(parameters)
        return self.node("simple_launch", "set_parameters",
                         parameters=params,
                         **kwargs)

    def rviz(self, config_file=None, warnings=False):
        """
        Runs RViz with the given config file and warning level
        """

        args = []

        # TODO 2026-02-07 (Will Free): replace this with several IfElseSubstitutions
        if config_file is not None:
            args += ["-d", config_file]

        # TODO 2026-02-07 (Will Free): replace this with several IfElseSubstitutions
        if not warnings:
            args += ["--ros-args", "--log-level", "FATAL"]
        self.node("rviz2", "rviz2", arguments=args)

    def robot_description(
            self,
            package: Optional[SomeSubstitutionsType] = None,
            directory: SomeSubstitutionsType = "urdf",
            file: Optional[SomeSubstitutionsType] = None,
            xacro_args: Optional[SomeSubstitutionsValueTypeDict] = None,
            strip_comments: Optional[SomeValueType] = None,
    ) -> Substitution:
        """
        Returns the robot description after potential xacro parse if the file ends with xacro or xacro_args are defined
        """
        urdf_path = PathJoinSubstitution([FindPackageShare(package), directory, file])

        return Xacro(file_path=urdf_path, mappings=xacro_args, strip_comments=strip_comments)

    def robot_state_publisher(
            self,
            package: Optional[SomeSubstitutionsType] = None,
            directory: SomeSubstitutionsType = "urdf",
            file: SomeSubstitutionsType = None,
            xacro_args: Optional[SomeSubstitutionsValueTypeDict] = None,
            strip_comments: Optional[SomeValueType] = None,
            publish_frequency: Optional[SomeParameterValue] = None,
            ignore_timestamp: Optional[SomeParameterValue] = None,
            frame_prefix: Optional[SomeParameterValue] = None,
            name: Optional[SomeSubstitutionsType] = None,
            namespace: Optional[SomeSubstitutionsType] = None,
            exec_name: Optional[SomeSubstitutionsType] = None,
            parameters: Optional[SomeParameters] = None,
            remappings: Optional[SomeRemapRules] = None,
            ros_arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
            arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
            **node_args,
    ) -> Node:
        """
        Add a robot state publisher node to the launch tree using the given description (urdf / xacro) file.

        * package -- is the name of the package that contains the description file (if None then assume an absolute description file)
        * file -- is the name of the urdf/xacro file
        * description_dir -- the name of the directory containing the file (None to have it found)
        * xacro_args -- arguments passed to xacro (will force use of xacro)
        * node_args -- any additional node arguments such as remappings or parameters
        """

        # TODO 2026-02-07 (Will Free): add support for use_robot_description_topic parameter

        description = self.robot_description(package, directory, file, xacro_args, strip_comments)

        parameters = [] if parameters is None else parameters

        # TODO 2026-02-07 (Will Free): it"s not always safe to just add robot_description to the parameters.
        #       the robot_description parameter might be specified in a file or smth else
        params_dict: Dict[Text, SomeParameterValue] = {
            "robot_description": description,
        }

        if publish_frequency is not None:
            params_dict["publish_frequency"] = publish_frequency

        if ignore_timestamp is not None:
            params_dict["ignore_timestamp"] = ignore_timestamp

        if frame_prefix is not None:
            params_dict["frame_prefix"] = frame_prefix

        parameters.append(params_dict)

        # Launch the robot state publisher with the desired URDF
        return self.node(
            "robot_state_publisher",
            name=name,
            namespace=namespace,
            exec_name=exec_name,
            parameters=parameters,
            remappings=remappings,
            ros_arguments=ros_arguments,
            arguments=arguments,
            **node_args,
        )

    def joint_state_publisher(self, use_gui: SomeSubstitutionsType = "True", **node_args):
        """
        Adds a joint_state_publisher / joint_state_publisher_gui with passed arguments as parameters
        Assumes some robot_description topic is published inside the namespace
        """
        if type(use_gui) == bool:
            use_gui = str(use_gui)

        self.node("joint_state_publisher", condition=UnlessCondition(use_gui), **node_args)
        self.node("joint_state_publisher_gui", condition=IfCondition(use_gui), **node_args)

    # Gazebo / Ignition methods

    def declare_gazebo_axes(
            self,
            disabled_axes: Optional[Sequence[Text]] = None,
            enabled_axes: Optional[Sequence[Text]] = None,
            **defaults: SomeSubstitutionsType,
    ):
        """
        Declares classical Gazebo axes as launch arguments
        If axes is void then declares all 6 axes with default value 0
        Otherwise declares the given axes with the given defaults
        """
        allowed_axes = {"x", "y", "z", "yaw", "pitch", "roll"}

        if enabled_axes is None:
            enabled_axes = allowed_axes

        if disabled_axes is None:
            disabled_axes = {}

        self.gz_axes = [axis for axis in enabled_axes if ((axis in allowed_axes) and (not axis in disabled_axes))]

        for axis in self.gz_axes:
            self.declare_arg(axis, default_value=defaults[axis] if axis in defaults else "0.0")

    def gazebo_axes_args(self):
        """
        Generate arguments corresponding to Gazebo spawner
        """
        axes = {"x": "x", "y": "y", "z": "z", "roll": "R", "pitch": "P", "yaw": "Y"}
        args: List[SomeSubstitutionsType] = []
        for axis, param in axes.items():
            # TODO 2026-02-07 (Will Free): should I really default to 0.0 here or just exclude the axis if it"s not in self.gz_axes?
            args.extend([f"-{param}", self.arg(axis) if axis in self.gz_axes else "0.0"])

        return args

    @contextmanager
    def gz_bridge(self, name: SomeSubstitutionsType = "gz_bridge") -> Generator[GazeboBridge, None, None]:
        """
        Create a ros_gz_bridge::parameter_bridge with the passed GazeboBridge instances
        The bridge has a default name if not specified
        If any bridge is used for sensor_msgs/Image, ros_{gz,ign}_image will be used instead
        """

        # TODO 2026-02-07 (Will Free): add support for override_timestamps_with_wall_time for bridge (see README for ros_gz_bridge)

        bridge = GazeboBridge()

        try:
            yield bridge
        finally:
            ros_gz = "ros_" + ros_gz_prefix().value

            # add camera_info for image bridges
            im_topics = [topic for topic in bridge.topics if topic.is_image]

            temp_file_substitution = WriteTempFile(suffix="yml", prefix="gz_bridge_", contents=bridge.yaml)
            # TODO 2026-02-06 (Will Free): QoS overrides?
            bridge_node = self.node(f"{ros_gz}_bridge", "parameter_bridge", name=name, parameters=[{"config_file": temp_file_substitution}])
            self.add_event(temp_file_substitution.on_exit(bridge_node))

            if len(im_topics):
                # use remapping to ROS topics
                remappings: List[SomeRemapRule] = []

                for topic in im_topics:
                    for ext in ("", "/compressed", "/compressedDepth", "/theora"):
                        gz_topic = flatten_substitutions([topic.gz_topic, ext])
                        ros_topic = flatten_substitutions([topic.ros_topic, ext])
                        remappings.append((gz_topic, ros_topic))

                self.node(
                    f"{ros_gz}_image",
                    "image_bridge",
                    name=flatten_substitutions([name, "_image"]),
                    arguments=[topic.gz_topic for topic in im_topics],
                    remappings=remappings,
                )

    # def gz_world_tf(self, world_frame=None):
    #     """
    #     Runs a static_transform_publisher to connect `world` and Gazebo world name, if different from `world`
    #     """
    #     if world_frame is None:
    #         world_frame = GazeboBridge.world()
    #     if world_frame != "world":
    #         self.node("tf2_ros", "static_transform_publisher",
    #                   name="gz_world_tf",
    #                   arguments=["--frame-id", "world", "--child-frame-id", world_frame])

    # def create_gz_clock_bridge(self, name="gz_clock_bridge"):
    #     """
    #     Create a ros_gz_bridge::parameter_bridge for the /clock topic
    #     Typically used in the launch file that runs the simulation before spawning things in
    #     """
    #     self.create_gz_bridge(GazeboBridge.clock(), name)

    def gazebo_world(
            self,
            package: Optional[SomeSubstitutionsType] = None,
            directory: SomeSubstitutionsType = "worlds",
            file: Optional[SomeSubstitutionsType] = None,
            xacro_args: Optional[SomeSubstitutionsValueTypeDict] = None,
            strip_comments: Optional[SomeValueType] = None,
    ) -> Substitution:
        """
        Returns the robot description after potential xacro parse if the file ends with xacro or xacro_args are defined
        """
        urdf_path = PathJoinSubstitution([FindPackageShare(package), directory, file])

        return Xacro(file_path=urdf_path, mappings=xacro_args, strip_comments=strip_comments)

    def gz_launch(
            self,
            package: Optional[SomeSubstitutionsType] = None,
            directory: SomeSubstitutionsType = "worlds",
            file: Optional[SomeSubstitutionsType] = None,
            xacro_args: Optional[SomeSubstitutionsValueTypeDict] = None,
            strip_comments: Optional[SomeValueType] = None,
            server_args: Optional[SomeSubstitutionsType] = None,
            client_args: Optional[SomeSubstitutionsType] = None,
            gz_args: Optional[SomeSubstitutionsType] = None,
            debug: bool = False,
    ):
        world = self.gazebo_world(package, directory, file, xacro_args, strip_comments)
        temp_world_file = WriteTempFile(suffix="yml", prefix="gz_bridge_", contents=world)

        # normalize substitution so it"s always a list
        server_args: List[SomeSubstitutionsType] = normalize_to_list_of_substitutions(server_args) if server_args is not None else []
        client_args: List[SomeSubstitutionsType] = normalize_to_list_of_substitutions(client_args) if client_args is not None else []
        gz_args: List[SomeSubstitutionsType] = normalize_to_list_of_substitutions(gz_args) if gz_args is not None else []

        # add separation space (so when it"s flattened, there are always spaces separating flags)
        gz_args.append(" ")

        if debug:
            gz_args.append("-v4 ")

        client_args = gz_args + client_args + [" -g"]

        server_args = gz_args + server_args + [" -r -s ", temp_world_file]

        if ros_gz_prefix() == GazeboType.GZ:
            package = "ros_gz_sim"
            launch_file = "gz_sim.launch.py"
            args_name = "gz_args"
        else:
            package = "ros_ign_gazebo"
            launch_file = "ign_gazebo.launch.py"
            args_name = "ign_args"

        server_node = self.include(
            package=package,
            launch_file=launch_file,
            launch_arguments={
                args_name: server_args,
                "on_exit_shutdown": "true",
            }.items(),
            exclude_default_args=True,
        )

        self.add_event(temp_world_file.on_exit(server_node))

        self.include(
            package=package,
            launch_file=launch_file,
            launch_arguments={
                args_name: client_args,
            }.items(),
            exclude_default_args=True,
        )

    def save_gz_world(self, dst, after=5.):
        """
        Saves the current world under dst
        Resolves any spawned URDF through their description parameter and converts to SDF
        """
        with self.group(when=When(delay=after)):
            self.node("simple_launch", "generate_gz_world", arguments=[dst])

    def spawn_gz_model(self, name, topic="robot_description", model_file=None, spawn_args=[], add: bool = True):
        """
        Spawns a model into Gazebo under the given name, from the given topic or file
        Additional spawn_args can be given to specify e.g. the initial pose
        """

        if model_file is not None:
            spawn_args = flatten_substitutions(spawn_args + ["-file", model_file, "-name", name])
        else:
            spawn_args = flatten_substitutions(spawn_args + ["-topic", topic, "-name", name])

        pkg = "ros_ign_gazebo" if ros_gz_prefix().value == GazeboType.IGN else "ros_gz_sim"
        return self.node(package=pkg, executable="create", arguments=spawn_args, add=add)
