"""Base class for action scopes."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Generator, Iterable
from contextlib import contextmanager
from typing import TYPE_CHECKING, Annotated, Literal, TypeVar, overload

import launch
from launch import Action, SomeSubstitutionsType, Substitution
from launch.actions import GroupAction, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handler import BaseEventHandler
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities.type_utils import SomeValueType
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import Parameter
from launch_ros.parameters_type import SomeParameterFile, SomeParameters, SomeParametersDict, SomeParameterValue
from launch_ros.remap_rule_type import SomeRemapRule, SomeRemapRules
from launch_ros.substitutions import FindPackageShare

from launch_util.gazebo import GazeboBridge, GazeboType, ros_gz_prefix
from launch_util.substitutions import SomeSubstitutionsValueTypeDict, WriteTempFile, Xacro
from launch_util.util import flatten_substitutions

if TYPE_CHECKING:
    from launch_util.composable import ComposableContainerContext

__all__ = [
    "ActionScope",
]

A = TypeVar("A", bound=Action)

logger = launch.logging.get_logger(__name__)


# noinspection PyMethodMayBeStatic
class ActionScope(ABC):
    """A base class for scopes that actions can be added to."""

    @contextmanager
    def group(
            self,
            namespace: SomeSubstitutionsType | None = None,
            scoped: bool = True,
            forwarding: bool = True,
    ) -> Generator[ActionScope]:
        """
        Creates a sub-group.

        :param namespace: The namespace of the group.
        :param scoped: If the group is scoped.
        :param forwarding: If the group is forwarding.
        """

        # local import to prevent cyclic import error
        from launch_util.group import ActionGroup  # noqa: PLC0415

        group = ActionGroup(self, namespace, scoped, forwarding)  # ty: ignore[invalid-argument-type]

        try:
            yield group  # ty: ignore[invalid-yield]
        finally:
            self.add_action(group.create_action())

    @overload
    @contextmanager
    def container(
            self,
            name: SomeSubstitutionsType,
            namespace: SomeSubstitutionsType,
            existing: Literal[False] = False,
            **kwargs,
    ) -> Generator[ComposableContainerContext]:
        pass

    @overload
    @contextmanager
    def container(
            self,
            name: SomeSubstitutionsType,
            namespace: None,
            existing: Literal[True],
            **kwargs,
    ) -> Generator[ComposableContainerContext]:
        pass

    @contextmanager
    def container(
            self,
            name: SomeSubstitutionsType,
            namespace: SomeSubstitutionsType | None = None,
            existing: bool = False,
            add: bool = True,
            **kwargs,
    ) -> Generator[ComposableContainerContext]:
        """
        Opens a Composition group to add nodes.

        If existing is True, then loads nodes in the (supposedly) existing container

        :param name:
        :param namespace:
        :param existing: Attempts to load the nodes into an already existing container instead of creating a new one.
        :param kwargs: Addition arguments to pass to the composable container.
        :param add: If the node should be added.
        :return:
        """

        # local import to prevent cyclic import error
        from launch_util.composable import ComposableContainerContext  # noqa: PLC0415

        # ty: ignore[invalid-argument-type]: why does it think self is not ActionScope...
        composable_context = ComposableContainerContext(name, namespace, existing, self, **kwargs)

        try:
            yield composable_context
        finally:
            if add:
                self.add_action(composable_context.create_action())

    @abstractmethod
    def add_action(self, action: A) -> A:
        """Adds an Action (Node, ComposableNodes, Group, etc.) at the current group level."""

    @property
    @abstractmethod
    def scope_included_files(self) -> bool:
        """If included files should be scoped."""

    @property
    @abstractmethod
    def default_parameters(self) -> dict[str, SomeSubstitutionsType]:
        """A dictionary of default parameters to pass to every node and every included launch file."""

    def add_event(self, event_handler: BaseEventHandler) -> RegisterEventHandler:
        """
        Adds an event to the list of events.

        :param event_handler: The event handler.
        :return: The registered event handler.
        """

        return self.add_action(RegisterEventHandler(event_handler=event_handler))

    @abstractmethod
    def make_parameters(
            self,
            parameters: SomeParameters | SomeParameterFile | Parameter | SomeParametersDict | None = None,
            exclude_default_params: bool = False,
    ) -> SomeParameters | None:
        """
        Normalizes the provided parameters, optionally appending the default parameters.

        :param parameters: The parameters to normalize.
        :param exclude_default_params: If the default parameters should be excluded.
        :return: The normalized parameters.
        """

    def node(  # noqa: PLR0913, PLR0917
            self,
            package: SomeSubstitutionsType,
            executable: SomeSubstitutionsType | None = None,
            name: SomeSubstitutionsType | None = None,
            namespace: SomeSubstitutionsType | None = None,
            exec_name: SomeSubstitutionsType | None = None,
            parameters: SomeParameters | SomeParameterFile | Parameter | SomeParametersDict | None = None,
            remappings: SomeRemapRules | None = None,
            ros_arguments: Iterable[SomeSubstitutionsType] | None = None,
            arguments: Iterable[SomeSubstitutionsType] | None = None,
            exclude_default_params: bool = False,
            add: bool = True,
            **node_args,
    ) -> Node:
        """
        Adds a new node to the launch tree.

        If use_sim_time was used then the use_sim_time parameter will be set if not explicitly given.

        **See also:** :class:`launch_ros.actions.Node`

        :param package: The package in which the node executable can be found.
        :param executable: The name of the executable to find if a package
            is provided or otherwise a path to the executable to run.
        :param name: The name of the node.
        :param namespace: The ROS namespace for this Node.
        :param exec_name: The label used to represent the process.
            Defaults to the basename of node executable.
        :param parameters: List of names of yaml files with parameter rules,
            or dictionaries of parameters.
        :param remappings: Ordered list of 'to' and 'from' string pairs to be
            passed to the node as ROS remapping rules
        :param ros_arguments: List of ROS arguments for the node
        :param arguments: List of extra arguments for the node
        :param exclude_default_params: If the default parameters should be excluded.
        :param add: If the node should be added to the launch tree.
        :param node_args: Any other arguments passed to the node constructor.
        :return: The newly created Node.
        """

        if executable is None:
            executable = package

        parameters = self.make_parameters(parameters, exclude_default_params)

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

        return self.add_action(node) if add else node

    def log_info(self, msg: SomeSubstitutionsType) -> LogInfo:
        """
        Adds an action which logs a message to the console.

        :param msg: The log message
        :return: The :class:`LogInfo` action.
        """

        return self.add_action(LogInfo(msg=msg))

    def include(
            self,
            *,
            package: SomeSubstitutionsType | None = None,
            directory: SomeSubstitutionsType = "launch",
            launch_file: SomeSubstitutionsType | None = None,
            launch_arguments: Iterable[tuple[SomeSubstitutionsType, SomeSubstitutionsType]] | None = None,
            exclude_default_args: bool = False,
    ) -> GroupAction | IncludeLaunchDescription:
        """
        Include another launch file.

        :param package: The package to include the launch file from.
        :param directory: The directory the launch file is in, defaults to ``launch``.
        :param launch_file: The name of the launch file.
        :param launch_arguments: The launch arguments to pass to the launch file.
        :param exclude_default_args: If the default arguments should be passed to the included launch file or not.
        :return: The action that includes the launch file.
        """

        paths = []
        if package is not None:
            paths.append(FindPackageShare(package))
        paths.append(directory)

        if launch_file is None:
            logger.error("The launch file cannot be None for an include()")
            # noinspection PyTypeChecker
            return None  # ty:ignore[invalid-return-type]

        paths.append(launch_file)

        if not exclude_default_args:
            if launch_arguments is None:
                launch_arguments: list[tuple[SomeSubstitutionsType, SomeSubstitutionsType]] = []
            else:
                launch_arguments: list[tuple[SomeSubstitutionsType, SomeSubstitutionsType]] = list(launch_arguments)

            additional_args = self.default_parameters

            launch_arguments.extend(additional_args.items())

        launch_file = PathJoinSubstitution(paths)

        inclusion = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(launch_file),
            launch_arguments=launch_arguments,
        )

        if self.scope_included_files:
            return self.add_action(GroupAction(scoped=True, actions=[inclusion]))

        return self.add_action(inclusion)

    def rviz(self, config_file: SomeSubstitutionsType | None = None, warnings: bool = False) -> Node:
        """
        Adds an action which runs RViz.

        :param config_file: The RViz config file.
        :param warnings: If warnings should be enabled.
        :return: The action that executes RViz.
        """

        args: list[SomeSubstitutionsType] = []

        # TODO 2026-02-07 (Will Free): replace this with several IfElseSubstitutions
        if config_file is not None:
            args += ["-d", config_file]

        # TODO 2026-02-07 (Will Free): replace this with several IfElseSubstitutions
        if not warnings:
            args += ["--ros-args", "--log-level", "FATAL"]
        return self.node("rviz2", "rviz2", arguments=args)

    def robot_description(
            self,
            *,
            package: SomeSubstitutionsType,
            directory: SomeSubstitutionsType = "urdf",
            file: SomeSubstitutionsType,
            xacro_args: SomeSubstitutionsValueTypeDict | None = None,
            strip_comments: SomeValueType | None = None,
    ) -> Substitution:
        """
        Returns a substitution for the robot description after expanding it with xacro.

        :param package: The package the description is in.
        :param directory: The directory the description is in, defaulting to ``urdf``.
        :param file: The file name of the description.
        :param xacro_args: The arguments to pass to xacro.
        :param strip_comments: If comments should be stripped from the file.
        :return: The xacro substitution.
        """
        urdf_path = PathJoinSubstitution([FindPackageShare(package), directory, file])

        return Xacro(file_path=urdf_path, mappings=xacro_args, strip_comments=strip_comments)

    def robot_state_publisher(  # noqa: PLR0913
            self,
            *,
            package: SomeSubstitutionsType,
            directory: SomeSubstitutionsType = "urdf",
            file: SomeSubstitutionsType,
            xacro_args: SomeSubstitutionsValueTypeDict | None = None,
            strip_comments: SomeValueType | None = None,
            publish_frequency: SomeParameterValue | None = None,
            ignore_timestamp: SomeParameterValue | None = None,
            frame_prefix: SomeParameterValue | None = None,
            name: SomeSubstitutionsType | None = None,
            namespace: SomeSubstitutionsType | None = None,
            exec_name: SomeSubstitutionsType | None = None,
            parameters: SomeParameters | None = None,
            remappings: SomeRemapRules | None = None,
            ros_arguments: Iterable[SomeSubstitutionsType] | None = None,
            arguments: Iterable[SomeSubstitutionsType] | None = None,
            **node_args,
    ) -> Node:
        """
        Adds a robot state publisher action.

        :param package: The package the robot description is in.
        :param directory: The directory the description is in, defaulting to ``urdf``.
        :param file: The file name of the description.
        :param xacro_args: The arguments to pass to xacro.
        :param strip_comments: If comments should be stripped from the file.
        :param publish_frequency: The frequency at which to publish non-static transforms.
        :param ignore_timestamp: Whether to accept all joint states regardless of the timestamp.
        :param frame_prefix: Prefix to add to the tf2 frames.
        :param name: The name of the robot state publisher node.
        :param namespace: The namespace of the robot state publisher node.
        :param exec_name: The label used to represent the robot state publisher node.
            Defaults to the basename of node executable.
        :param parameters: Additional parameters passed to the robot state publisher node.
        :param remappings: Topic remappings for the robot state publisher node.
        :param ros_arguments: Additional ROS arguments for the robot state publisher node.
        :param arguments: List of extra arguments for the robot state publisher node.
        :param node_args: Additional node arguments.
        :return: The robot state publisher action.
        """

        # TODO 2026-02-07 (Will Free): add support for use_robot_description_topic parameter

        description = self.robot_description(package=package, directory=directory, file=file, xacro_args=xacro_args, strip_comments=strip_comments)

        parameters: Annotated[SomeParameters, list] = [] if parameters is None else [*parameters]

        # TODO 2026-02-07 (Will Free): it's not always safe to just add robot_description to the parameters.
        #       the robot_description parameter might be specified in a file or smth else
        params_dict: dict[str, SomeParameterValue] = {
            "robot_description": description,
        }

        if publish_frequency is not None:
            params_dict["publish_frequency"] = publish_frequency

        if ignore_timestamp is not None:
            params_dict["ignore_timestamp"] = ignore_timestamp

        if frame_prefix is not None:
            params_dict["frame_prefix"] = frame_prefix

        # noinspection PyUnresolvedReferences
        parameters.append(params_dict)

        # Launch the robot state publisher with the desired URDF
        return self.node(
            package="robot_state_publisher",
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
        Adds a joint_state_publisher / joint_state_publisher_gui with passed arguments as parameters.

        Assumes some robot_description topic is published inside the namespace
        :param use_gui: Whether the joint state publisher gui node should be launched.
        :param node_args: Additional node arguments.
        """

        self.node("joint_state_publisher", condition=UnlessCondition(use_gui), add=False, **node_args)
        self.node("joint_state_publisher_gui", condition=IfCondition(use_gui), add=False, **node_args)

    @contextmanager
    def gz_bridge(self, name: SomeSubstitutionsType = "gz_bridge") -> Generator[GazeboBridge]:
        """
        Returns a context that can be used to configure a gazebo bridge.

        Attempting to bridge any image topics will automatically add an image bridge node as well.

        :param name: The name of the gazebo bridge node.
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
            bridge_node = self.node(
                package=f"{ros_gz}_bridge",
                executable="parameter_bridge",
                name=name,
                parameters=[{"config_file": temp_file_substitution}],
            )
            self.add_event(temp_file_substitution.on_exit(bridge_node))

            if im_topics:
                # use remapping to ROS topics
                remappings: list[SomeRemapRule] = []

                for topic in im_topics:
                    for ext in ("", "/compressed", "/compressedDepth", "/g", "/zstd"):
                        gz_topic = flatten_substitutions([topic.gz_topic, ext])
                        ros_topic = flatten_substitutions([topic.ros_topic, ext])
                        remappings.append((gz_topic, ros_topic))

                image_bridge_params: dict[str, SomeParameterValue] = {}

                if bridge.image_bridge_qos is not None:
                    image_bridge_params["qos"] = bridge.image_bridge_qos.value
                if bridge.image_bridge_lazy is not None:
                    image_bridge_params["lazy"] = bridge.image_bridge_lazy
                if bridge.image_bridge_subscription_heartbeat is not None:
                    image_bridge_params["subscription_heartbeat"] = bridge.image_bridge_subscription_heartbeat

                self.node(
                    f"{ros_gz}_image",
                    "image_bridge",
                    name=flatten_substitutions([name, "_image"]),
                    arguments=[topic.gz_topic for topic in im_topics],
                    remappings=remappings,
                    parameters=[image_bridge_params],
                )

    @abstractmethod
    def gazebo_world(
            self,
            package: SomeSubstitutionsType | None = None,
            directory: SomeSubstitutionsType = "worlds",
            file: SomeSubstitutionsType | None = None,
            xacro_args: SomeSubstitutionsValueTypeDict | None = None,
            strip_comments: SomeValueType | None = None,
    ) -> Substitution:
        """
        Expands a file for a gazebo world with xacro as a substitution.

        :param package: The package the gazebo world is in.
        :param directory: The directory the gazebo world is in, defaulting to ``worlds``.
        :param file: The file name of the gazebo world.
        :param xacro_args: Arguments to pass to xacro.
        :param strip_comments: If comments should be stripped from the XML file.
        :return: A substitution for the gazebo world.
        """

    def gz_launch(  # noqa: PLR0913, PLR0917
            self,
            package: SomeSubstitutionsType | None = None,
            directory: SomeSubstitutionsType = "worlds",
            file: SomeSubstitutionsType | None = None,
            xacro_args: SomeSubstitutionsValueTypeDict | None = None,
            strip_comments: SomeValueType | None = None,
            server_args: SomeSubstitutionsType | None = None,
            client_args: SomeSubstitutionsType | None = None,
            gz_args: SomeSubstitutionsType | None = None,
            debug: bool = False,
    ):
        """
        Adds an action which launches gazebo with a given world.

        :param package: The package the gazebo world file is in.
        :param directory: The directory the gazebo world file is in, defaulting to ``worlds``.
        :param file: The file name of the gazebo world.
        :param xacro_args: The arguments to pass to xacro for expanding the gazebo world.
        :param strip_comments: If comments should be stripped from the gazebo world.
        :param server_args: Additional arguments to pass to the gazebo server.
        :param client_args: Additional arguments to pass to the gazebo client.
        :param gz_args: Additional arguments to pass to gazebo.
        :param debug: If debug logs should be enabled.
        """

        world = self.gazebo_world(package, directory, file, xacro_args, strip_comments)
        temp_world_file = WriteTempFile(suffix="yml", prefix="gz_bridge_", contents=world)

        # normalize substitution so it's always a list
        server_args: list[SomeSubstitutionsType] = [*normalize_to_list_of_substitutions(server_args or [])]
        client_args: list[SomeSubstitutionsType] = [*normalize_to_list_of_substitutions(client_args or [])]
        gz_args: list[SomeSubstitutionsType] = [*normalize_to_list_of_substitutions(gz_args or []), " "]

        # add separation space (so when it's flattened, there are always spaces separating flags)

        if debug:
            gz_args.append("-v4 ")

        client_args = [*gz_args, *client_args, " -g"]

        server_args = [*gz_args, *server_args, " -r -s ", temp_world_file]

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
                args_name: flatten_substitutions(server_args),
                "on_exit_shutdown": "true",
            }.items(),
            exclude_default_args=True,
        )

        self.add_event(temp_world_file.on_exit(server_node))

        self.include(
            package=package,
            launch_file=launch_file,
            launch_arguments={
                args_name: flatten_substitutions(client_args),
            }.items(),
            exclude_default_args=True,
        )

    def spawn_gz_model(
            self,
            name: SomeSubstitutionsType,
            topic: SomeSubstitutionsType = "robot_description",
            model_file: SomeSubstitutionsType | None = None,
            spawn_args: SomeSubstitutionsType | None = None,
            add: bool = True,
    ) -> Node:
        """
        Spawns a model into Gazebo under the given name, from the given topic or file.

        Additional spawn_args can be given to specify e.g. the initial pose

        :param name: The name of the gazebo model.
        :param topic: The topic to spawn the model from.
        :param model_file: The name of the file to spawn the model from.
        :param spawn_args: Additional arguments to pass when spawning.
        :param add: If the spawn node should be added to the launch tree.
        :return: The node that spawns the gazebo model.
        """

        if spawn_args is None:
            spawn_args = []  # ty: ignore[unsound-assignment]

        if model_file is not None:
            spawn_args = flatten_substitutions([spawn_args, "-file", model_file, "-name", name])
        else:
            spawn_args = flatten_substitutions([spawn_args, "-topic", topic, "-name", name])

        pkg = "ros_ign_gazebo" if ros_gz_prefix() == GazeboType.IGN else "ros_gz_sim"
        return self.node(package=pkg, executable="create", arguments=spawn_args, add=add)
