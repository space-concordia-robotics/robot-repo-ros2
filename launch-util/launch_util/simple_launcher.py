"""Simple launcher action scope."""

import typing
from collections.abc import Callable, Iterable, Sequence
from typing import Any, TypeVar, Union, override

import launch
from launch import Action, LaunchContext, LaunchDescription, LaunchDescriptionEntity, SomeSubstitutionsType, Substitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.utilities.type_utils import SomeValueType
from launch_ros.parameter_descriptions import Parameter
from launch_ros.parameters_type import SomeParameterFile, SomeParameters, SomeParametersDict
from launch_ros.substitutions import FindPackageShare

from launch_util.action_scope import ActionScope
from launch_util.substitutions import SomeSubstitutionsValueTypeDict, Xacro
from launch_util.util import flatten_substitutions

__all__ = [
    "SimpleLauncher",
]

A = TypeVar("A", bound=Action)

SingleValueType = Union[str, int, float, bool]  # noqa: UP007

logger = launch.logging.get_logger(__name__)


# noinspection PyMethodMayBeStatic
class SimpleLauncher(ActionScope):
    """
    Simple launcher helps simplify writing ROS launch scripts.

    ..
        TODO 2026-11-21 (Will Free): better explanation of features + maybe some examples?
    """

    gz_axes: list[str]
    __context: LaunchContext | None
    __scope_included_files: bool

    __actions: list[LaunchDescriptionEntity]

    __mode_arg: SomeSubstitutionsType
    __control_arg: SomeSubstitutionsType
    __sim_time_param: Substitution

    def __init__(
            self,
            mode: SomeSubstitutionsType | None = None,
            control: SomeSubstitutionsType | None = None,
            scope_included_files: bool = False,
    ):
        """
        Creates a new SimpleLauncher.

        scope_included_files will make this launch file include other ones with scoped arguments,
        so that modifying them does not override the ones of this launch file

        :param mode:
        :param control:
        :param scope_included_files: If included launch files should be scoped.
        """

        self.gz_axes = ["x", "y", "z", "yaw", "pitch", "roll"]
        self.__context = None
        self.__scope_included_files = scope_included_files

        self.__actions = []

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

    def declare_arg(
            self,
            name: str,
            *,
            default_value: SomeSubstitutionsType | SingleValueType | None = None,  # type: ignore[unsupported-operator]
            description: str | None = None,
            choices: list[str] | None = None,
            **kwargs,
    ) -> LaunchConfiguration:
        """
        Declares a new argument for this launch file.

        **See also:**
            - :class:`DeclareLaunchArgument`
            - :class:`LaunchConfiguration`

        :param name: The name of the argument.
        :param default_value: The default value of the argument.
        :param description: The description of the argument.
        :param choices: The available choices for the argument, ``None`` if the choices are not restricted.
        :param kwargs: Additional arguments to pass to :class:`DeclareLaunchArgument`
        :return: The launch configuration for the argument.
        """

        if self.__has_context():
            logger.error('declaring a launch argument "%s" while inside an opaque function\nyou should declare the arguments before the function', name)

        if isinstance(default_value, (int, float, bool)):
            default_value = str(default_value)

        self.add_action(
            DeclareLaunchArgument(
                name,
                default_value=default_value,
                description=description,
                choices=choices,
                **kwargs,
            ),
        )

        return self.arg(name)

    def arg(self, name: SomeSubstitutionsType, *, default: Any | Iterable[Any] | None = None) -> LaunchConfiguration:
        """Retrieve an argument, should be a string otherwise -s will crash."""
        """
        Retrieves the :class:`LaunchConfiguration` for a previously declared argument.

        :param name:
        :param default:
        :return:
        """
        return LaunchConfiguration(name, default=default)

    @property
    def mode_arg(self) -> SomeSubstitutionsType:
        """:return: The mode argument substitution."""

        return self.__mode_arg

    @property
    def control_arg(self) -> SomeSubstitutionsType:
        """:return: The control type argument substitution."""

        return self.__control_arg

    @property
    def sim_time_param(self) -> SomeSubstitutionsType:
        """:return: The simulation type parameter substitution."""

        return self.__sim_time_param

    def arg_map(self, *names: SomeSubstitutionsType) -> dict[SomeSubstitutionsType, LaunchConfiguration]:
        """
        Retrieves several arguments as a dict.

        :param names: The names of the arguments
        :return: The argument substitutions in a dictionary.
        """

        return {
            name: self.arg(name) for name in names
        }

    def ros_distro(self) -> Substitution:
        """:return: The name of the currently sourced ros distro (e.g. ``$ROS_VERSION``)."""

        return EnvironmentVariable("ROS_DISTRO")

    def ros_version(self) -> Substitution:
        """:return: the name of the currently sourced ros version (e.g. ``$ROS_VERSION``)."""

        return EnvironmentVariable("ROS_VERSION")

    def params(
            self,
            *,
            package: SomeSubstitutionsType | None = None,
            directory: SomeSubstitutionsType = "config",
            file: SomeSubstitutionsType | None = None,
    ) -> PathJoinSubstitution:
        """
        Creates a substitution for the path to a parameter file.

        :param package: The package the parameter file is in.
        :param directory: The directory the parameter file is in, defaulting to ``config``.
        :param file: The name of the parameter file.
        :return: The path substitution for the parameter file.
        """

        return self.find(package, directory, file)

    def find(
            self,
            package: SomeSubstitutionsType | None = None,
            directory: SomeSubstitutionsType | None = None,
            file: SomeSubstitutionsType | None = None,
    ) -> PathJoinSubstitution:
        """
        Creates a substitution for the path to a file.

        :param package: The package the file is in.
        :param directory: The directory the file is in.
        :param file: The name of the file.
        :return: The path substitution for the file.
        """

        # noinspection PyTypeChecker
        return PathJoinSubstitution([FindPackageShare(package), directory, file])  # ty:ignore[invalid-argument-type]

    def launch_description(
            self,
            opaque_function: Callable[[], None] | None = None,
    ) -> LaunchDescription | Iterable[LaunchDescriptionEntity] | Callable[[], LaunchDescription]:
        """
        Returns the setup launch description.

        This value can be directly returned from ``generate_launch_description()``
        It can also be stored in order to add custom entities

        If ``opaque_function`` is not ``None``, then it returns a wrapper function around the passed function, so that :class:`LaunchContext` can be saved.

        :param opaque_function: An opaque function to add to the launch actions.
        :return: The LaunchDescription, a wrapper function around it, or an iterable of launch description entities.
        """

        if opaque_function is None:
            # classical call without opaque function
            return self.__actions if self.__has_context() else LaunchDescription(self.__actions)

        if self.__has_context():
            # opaque function but we already have context
            logger.error("Calling SimpleLauncher.launch_description with opaque function within an opaque function makes it really opaque")

        # the actual generate_launch_description function, just wrapping the passed opaque function
        def generate_launch_description() -> LaunchDescription:
            def wrapped_opaque_function(context: LaunchContext) -> Any:
                self.__context = context

                assert opaque_function is not None

                return opaque_function()

            return LaunchDescription([*self.__actions, OpaqueFunction(function=wrapped_opaque_function)])

        return generate_launch_description

    def __has_context(self) -> bool:
        return self.__context is not None

    @override
    def add_action(self, action: A) -> A:
        self.__actions.append(action)
        return action

    @property
    @override
    def scope_included_files(self) -> bool:
        return self.__scope_included_files

    @property
    @override
    def default_parameters(self) -> dict[str, SomeSubstitutionsType]:
        return {
            "mode": self.__mode_arg,
            "control": self.__control_arg,
            "use_sim_time": self.__sim_time_param,
        }

    @override
    def make_parameters(
            self,
            parameters: SomeParameters | SomeParameterFile | Parameter | SomeParametersDict | None = None,
            exclude_default_params: bool = False,
    ) -> SomeParameters | None:
        if parameters is not None and isinstance(parameters, Sequence):
            # noinspection PyUnnecessaryCast
            parameters = typing.cast(Sequence[SomeParameterFile | Parameter | SomeParametersDict], parameters)
        elif parameters is not None:
            parameters = [parameters]

        if not exclude_default_params:
            additional_params = self.default_parameters

            parameters = [additional_params] if parameters is None else [*parameters, additional_params]

        return parameters

    def declare_gazebo_axes(
            self,
            disabled_axes: Iterable[str] | None = None,
            enabled_axes: Iterable[str] | None = None,
            **defaults: SomeSubstitutionsType,
    ):
        """
        Declares classical Gazebo axes as launch arguments.

        If axes is void then declares all 6 axes with default value 0
        Otherwise declares the given axes with the given defaults
        """
        allowed_axes = {"x", "y", "z", "yaw", "pitch", "roll"}

        if enabled_axes is None:
            enabled_axes: set[str] = allowed_axes

        if disabled_axes is None:
            disabled_axes = typing.cast(Iterable[str], [])  # ruff complains here without the cast

        self.gz_axes = [axis for axis in enabled_axes if ((axis in allowed_axes) and (axis not in disabled_axes))]

        for axis in self.gz_axes:
            self.declare_arg(axis, default_value=defaults.get(axis, "0.0"))

    def gazebo_axes_args(self) -> SomeSubstitutionsType:
        """Generate arguments corresponding to Gazebo spawner."""
        axes = {"x": "x", "y": "y", "z": "z", "roll": "R", "pitch": "P", "yaw": "Y"}
        args: list[SomeSubstitutionsType] = []
        for axis, param in axes.items():
            # TODO 2026-02-07 (Will Free): should I really default to 0.0 here or just exclude the axis if it's not in self.gz_axes?
            args.extend([f"-{param}", self.arg(axis, default="0.0")] if axis in self.gz_axes else ["0.0"])

        return flatten_substitutions(args)

    @override
    def gazebo_world(
            self,
            package: SomeSubstitutionsType | None = None,
            directory: SomeSubstitutionsType = "worlds",
            file: SomeSubstitutionsType | None = None,
            xacro_args: SomeSubstitutionsValueTypeDict | None = None,
            strip_comments: SomeValueType | None = None,
    ) -> Substitution:
        urdf_path = self.find(package, directory, file)

        return Xacro(file_path=urdf_path, mappings=xacro_args, strip_comments=strip_comments)
