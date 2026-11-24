"""Composable node scope."""
from __future__ import annotations

from typing import TYPE_CHECKING, Any

from launch import Condition, SomeSubstitutionsType
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import Parameter
from launch_ros.parameters_type import SomeParameterFile, SomeParameters, SomeParametersDict
from launch_ros.remap_rule_type import SomeRemapRules

from launch_util.util import flatten_substitutions

if TYPE_CHECKING:
    from launch_util.action_scope import ActionScope

__all__ = [
    "ComposableContainerContext",
]


class ComposableContainerContext:
    """A context for loading composable notes into a container."""

    __name: SomeSubstitutionsType
    __namespace: SomeSubstitutionsType | None
    __existing: bool
    __composable_nodes: list[ComposableNode]
    __scope: ActionScope
    __kwargs: dict[str, Any]

    def __init__(self, name: SomeSubstitutionsType, namespace: SomeSubstitutionsType | None, existing: bool, scope: ActionScope, **kwargs) -> None:
        """
        Creates a new composable container context.

        **See also:**
            - :class:`launch_ros.descriptions.ComposableNode`
            - :class:`launch_ros.actions.ComposableNodeContainer`
            - :class:`launch_ros.actions.LoadComposableNodes`

        :param name: The name of the composable container.
        :param namespace: The namespace of the composable container.
        :param existing: If this should instead append to an existing composable container
        :param scope: The parent scope.
        :param kwargs: Any additional args to pass to the composable container
        """
        self.__name = name
        self.__namespace = namespace
        self.__existing = existing
        self.__composable_nodes = []
        self.__scope = scope
        self.__kwargs = kwargs

    def composable_node(  # noqa: PLR0913, PLR0917
            self,
            package: SomeSubstitutionsType,
            plugin: SomeSubstitutionsType,
            name: SomeSubstitutionsType | None = None,
            namespace: SomeSubstitutionsType | None = None,
            parameters: SomeParameters | SomeParameterFile | Parameter | SomeParametersDict | None = None,
            remappings: SomeRemapRules | None = None,
            extra_arguments: SomeParameters | None = None,
            condition: Condition | None = None,
            exclude_default_params: bool = False,
    ):
        """
        Adds a composable node to be loaded into a container.

        :param package: name of the package
        :param plugin: name of the composed node plugin within the package to load (you can omit the package name)
        :param name: name to give to the ROS node
        :param namespace: namespace to give to the ROS node
        :param parameters: list of either paths to yaml files or dictionaries of parameters
        :param remappings: list of from/to pairs for remapping names
        :param extra_arguments: container specific arguments to be passed to the loaded node
        :param condition: action will be executed if the condition evaluates to true
        :param exclude_default_params: if default parameters from simple_launcher should be excluded
        """

        parameters = self.__scope.make_parameters(parameters, exclude_default_params)

        # check plugin name - add package if needed
        if isinstance(plugin, str) and "::" not in plugin:
            plugin: SomeSubstitutionsType = flatten_substitutions([package, "::", plugin])
        assert plugin is not None

        self.__composable_nodes.append(
            ComposableNode(
                package=package,
                plugin=plugin,
                name=name,
                namespace=namespace,
                parameters=parameters,
                remappings=remappings,
                extra_arguments=extra_arguments,
                condition=condition,
            ),
        )

    def create_action(self) -> ComposableNodeContainer | LoadComposableNodes:
        """
        Creates an action that loads the composable nodes.

        :return: The action to load the composable nodes.
        """

        if self.__existing:
            return LoadComposableNodes(
                target_container=self.__name,
                composable_node_descriptions=self.__composable_nodes,
                **self.__kwargs,
            )

        assert self.__namespace is not None

        return ComposableNodeContainer(
            name=self.__name,
            namespace=self.__namespace,
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=self.__composable_nodes,
            **self.__kwargs,
        )

    @property
    def composable_nodes(self) -> list[ComposableNode]:
        """Returns all the added composable nodes."""
        return self.__composable_nodes
