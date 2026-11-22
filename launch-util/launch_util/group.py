"""Action group scopes."""

from __future__ import annotations

from typing import TypeVar, override

from launch import Action, SomeSubstitutionsType, Substitution
from launch.actions import GroupAction
from launch.utilities.type_utils import SomeValueType
from launch_ros.actions import PushRosNamespace
from launch_ros.parameter_descriptions import Parameter
from launch_ros.parameters_type import SomeParameterFile, SomeParameters, SomeParametersDict

from launch_util.action_scope import ActionScope
from launch_util.substitutions import SomeSubstitutionsValueTypeDict

__all__ = [
    "ActionGroup",
]

A = TypeVar("A", bound=Action)


class ActionGroup(ActionScope):
    """
    A scoped action group.

    **See also:** :class:`GroupAction`
    """

    __parent: ActionScope
    __namespace: SomeSubstitutionsType | None
    __scoped: bool
    __forwarding: bool
    __actions: list[Action]

    def __init__(self, parent: ActionScope, namespace: SomeSubstitutionsType | None, scoped: bool, forwarding: bool):
        """
        Creates a new action group.

        **See also:** :class:`launch.actions.GroupAction`

        :param parent: The parent scope.
        :param namespace: The namespace for this group.
        :param scoped: If the action group is scoped.
        :param forwarding: If the action group is forwarding.
        """

        super().__init__()
        self.__parent = parent
        self.__namespace = namespace
        self.__scoped = scoped
        self.__forwarding = forwarding
        self.__actions = []

    @property
    @override
    def scope_included_files(self) -> bool:
        return self.__parent.scope_included_files

    @property
    @override
    def default_parameters(self) -> dict[str, SomeSubstitutionsType]:
        return self.__parent.default_parameters

    @override
    def add_action(self, action: A) -> A:
        self.__actions.append(action)
        return action

    @override
    def make_parameters(
            self,
            parameters: SomeParameters | SomeParameterFile | Parameter | SomeParametersDict | None = None,
            exclude_default_params: bool = False,
    ) -> SomeParameters | None:
        return self.__parent.make_parameters(parameters, exclude_default_params)

    def create_action(self) -> GroupAction:
        """
        Creates a :class:`GroupAction` for the current group.

        :return: The newly created :class:`GroupAction`
        """

        actions = list(self.__actions)
        if self.__namespace is not None:
            actions.insert(0, PushRosNamespace(self.__namespace))
        return GroupAction(
            scoped=self.__scoped,
            forwarding=self.__forwarding,
            actions=actions,
        )

    @override
    def gazebo_world(
            self,
            package: SomeSubstitutionsType | None = None,
            directory: SomeSubstitutionsType = "worlds",
            file: SomeSubstitutionsType | None = None,
            xacro_args: SomeSubstitutionsValueTypeDict | None = None,
            strip_comments: SomeValueType | None = None,
    ) -> Substitution:
        return self.__parent.gazebo_world(package, directory, file, xacro_args, strip_comments)
