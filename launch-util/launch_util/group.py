# ruff: noqa: D100, D101, D102, D107
from __future__ import annotations

import typing

from launch import Action, Condition, SomeSubstitutionsType
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode

from launch_util.events import When

__all__ = [
    "Group",
]


class Group:
    __parent: Group | None
    __condition: Condition | None
    __container: str
    __when: When | None
    __root: Group
    __ns: list[SomeSubstitutionsType]
    __actions: list[Action | ComposableNode]

    def __init__(
            self,
            ns: SomeSubstitutionsType | None = None,
            condition: Condition | None = None,
            container: str = "",
            parent: Group | None = None,
            when: When | None = None,
    ):
        self.__parent = parent
        self.__condition = condition
        self.__container = container
        self.__when = when

        # inherit namespace from this branch
        ns: list[SomeSubstitutionsType] = [ns] if ns is not None else []
        if parent is None:
            self.__root = self
            self.__ns = ns
        else:
            self.__root = parent.__root  # noqa: SLF001
            self.__ns = parent.__ns + ns  # noqa: SLF001

        # actions are always active and added to the root group
        self.__actions = []
        # managed are added as Event or Containers
        self.__managed = []  # TODO 2026-02-07 (Will Free): what is this?

    def is_container(self) -> bool:
        return bool(self.__container)

    def close(self) -> tuple[Group, list[Action | ComposableNode]] | list[Action | ComposableNode] | Group:
        if self.__parent is None:
            # main group
            return self.__actions

        if self.__container:
            # actions (e.g. ComposableNodes) are added by SimpleLauncher when creating the container
            return self.__parent, self.__actions

        # closing a classical group, potentially with event handling
        if self.__actions:
            group: list[Action | ComposableNode] | Action
            if any(not isinstance(action, Action) for action in self.__actions):
                # probably a raw function call due to OnProcessIO
                # skip GroupAction as it makes it not callable
                group = self.__actions
            else:
                ns_tree = list(map(PushRosNamespace, self.__ns))
                group = GroupAction(typing.cast(list[Action], ns_tree + self.__actions), condition=self.__condition)
            if self.__when is not None:
                self.__root.add_action(self.__when.register(group))
            else:
                self.__root.add_action(group)

        return self.__parent

    def add_action(self, action: Action | ComposableNode | list[Action | ComposableNode]):
        if isinstance(action, list):
            self.__actions.extend(action)  # ty:ignore[invalid-argument-type]
        else:
            self.__actions.append(action)
