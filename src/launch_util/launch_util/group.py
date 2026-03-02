from typing import Optional, List, Text, Union, Tuple

from launch import Action, Condition, SomeSubstitutionsType
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

from .events import When


class Group:
    __parent: Optional['Group']
    __condition: Optional[Condition]
    __container: Text
    __when: Optional[When]
    __root: 'Group'
    __ns: List[SomeSubstitutionsType]
    __actions: List[Action]

    def __init__(
            self,
            ns: SomeSubstitutionsType = None,
            condition: Optional[Condition] = None,
            container: Text = '',
            parent: Optional['Group'] = None,
            when: When = None
    ):
        self.__parent = parent
        self.__condition = condition
        self.__container = container
        self.__when = when

        # inherit namespace from this branch
        ns = [ns] if ns is not None else []
        if parent is None:
            self.__root = self
            self.__ns = ns
        else:
            self.__root = parent.__root
            self.__ns = parent.__ns + ns

        # actions are always active and added to the root group
        self.__actions = []
        # managed are added as Event or Containers
        self.__managed = []  # TODO 2026-02-07 (Will Free): what is this?

    def is_container(self):
        return self.__container

    def close(self) -> Union[Tuple['Group', List[Action]], List[Action], 'Group']:
        if self.__parent is None:
            # main group
            return self.__actions

        if self.__container:
            # actions (e.g. ComposableNodes) are added by SimpleLauncher when creating the container
            return self.__parent, self.__actions

        # closing a classical group, potentially with event handling
        if self.__actions:

            if any(not isinstance(action, Action) for action in self.__actions):
                # probably a raw function call due to OnProcessIO
                # skip GroupAction as it makes it not callable
                group = self.__actions
            else:
                ns_tree = list(map(PushRosNamespace, self.__ns))
                group = GroupAction(ns_tree + self.__actions, condition=self.__condition)
            if self.__when is not None:
                self.__root.add_action(self.__when.register(group))
            else:
                self.__root.add_action(group)

        return self.__parent

    def add_action(self, action: Action):
        if isinstance(action, list):
            self.__actions.extend(action)
        else:
            self.__actions.append(action)
