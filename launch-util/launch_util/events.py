# ruff: noqa: D100, D101, D102, D107, ANN001, ANN201
from typing import Any

import launch
from launch import LaunchDescriptionEntity, SomeSubstitutionsType
from launch.actions import GroupAction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnExecutionComplete, OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown

__all__ = [
    "When",
]

logger = launch.logging.get_logger(__name__)


def wrap(action: LaunchDescriptionEntity, delay: float | SomeSubstitutionsType | None) -> list[Any] | list[TimerAction]:
    if delay is None:
        return [action]
    if isinstance(delay, int):
        delay = float(delay)
    return [TimerAction(period=delay, actions=[action])]


# TODO 2026-06-29 (Will Free): I hate this api, I really need to get rid of it
class When:
    __arg: str | None

    def __init__(
            self,
            action=None,
            event=None,
            delay=None,
            io: str | None = None,
    ):
        self.__ref = action
        self.__delay = delay
        self.__event = event

        if event is None:
            return

        if event == OnProcessExit:
            self.__arg = "on_exit"
        elif event == OnProcessStart:
            self.__arg = "on_start"
        elif event == OnExecutionComplete:
            self.__arg = "on_completion"
        elif event == OnShutdown:
            self.__ref = None
            self.__arg = "on_shutdown"
        elif event == OnProcessIO:
            if io not in ("stdin", "stdout", "stderr") or io is None:
                logger.error("Cannot guess which sub-event of OnProcessIO is to be treated, specify arg = 'stdin', 'stdout' or 'stderr'")
                self.__arg = None
            else:
                self.__arg = "on_" + io
        else:
            self.__arg = None

    def register(self, action):
        if self.__event == OnProcessIO:
            # this one expects Python functions that returns actions
            # noinspection PyTypeChecker
            kwargs: dict[str, Any] = {self.__arg: None}  # ty:ignore[invalid-assignment]
            if len(action) == 0:
                logger.warning("No callback in OnProcessIO block")
            else:
                def callback(event: object) -> GroupAction:
                    if self.__delay is not None:
                        import time  # noqa: PLC0415
                        time.sleep(self.__delay)
                    return GroupAction([cb(event) for cb in action])

                # noinspection PyTypeChecker
                kwargs[self.__arg] = callback  # ty:ignore[invalid-assignment]
            return RegisterEventHandler(OnProcessIO(**kwargs))

        if self.__event is None:
            return wrap(action, self.__delay)

        if self.__arg is None:
            name = f"{self.__event}".split()[1][1:-2]
            logger.error("sorry I cannot handle events of type %s", name)

        wrapped = wrap(action, self.__delay)

        # noinspection PyTypeChecker
        kwargs: dict[str, Any] = {self.__arg: wrapped}  # ty:ignore[invalid-assignment]
        if self.__ref is not None:
            # noinspection PyTypeChecker
            kwargs["target_action"] = self.__ref

        return RegisterEventHandler(self.__event(**kwargs))
