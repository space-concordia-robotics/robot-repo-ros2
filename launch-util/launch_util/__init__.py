from typing import Any

from launch_util.events import When
from launch_util.gazebo import BridgeDirection, GazeboBridge, GazeboType, ImageBridgeQoS
from launch_util.group import Group
from launch_util.simple_launcher import SimpleLauncher
from launch_util.substitutions import (
    NormalizedSubstitutionsValueTypeDict,
    OpaqueFunctionSubstitution,
    SomeSubstitutionsValueTypeDict,
    Templated,
    WriteTempFile,
    Xacro,
    normalize_typed_dict_substitutions,
    perform_typed_dict_substitutions,
)
from launch_util.util import flatten_substitutions

__all__ = [
    "BridgeDirection",
    "GazeboBridge",
    "GazeboType",
    "Group",
    "ImageBridgeQoS",
    "NormalizedSubstitutionsValueTypeDict",
    "OpaqueFunctionSubstitution",
    "SimpleLauncher",
    "SomeSubstitutionsValueTypeDict",
    "Templated",
    "When",
    "WriteTempFile",
    "Xacro",
    "flatten_substitutions",
    "normalize_typed_dict_substitutions",
    "perform_typed_dict_substitutions",
]


def init_logging(enable: bool):
    if not enable:
        return

    import logging  # noqa: PLC0415
    from typing import Literal  # noqa: PLC0415

    import launch  # noqa: PLC0415
    from colorlog.formatter import ColoredFormatter  # noqa: PLC0415

    launch_config = launch.logging.launch_config
    formatter: logging.Formatter | None = launch_config.screen_formatter
    screen_handler: Any = launch_config.screen_handler
    fmt = formatter._fmt  # noqa: SLF001  # ty:ignore[unresolved-attribute]
    formatter_style = formatter._style  # noqa: SLF001  # ty:ignore[unresolved-attribute]

    style: Literal["%", "{", "$"] = "%"
    if isinstance(formatter_style, logging.StrFormatStyle):
        style = "{"
    elif isinstance(formatter_style, logging.StringTemplateStyle):
        style = "$"
    elif isinstance(formatter_style, logging.PercentStyle):
        style = "%"

    colored_formatter = ColoredFormatter(fmt=f"{{log_color}}{fmt}", style=style)
    launch_config.screen_formatter = colored_formatter
    screen_handler.setFormatter(colored_formatter)


init_logging(False)
