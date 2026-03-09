from .events import When
from .gazebo import GazeboType, BridgeDirection, GazeboBridge
from .group import Group
from .simple_launcher import SimpleLauncher
from .substitutions import normalize_typed_dict_substitutions, perform_typed_dict_substitutions, SomeSubstitutionsValueTypeDict, \
    NormalizedSubstitutionsValueTypeDict, Xacro, WriteTempFile, OpaqueFunctionSubstitution, Templated
from .util import flatten_substitutions

__all__ = [
    "When",
    "GazeboType",
    "BridgeDirection",
    "GazeboBridge",
    "Group",
    "SimpleLauncher",
    "SomeSubstitutionsValueTypeDict",
    "NormalizedSubstitutionsValueTypeDict",
    "normalize_typed_dict_substitutions",
    "perform_typed_dict_substitutions",
    "Xacro",
    "WriteTempFile",
    "OpaqueFunctionSubstitution",
    "Templated",
    "flatten_substitutions",
]


def init_logging(enable: bool):
    if not enable:
        return

    import launch
    import logging
    from typing import Literal
    from colorlog.formatter import ColoredFormatter

    launch_config = launch.logging.launch_config
    formatter: logging.Formatter = launch_config.screen_formatter
    screen_handler = launch_config.screen_handler
    fmt = formatter._fmt
    formatter_style = formatter._style

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
