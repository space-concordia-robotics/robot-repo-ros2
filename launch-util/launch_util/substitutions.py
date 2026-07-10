# ruff: noqa: D100, D101, D102, D103, D107
import typing
from collections.abc import Callable, Iterable, Mapping
from logging import Logger
from string import Template
from tempfile import NamedTemporaryFile
from typing import IO, Any
from xml.dom import minidom

import launch
import xacro
from launch import Action, Substitution
from launch.event_handlers import OnExecutionComplete, OnShutdown
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import SubstitutionFailure
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch.utilities.type_utils import (
    AllowedTypesType,
    NormalizedValueType,
    SomeValueType,
    StrSomeValueType,
    normalize_typed_substitution,
    perform_typed_substitution,
)

__all__ = [
    "NormalizedSubstitutionsValueTypeDict",
    "OpaqueFunctionSubstitution",
    "SomeSubstitutionsValueTypeDict",
    "Templated",
    "WriteTempFile",
    "Xacro",
    "normalize_typed_dict_substitutions",
    "perform_typed_dict_substitutions",
]

SomeSubstitutionsValueTypeDict = Mapping[SomeSubstitutionsType, SomeValueType]

NormalizedSubstitutionsValueTypeDict = Mapping[tuple[Substitution, ...], NormalizedValueType]


def normalize_typed_dict_substitutions(
        items: SomeSubstitutionsValueTypeDict,
        data_type: AllowedTypesType | None,
) -> NormalizedSubstitutionsValueTypeDict:
    return {
        tuple(normalize_to_list_of_substitutions(key)): normalize_typed_substitution(value, data_type) for key, value in items.items()
    }


def perform_typed_dict_substitutions(
        context: LaunchContext,
        items: NormalizedSubstitutionsValueTypeDict,
        data_type: AllowedTypesType | None,
) -> dict[str, StrSomeValueType]:
    return {
        perform_substitutions(context, list(key)): perform_typed_substitution(context, value, data_type) for key, value in items.items()
    }


class Xacro(Substitution):
    """
    Substitution that processes a xacro file and returns the result as a string.

    :param file_path: The path to the xacro file to process
    :param mappings: A dictionary of mappings to pass to xacro

    Example:
      state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": Xacro(
                    file_path=JoinLaunchSubstitutions(
                        [
                            FindPackageShare("my_robot_description"),
                            "urdf",
                            "my_robot.urdf.xacro",
                        ]
                    ),
                    mappings={
                        "thing_to_substitute": "substitution_value",
                        "another_thing_to_substitute": LaunchConfiguration("another_thing_to_substitute"),
                    },
                )
            }
        ],
    )
    """

    __file_path: list[Substitution]
    __mappings: NormalizedSubstitutionsValueTypeDict
    __strip_comments: NormalizedValueType
    __verbose: bool
    __logger: Logger

    def __init__(
            self,
            file_path: SomeSubstitutionsType,
            mappings: SomeSubstitutionsValueTypeDict | None = None,
            strip_comments: SomeValueType | None = None,
            verbose: bool = False,
    ):
        """Create a Xacro."""
        super().__init__()

        self.__file_path = normalize_to_list_of_substitutions(file_path)
        self.__mappings = normalize_typed_dict_substitutions(mappings, None) if mappings is not None else {}
        self.__strip_comments = normalize_typed_substitution(strip_comments, bool) if strip_comments is not None else False
        self.__verbose = verbose
        self.__logger = launch.logging.get_logger(__name__)

    def describe(self) -> str:
        """Return a description of this substitution as a string."""
        return f"Xacro: {self.__file_path}"

    def perform(self, context: LaunchContext) -> str:
        """Perform the substitution by returning the string with values substituted."""
        file_path = perform_substitutions(context, self.__file_path)
        mappings = perform_typed_dict_substitutions(context, self.__mappings, None)

        # TODO 2026-02-07 (Will Free): I don't think an __verbose field is needed here at all?
        if self.__verbose:
            self.__logger.debug("xacro file_path: %s", file_path)
            self.__logger.debug("xacro mappings: %s", mappings)

        document = xacro.process_file(file_path, mappings=mappings)

        try:
            strip_comments = perform_typed_substitution(context, self.__strip_comments, bool)
        except (TypeError, ValueError) as e:
            raise SubstitutionFailure("Could not perform substitution") from e

        if strip_comments:
            Xacro.remove_comments(document)

        document_string = document.toprettyxml(indent="  ")

        if self.__verbose:
            self.__logger.debug("xacro result: %s", document_string)

        return document_string

    @staticmethod
    def remove_comments(parent: minidom.Node):
        node: minidom.Node
        for node in parent.childNodes:
            if isinstance(node, minidom.Comment):
                parent = typing.cast(minidom.Element, node.parentNode)
                parent.removeChild(node)
            else:
                Xacro.remove_comments(node)


class WriteTempFile(Substitution):
    r"""
    Substitution that creates a temporary file with content and returns its path.

    This substitution is useful for creating temporary configuration files,
    parameter files, or any other content that needs to be written to disk
    for use by other processes during launch.

    Example:
        WriteTempFile("configuration content")
        WriteTempFile(["line 1\n", "line 2\n"])

    The temporary file is created with delete=False, so it persists until
    manually cleaned up or the system removes it.
    """

    __contents: list[Substitution]
    __suffix: list[Substitution]
    __prefix: list[Substitution]
    __temp_file: IO[bytes | str] | None

    def __init__(
            self,
            suffix: SomeSubstitutionsType | None = None,
            prefix: SomeSubstitutionsType | None = None,
            contents: SomeSubstitutionsType | None = None,
    ) -> None:
        """
        Initialize the WriteTempFile substitution.

        Arguments:
        ---------
        suffix: temporary file suffix
        prefix: temporary file prefix
        contents: Content to write to the temporary file. Can be a string,
                  substitution, or list of strings/substitutions.
        """
        super().__init__()
        self.__contents = normalize_to_list_of_substitutions(contents if contents is not None else [])
        self.__suffix = normalize_to_list_of_substitutions(suffix if suffix is not None else [])
        self.__prefix = normalize_to_list_of_substitutions(prefix if prefix is not None else [])
        self.__temp_file = None

    def describe(self) -> str:
        """
        Return a description of this substitution.

        :return String description of the substitution for debugging
        """
        contents_str = " + ".join([sub.describe() for sub in self.__contents])
        return f"WriteTempFile(contents={contents_str})"

    def write(self, handle: IO, context: LaunchContext) -> None:
        """
        Write the contents to a file handle.

        :param handle: Binary file handle to write to
        :param context: Launch context for performing substitutions

        :return: None
        """
        contents = perform_substitutions(context, self.__contents)
        handle.write(contents.encode())

    def close_file(self):
        if self.__temp_file is not None:
            self.__temp_file.close()

    def on_shutdown(self) -> OnShutdown:
        return OnShutdown(on_shutdown=lambda _event, _context: self.close_file())

    def on_exit(self, action: Action) -> OnExecutionComplete:
        return OnExecutionComplete(target_action=action, on_completion=lambda _event, _context: self.close_file())

    def perform(self, context: LaunchContext) -> str:
        """
        Create a temporary file with the content and return its path.

        Creates a named temporary file, writes the substituted content to it,
        and returns the file path. The file is not automatically deleted.

        :param context: Launch context for performing substitutions

        :return: Path to the created temporary file
        """
        suffix = perform_substitutions(context, self.__suffix)
        prefix = perform_substitutions(context, self.__prefix)

        self.__temp_file = NamedTemporaryFile(suffix=suffix, prefix=prefix)  # noqa: SIM115  # ty:ignore[invalid-assignment]

        assert self.__temp_file is not None
        self.write(self.__temp_file, context)
        self.__temp_file.flush()

        return self.__temp_file.name


class OpaqueFunctionSubstitution(Substitution):
    OpaqueSubstitution = Callable[[LaunchContext, Iterable[Any] | None, dict[str, Any] | None], str]

    def __init__(
            self,
            function: OpaqueSubstitution,
            args: Iterable[Any] | None = None,
            kwargs: dict[str, Any] | None = None,
    ) -> None:
        super().__init__()

        self.__function = function
        self.__args = args if args is not None else []
        self.__kwargs = kwargs if kwargs is not None else {}

    def perform(self, context: LaunchContext) -> str:
        return self.__function(context, *self.__args, **self.__kwargs)


class Templated(Substitution):
    """
    Template-based substitution using Python's string.Template syntax.

    This substitution class allows for template strings with variables that
    are replaced with launch configuration values. Template variables are
    specified using ${variable_name} syntax.

    Example:
        Templated("Hello ${name}, mode is ${mode}")

    The variables 'name' and 'mode' will be replaced with their corresponding
    launch configuration values.

    """

    __template: list[Substitution]
    __args: NormalizedSubstitutionsValueTypeDict

    def __init__(self, template: SomeSubstitutionsType, args: SomeSubstitutionsValueTypeDict | None = None) -> None:
        """
        Initialize the Templated substitution.

        :param template: Template string containing variable placeholders
        :param args: Optional arguments
        """
        super().__init__()

        self.__template = normalize_to_list_of_substitutions(template)
        self.__args = normalize_typed_dict_substitutions(args, None) if args is not None else {}

    @property
    def raw_template(self) -> list[Substitution]:
        """
        Get the raw template as a list of substitutions.

        :return: List of substitution objects representing the template
        """
        return self.__template

    @property
    def args(self) -> NormalizedSubstitutionsValueTypeDict:
        return self.__args

    @property
    def describe(self) -> str:
        """
        Return a description of this substitution as a string.

        :return: String representation of the template for debugging purposes
        """
        return f"'{self.raw_template}'"

    def perform(self, context: LaunchContext) -> str:
        """
        Perform the substitution by replacing template variables.

        Uses Python's string.Template to substitute variables with their
        corresponding values from the launch context configuration.

        :param context: Launch context containing configuration values

        :return: String with template variables replaced by their values

        :raise KeyError: If a template variable is not found in launch configurations
        """
        args = perform_typed_dict_substitutions(context, self.args, None) | context.launch_configurations

        raw_template = perform_substitutions(context, self.raw_template)
        template = Template(raw_template)
        return template.substitute(args)
