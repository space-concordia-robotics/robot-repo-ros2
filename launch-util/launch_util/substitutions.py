"""A collection of useful substitutions."""

import typing
from collections.abc import Callable, Iterable, Mapping
from logging import Logger
from string import Template
from tempfile import NamedTemporaryFile
from typing import IO, Any, override
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
    """
    Normalizes a dictionary of substitutions.

    **See also:** :func:`launch.utilities.type_utils.normalize_typed_substitution`

    :param items: The dictionary of substitutions.
    :param data_type: An optional data type the substitutions can be checked against.
    :return: The normalized substitution dictionary.
    """

    return {
        tuple(normalize_to_list_of_substitutions(key)): normalize_typed_substitution(value, data_type) for key, value in items.items()
    }


def perform_typed_dict_substitutions(
        context: LaunchContext,
        items: NormalizedSubstitutionsValueTypeDict,
        data_type: AllowedTypesType | None,
) -> dict[str, StrSomeValueType]:
    """
    Performs substitutions on a dictionary of substitutions.

    :param context: The context to use for performing the substitutions.
    :param items: The dictionary of substitutions.
    :param data_type: An optional data type used to coerce the substitutions.
    :return: The dictionary of resolved substitutions.
    """

    return {
        perform_substitutions(context, list(key)): perform_typed_substitution(context, value, data_type) for key, value in items.items()
    }


class Xacro(Substitution):
    """
    Substitution that processes a xacro file and returns the result as a string.

    :param file_path: The path to the xacro file to process
    :param mappings: A dictionary of mappings to pass to xacro

    .. code:: python

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
        """Create a Xacro substitution."""

        super().__init__()

        self.__file_path = normalize_to_list_of_substitutions(file_path)
        self.__mappings = normalize_typed_dict_substitutions(mappings, None) if mappings is not None else {}
        self.__strip_comments = normalize_typed_substitution(strip_comments, bool) if strip_comments is not None else False
        self.__verbose = verbose
        self.__logger = launch.logging.get_logger(__name__)

    @override
    def describe(self) -> str:
        """:return: a description of this substitution as a string."""

        return f"Xacro: {self.__file_path}"

    @override
    def perform(self, context: LaunchContext) -> str:
        """
        Perform the substitution by returning the string with values substituted.

        :return: The stringified XML file after executing xacro.
        """

        file_path = perform_substitutions(context, self.__file_path)
        mappings = perform_typed_dict_substitutions(context, self.__mappings, None)

        # TODO 2026-02-07 (Will Free): I don't think an __verbose field is needed here at all?
        if self.__verbose:
            self.__logger.debug("xacro file_path: %s", file_path)
            self.__logger.debug("xacro mappings: %s", mappings)

        document: minidom.Document = typing.cast(minidom.Document, xacro.process_file(file_path, mappings=mappings))  # type: ignore[unnecessary-cast]

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
        """
        Removes the comments from a minidom XML node.

        :param parent: The node to remove comments from.
        """

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

    .. code:: python

        WriteTempFile("configuration content")
        WriteTempFile(["line 1\n", "line 2\n"])

    The temporary file is created with delete=False, so it persists until
    manually cleaned up or the system removes it.
    """

    __contents: list[Substitution]
    __suffix: list[Substitution]
    __prefix: list[Substitution]
    __temp_file: IO[bytes] | None

    def __init__(
            self,
            suffix: SomeSubstitutionsType | None = None,
            prefix: SomeSubstitutionsType | None = None,
            contents: SomeSubstitutionsType | None = None,
    ) -> None:
        """
        Create a WriteTempFile substitution.

        :param suffix: Temporary file suffix.
        :param prefix: Temporary file prefix.
        :param contents: Content to write to the temporary file.
        """

        super().__init__()
        self.__contents = normalize_to_list_of_substitutions(contents if contents is not None else [])
        self.__suffix = normalize_to_list_of_substitutions(suffix if suffix is not None else [])
        self.__prefix = normalize_to_list_of_substitutions(prefix if prefix is not None else [])
        self.__temp_file = None

    @override
    def describe(self) -> str:
        """
        Return a description of this substitution.

        :return String description of the substitution for debugging
        """

        contents_str = " + ".join([sub.describe() for sub in self.__contents])
        return f"WriteTempFile(contents={contents_str})"

    def write(self, handle: IO[bytes], context: LaunchContext):
        """
        Write the contents to a file handle.

        :param handle: Binary file handle to write to
        :param context: Launch context for performing substitutions
        """

        contents = perform_substitutions(context, self.__contents)
        handle.write(contents.encode())

    def close_file(self):
        """Closes the temporary file."""

        if self.__temp_file is not None:
            self.__temp_file.close()

    def on_shutdown(self) -> OnShutdown:
        """
        Creates a shutdown hook that closes & deletes the temporary file.

        :return: The shutdown hook.
        """

        return OnShutdown(on_shutdown=lambda _event, _context: self.close_file())

    def on_exit(self, action: Action) -> OnExecutionComplete:
        """
        Creates an exit hook which closes & deletes the temporary file.

        :param action: The target action for the hook.
        :return: The on exit hook.
        """

        return OnExecutionComplete(target_action=action, on_completion=lambda _event, _context: self.close_file())

    @override
    def perform(self, context: LaunchContext) -> str:
        """
        Create a temporary file with the content and return its path.

        Creates a named temporary file, writes the substituted content to it,
        and returns the file path. The file is not automatically deleted.

        :param context: Launch context for performing substitutions.

        :return: Path to the created temporary file.
        """

        suffix = perform_substitutions(context, self.__suffix)
        prefix = perform_substitutions(context, self.__prefix)

        self.__temp_file = NamedTemporaryFile(suffix=suffix, prefix=prefix)  # noqa: SIM115

        assert self.__temp_file is not None
        self.write(self.__temp_file, context)
        self.__temp_file.flush()

        return self.__temp_file.name


class OpaqueFunctionSubstitution(Substitution):
    """A substitution that evaluates an opaque function."""

    OpaqueSubstitution = Callable[[LaunchContext, Iterable[Any] | None, dict[str, Any] | None], str]

    def __init__(
            self,
            function: OpaqueSubstitution,
            args: Iterable[Any] | None = None,
            kwargs: dict[str, Any] | None = None,
    ) -> None:
        """
        Creates an opaque function substitution.

        :param function: The function to be executed.
        :param args: The function arguments
        :param kwargs: Additional function arguments.
        """
        super().__init__()

        self.__function = function
        self.__args = args if args is not None else []
        self.__kwargs = kwargs if kwargs is not None else {}

    @override
    def perform(self, context: LaunchContext) -> str:
        return self.__function(context, *self.__args, **self.__kwargs)


class Templated(Substitution):
    """
    Template-based substitution using Python's string.Template syntax.

    This substitution class allows for template strings with variables that
    are replaced with launch configuration values. Template variables are
    specified using ${variable_name} syntax.

    .. code:: python

        Templated("Hello ${name}, mode is ${mode}")

    The variables 'name' and 'mode' will be replaced with their corresponding
    launch configuration values.

    """

    __template: list[Substitution]
    __args: NormalizedSubstitutionsValueTypeDict

    def __init__(self, template: SomeSubstitutionsType, args: SomeSubstitutionsValueTypeDict | None = None) -> None:
        """
        Create a Templated substitution.

        :param template: Template string containing variable placeholders.
        :param args: Optional arguments.
        """

        super().__init__()

        self.__template = normalize_to_list_of_substitutions(template)
        self.__args = normalize_typed_dict_substitutions(args, None) if args is not None else {}

    @property
    def raw_template(self) -> list[Substitution]:
        """
        Get the raw template as a list of substitutions.

        :return: List of substitution objects representing the template.
        """

        return self.__template

    @property
    def args(self) -> NormalizedSubstitutionsValueTypeDict:
        """The arguments for the template."""

        return self.__args

    @override
    def describe(self) -> str:
        """
        Return a description of this substitution as a string.

        :return: String representation of the template for debugging purposes
        """

        return f"'{self.raw_template}'"

    @override
    def perform(self, context: LaunchContext) -> str:
        """
        Perform the substitution by replacing template variables.

        Uses Python's :class:`string.Template` to substitute variables with their
        corresponding values from the launch context configuration.

        :param context: Launch context containing configuration values.
        :return: String with template variables replaced by their values.
        :raise KeyError: If a template variable is not found in launch configurations.
        """

        args = perform_typed_dict_substitutions(context, self.args, None) | context.launch_configurations

        raw_template = perform_substitutions(context, self.raw_template)
        template = Template(raw_template)
        return template.substitute(args)
