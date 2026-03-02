from logging import Logger
from tempfile import NamedTemporaryFile, _TemporaryFileWrapper
from typing import Dict, IO, Optional, Mapping
from typing import List
from xml.dom import minidom

import launch
import xacro
from launch import Action
from launch.event_handlers import OnShutdown, OnExecutionComplete
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution
from launch.substitutions import SubstitutionFailure
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions
from launch.utilities.type_utils import SomeValueType, normalize_typed_substitution, perform_typed_substitution, NormalizedValueType

SomeSubstitutionsDict = Mapping[SomeSubstitutionsType, SomeSubstitutionsType]

SomeSubstitutionsValueTypeDict = Mapping[SomeSubstitutionsType, SomeValueType]


class Xacro(Substitution):
    """
    Substitution that processes a xacro file and returns the result as a string

    :param file_path: The path to the xacro file to process
    :param mappings: A dictionary of mappings to pass to xacro

    example:

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
    __file_path: List[Substitution]
    __mappings: Dict[List[Substitution], List[Substitution]]
    __strip_comments: NormalizedValueType
    __verbose: bool
    __logger: Logger

    def __init__(
            self,
            file_path: SomeSubstitutionsType,
            mappings: Optional[SomeSubstitutionsValueTypeDict] = None,
            strip_comments: Optional[SomeValueType] = None,
            verbose: bool = False,
    ):
        """Create a Xacro."""
        super().__init__()

        if mappings is None:
            mappings = {}

        self.__file_path = normalize_to_list_of_substitutions(file_path)
        self.__mappings = {
            key: normalize_typed_substitution(value, None) for key, value in mappings.items()
        }
        self.__strip_comments = normalize_typed_substitution(strip_comments, bool) if strip_comments is not None else False
        self.__verbose = verbose
        self.__logger = launch.logging.get_logger(__name__)

    def describe(self) -> str:
        """Return a description of this substitution as a string."""
        return f"Xacro: {self.__file_path}"

    def perform(self, context: LaunchContext) -> str:
        """Perform the substitution by returning the string with values substituted."""

        file_path = perform_substitutions(context, self.__file_path)
        mappings = {
            perform_substitutions(context, normalize_to_list_of_substitutions(key)): perform_typed_substitution(context, value, None) \
            for key, value in self.__mappings.items()
        }

        # TODO 2026-02-07 (Will Free): I don't think an __verbose field is needed here at all?
        if self.__verbose:
            self.__logger.debug(f"xacro file_path: {file_path}")
            self.__logger.debug(f"xacro mappings: {mappings}")

        document = xacro.process_file(file_path, mappings=mappings)

        try:
            strip_comments = perform_typed_substitution(context, self.__strip_comments, bool)
        except (TypeError, ValueError) as e:
            raise SubstitutionFailure(e)

        if strip_comments:
            Xacro.remove_comments(document)

        document_string = document.toprettyxml(indent="  ")

        if self.__verbose:
            self.__logger.debug(f"xacro result: {document_string}")

        return document_string

    @staticmethod
    def remove_comments(parent: minidom.Node):
        node: minidom.Node
        for node in parent.childNodes:
            if isinstance(node, minidom.Comment):
                parent: minidom.Node = node.parentNode
                parent.removeChild(node)
            else:
                Xacro.remove_comments(node)


class WriteTempFile(Substitution):
    """Substitution that creates a temporary file with content and returns its path.

    This substitution is useful for creating temporary configuration files,
    parameter files, or any other content that needs to be written to disk
    for use by other processes during launch.

    Example:
        WriteTempFile("configuration content")
        WriteTempFile(["line 1\n", "line 2\n"])

    The temporary file is created with delete=False, so it persists until
    manually cleaned up or the system removes it.
    """
    __contents: List[Substitution]
    __suffix: List[Substitution]
    __prefix: List[Substitution]
    __temp_file: Optional[_TemporaryFileWrapper]

    def __init__(self, suffix: SomeSubstitutionsType = None, prefix: SomeSubstitutionsType = None, contents: SomeSubstitutionsType = None) -> None:
        """Initialize the WriteTempFile substitution.

        Args:
            contents: Content to write to the temporary file. Can be a string,
                     substitution, or list of strings/substitutions.
        """
        super().__init__()
        self.__contents = normalize_to_list_of_substitutions(contents)
        self.__suffix = normalize_to_list_of_substitutions(suffix)
        self.__prefix = normalize_to_list_of_substitutions(prefix)
        self.__temp_file = None

    def describe(self) -> str:
        """Return a description of this substitution.

        Returns:
            String description of the substitution for debugging
        """
        contents_str = " + ".join([sub.describe() for sub in self.__contents])
        return f"WriteTempFile(contents={contents_str})"

    def write(self, handle: IO, context: LaunchContext) -> None:
        """Write the contents to a file handle.

        Args:
            handle: Binary file handle to write to
            context: Launch context for performing substitutions

        Returns:
            None
        """
        contents = perform_substitutions(context, self.__contents)
        handle.write(contents.encode())

    def close_file(self):
        if self.__temp_file is not None:
            self.__temp_file.close()

    def on_shutdown(self):
        return OnShutdown(on_shutdown=lambda event, context: self.close_file())

    def on_exit(self, action: Action):
        return OnExecutionComplete(target_action=action, on_completion=lambda event, context: self.close_file())

    def perform(self, context: LaunchContext) -> str:
        """Create a temporary file with the content and return its path.

        Creates a named temporary file, writes the substituted content to it,
        and returns the file path. The file is not automatically deleted.

        Args:
            context: Launch context for performing substitutions

        Returns:
            Path to the created temporary file
        """
        suffix = perform_substitutions(context, self.__suffix)
        prefix = perform_substitutions(context, self.__prefix)

        self.__temp_file = NamedTemporaryFile(suffix=suffix, prefix=prefix)

        self.write(self.__temp_file, context)
        self.__temp_file.flush()

        return self.__temp_file.name

# TODO 2026-02-06 (Will Free): Add OpaqueFunctionSubstitution based off of the Command substitution & OpaqueFunction action?
