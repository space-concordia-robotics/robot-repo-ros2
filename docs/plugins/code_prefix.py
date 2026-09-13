# ruff: noqa: ARG001

"""
code prefix superfences formatter.

A custom formatter for superfences that adds a prefix to each line in the margins.
"""

import logging
from typing import Any
from xml.etree import ElementTree as ET

from bs4 import BeautifulSoup, Tag
from markdown import Markdown
from pymdownx.superfences import SuperFencesBlockPreprocessor, SuperFencesCodeExtension, SuperFencesException

log = logging.getLogger("mkdocs")

SUPPORTED_PREFIXES = {
    "prefix": "*",
    "venv": "(venv)$",
    "dollar": "$",
    "hash": "#",
}


# noinspection unused-parameter
def validator(language: str, inputs: dict[str, str], options: dict[str, Any], attrs: dict[str, Any], md: Markdown) -> bool:
    """Validate a prefix fence from the list of supported prefixes."""

    options["prefix"] = ""
    for k, v in inputs.items():
        # add all inputs to options
        if k in SUPPORTED_PREFIXES:
            if k == "prefix":
                # use custom value if custom prefix
                options[k] = v
            else:
                # use pre-defined supported value for key
                options["prefix"] = SUPPORTED_PREFIXES[k]

            if "lines" not in options:
                options["lines"] = 0  # default to use prefix on all lines if set
        else:
            options[k] = v

    return True


# noinspection unused-parameter
# ruff: ignore[C901, PLR0912]
def formatter(
        source: str,
        language: str,
        class_name: str,
        options: dict[str, Any],
        md: Markdown,
        classes: list[str] | None,
        id_value: str,
        attrs: dict[str, Any],
        **kwargs,
) -> str | ET.Element:
    """Format a given code block with a given prefix if available."""

    prefix = ""
    lines = "0"
    num_lines = len(source.splitlines())
    classes = classes or []
    if "classes" in kwargs:
        classes += kwargs["classes"]
    if class_name != "prefix":
        classes.append(class_name)

    keep_hl = False

    if options["prefix"]:
        prefix = options["prefix"]
        # class_name = 'highlight'  # use the prefix as the class name

        lines = f"1-{num_lines}" if "lines" not in options or options["lines"] == 0 else options["lines"]

        # handle the case where prefix and hl_lines are specified
        keep_hl = False
        if "hl_lines" in options:
            keep_hl = True
        if "hl_lines" not in classes:
            classes.append("hl_lines")
        # set hl_lines to lines to reuse hl_lines formatter logic
        options["hl_lines"] = lines
        # use linenum preprocessor to replace them later with prefix
        if "linenums" not in options:
            options["linenums"] = "1"

    try:
        sf_ext = SuperFencesCodeExtension()
        preprocessor = SuperFencesBlockPreprocessor(md)
        # noinspection unresolved-references
        preprocessor.config = sf_ext.getConfigs()  # ty:ignore[unresolved-attribute]
        # noinspection unresolved-references
        preprocessor.extension = sf_ext  # ty:ignore[unresolved-attribute]
        preprocessor.get_hl_settings()
        preprocessor.line_count = num_lines
        lines_int = preprocessor.parse_hl_lines(lines)
        soup = BeautifulSoup(
            preprocessor.highlight(
                src=source,
                language=language,
                options=options,
                md=md,
                classes=classes,
                id_value="",
                attrs=None,
            ),
            features="lxml",
        )

        if prefix:
            line_nums_col: Tag | None = soup.find("div", {"class": "linenodiv"})
            if line_nums_col is None:
                raise Exception("html did not have div with linenodiv class")  # noqa: TRY002, TRY301
            for index, row in enumerate(line_nums_col.find_all(class_="normal")):
                if (index + 1) in lines_int:  # index starts at 0, lines expected to start at 1
                    row.string = prefix
                else:
                    row.string = ""
                if not keep_hl:
                    # remove highlight if hl_lines was not specified in markdown
                    hl_div = soup.find("span", {"class": "hll"})
                    if hl_div is not None:
                        hl_div.attrs["class"] = ""
    except Exception as err:
        log.exception("Caught exception while trying to prefix lines:", exc_info=err)
        raise SuperFencesException from err
    return soup.prettify()
