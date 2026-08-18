# ruff: noqa: D100, D101, D102, D103, D107, CPY001

import re
import xml.etree.ElementTree as ET
from typing import override
from xml.etree.ElementTree import Element

from markdown import Markdown
from markdown.extensions import Extension
from markdown.inlinepatterns import InlineProcessor

ROS_DOCS_HOST = "docs.ros.org"

INTERFACE_TYPES = {
    "msg": "msg",
    "srv": "srv",
    "action_type": "action",
}

TYPE_CLASS = {
    "topic": "ros-topic",
    "service": "ros-service",
    "action": "ros-action",
    "node": "ros-node",
    "package": "ros-package",

    "msg": "ros-msg",
    "srv": "ros-srv",
    "action_type": "ros-action-type",
}

TYPE_COLOR_MAP = {
    "topic": "teal",
    "service": "deep-purple",
    "action": "indigo",
    "node": "indigo",
    "package": "deep-orange",

    "msg": "purple",
    "srv": "green",
    "action_type": "red",
}

INTERFACE_PATTERN = re.compile(r"^(\w+)/(msg|srv|action)/(\w+)$")


def parse_interface(value: str) -> tuple[str, str, str] | None:
    match = INTERFACE_PATTERN.fullmatch(value)

    if match is None:
        return None

    package: str = match.group(1)
    interface_kind: str = match.group(2)
    interface_name: str = match.group(3)

    return package, interface_kind, interface_name


def interface_url(
        distro: str,
        package: str,
        interface_kind: str,
        interface_name: str,
) -> str:
    return f"https://{ROS_DOCS_HOST}/en/{distro}/p/{package}/{interface_kind}/{interface_name}.html"


def package_url(distro: str, package: str) -> str:
    return f"https://{ROS_DOCS_HOST}/en/{distro}/p/{package}/"


class ROSInlineProcessor(InlineProcessor):
    def __init__(self, pattern: str, md: Markdown, distro: str):
        super().__init__(pattern, md)
        self.distro = distro

    @override
    def handleMatch(self, m: re.Match[str], data: str) -> tuple[Element, int, int]:
        reference_type = m.group(1)
        value = m.group(2)

        if reference_type in INTERFACE_TYPES:
            return self.handle_interface(m, reference_type, value)

        if reference_type == "package":
            return self.handle_package(m, value)

        return self.handle_reference(m, reference_type, value)

    def handle_interface(
            self,
            match: re.Match[str],
            reference_type: str,
            value: str,
    ) -> tuple[Element, int, int]:
        parsed = parse_interface(value)

        # A :msg:, :srv:, or :action_type: reference must be qualified.
        #
        # If it isn't, don't generate a broken link. Instead, render it
        # as ordinary styled text.
        if parsed is None:
            return self.make_code(match, value, reference_type)

        package, interface_kind, interface_name = parsed

        # Make sure the syntax agrees with the prefix.
        if interface_kind != INTERFACE_TYPES[reference_type]:
            return self.make_code(match, value, reference_type)

        url = interface_url(
            distro=self.distro,
            package=package,
            interface_kind=interface_kind,
            interface_name=interface_name,
        )

        return self.make_link(
            match=match,
            value=value,
            reference_type=reference_type,
            url=url,
        )

    def handle_package(self, match: re.Match[str], value: str) -> tuple[Element, int, int]:
        if not re.fullmatch(r"\w+", value):
            return self.make_code(match, value, "package")

        url = package_url(
            distro=self.distro,
            package=value,
        )

        return self.make_link(
            match=match,
            value=value,
            reference_type="package",
            url=url,
        )

    def handle_reference(
            self,
            match: re.Match[str],
            reference_type: str,
            value: str,
    ) -> tuple[Element, int, int]:
        if reference_type in {"topic", "service", "action"}:
            return self.make_hierarchical_reference(match, value, reference_type)

        if reference_type == "node":
            code = self.make_code(match, value, reference_type)[0]
            return self.make_reference(match, reference_type, code)

        return self.make_code(match, value, reference_type)

    @staticmethod
    def make_hierarchical_reference(match: re.Match[str], value: str, reference_type: str) -> tuple[Element, int, int]:
        element = ROSInlineProcessor.make_code(match, None, reference_type)[0]

        parts = value.split("/")

        for i, part in enumerate(parts):
            if i > 0:
                separator = ET.SubElement(element, "span")
                separator.set("class", "ros-reference-separator")
                separator.set("data-depth", str(i - 1 if value.startswith("/") else i))
                separator.text = "/"

            if not part:
                continue

            component = ET.SubElement(element, "span")
            component.set("class", "ros-reference-component")
            component.text = part

        return ROSInlineProcessor.make_reference(match, reference_type, element)

    @staticmethod
    def make_link(
            match: re.Match[str],
            value: str,
            reference_type: str,
            url: str,
    ) -> tuple[Element, int, int]:
        link = ET.Element("a")
        link.set("href", url)

        link.append(ROSInlineProcessor.make_code(match, value, reference_type)[0])

        return link, match.start(0), match.end(0)

    @staticmethod
    def make_reference(
            match: re.Match[str],
            reference_type: str,
            code: Element,
    ) -> tuple[Element, int, int]:
        wrapper = ET.Element("span")
        wrapper.set("class", f"ros-reference {TYPE_CLASS[reference_type]}")

        icon = ET.SubElement(wrapper, "span")
        icon.set("class", "ros-reference-icon")

        wrapper.append(code)

        return wrapper, match.start(0), match.end(0)

    @staticmethod
    def make_code(match: re.Match[str], value: str | None, reference_type: str) -> tuple[Element, int, int]:
        element = ET.Element("code")
        element.set("class", TYPE_CLASS[reference_type])
        element.set("data-md-color-primary", TYPE_COLOR_MAP[reference_type])
        element.set("data-md-color-accent", TYPE_COLOR_MAP[reference_type])
        element.text = value

        return element, match.start(0), match.end(0)


class ROSMarkdownExtension(Extension):
    r"""
    Markdown extension for ROS types.

    e.g.
        - \:topic\:\`/plan\`
        - \:package\:\`nav2_controller\`
    """

    def __init__(self, **kwargs):
        self.config = {
            "distro": [
                "rolling",
                "ROS 2 distribution used for external documentation links.",
            ],
        }

        super().__init__(**kwargs)

    @override
    def extendMarkdown(self, md: Markdown):
        reference_types = list(TYPE_CLASS.keys())

        types = "|".join(re.escape(reference_type) for reference_type in reference_types)

        pattern = rf":({types}):`([^`]+)`"

        processor = ROSInlineProcessor(pattern, md, distro=self.getConfig("distro"))

        md.inlinePatterns.register(processor, "ros-reference", 1000)


# ruff: ignore[N802]
def makeExtension(**kwargs) -> ROSMarkdownExtension:
    return ROSMarkdownExtension(**kwargs)
