# ruff: noqa: CPY001, D100

from typing import Literal, override

from mkdocs.config import Config
from mkdocs.plugins import BasePlugin
from pygments.lexers.shell import BashLexer
from pygments.token import Name


class BashExtensionConfig(Config):
    pass


class ExtendBashPlugin(BasePlugin[BashExtensionConfig]):
    @override
    def on_startup(self, *, command: Literal["build", "gh-deploy", "serve"], dirty: bool):
        root_ = BashLexer.tokens["basic"]
        root_.append(
            (
                r"^sudo",
                Name.Builtin,
            ),  # ty: ignore[invalid-argument-type]
        )
        root_.append(
            (
                (r"\b(sudo|apt(-(get|fast|mark)|titude)?|systemctl|journalctl|"
                 r"tee|chmod|cat|rm|sed|grep|awk|bash)(?=[\s)`$])"),
                Name.Builtin,
            ),  # ty: ignore[invalid-argument-type]
        )
