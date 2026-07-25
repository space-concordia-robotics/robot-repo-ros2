# ruff: noqa: D100, D103
from __future__ import annotations

from collections.abc import Generator, Iterable
from typing import Any

from launch import SomeSubstitutionsType, Substitution
from launch.utilities import normalize_to_list_of_substitutions

__all__ = [
    "flatten_substitutions",
]


def flatten_substitutions(nested: Any) -> list[Substitution]:
    def inner_flatten(element: SomeSubstitutionsType) -> Generator[Substitution, None, None]:
        if isinstance(element, Iterable) and not isinstance(element, (str, bytes)):
            item: SomeSubstitutionsType
            for item in element:  # ty:ignore[invalid-assignment]
                yield from inner_flatten(item)
        else:
            yield from normalize_to_list_of_substitutions(element)

    return list(inner_flatten(nested))
