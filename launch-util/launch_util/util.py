from __future__ import annotations

from collections.abc import Iterable
from typing import List, Any

from launch import Substitution
from launch.utilities import normalize_to_list_of_substitutions

__all__ = [
    "flatten_substitutions",
]


def flatten_substitutions(nested: Any) -> List[Substitution]:
    def inner_flatten(element):
        if isinstance(element, Iterable) and not isinstance(element, (str, bytes)):
            for item in element:
                yield from inner_flatten(item)
        else:
            yield from normalize_to_list_of_substitutions(element)

    return list(inner_flatten(nested))
