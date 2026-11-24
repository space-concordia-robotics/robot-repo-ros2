"""Miscellaneous utilities."""

from __future__ import annotations

from collections.abc import Generator, Iterable

from launch import SomeSubstitutionsType, Substitution
from launch.utilities import normalize_to_list_of_substitutions

__all__ = [
    "flatten_substitutions",
]


def flatten_substitutions(nested: SomeSubstitutionsType | Iterable[SomeSubstitutionsType]) -> list[Substitution]:
    """
    Flattens a nested substitution.

    :param nested:
    :return:
    """

    def inner_flatten(element: SomeSubstitutionsType | Iterable[SomeSubstitutionsType]) -> Generator[Substitution, None, None]:
        if isinstance(element, Iterable) and not isinstance(element, (str, bytes)):
            for item in element:
                yield from inner_flatten(item)
        else:
            assert not isinstance(element, Iterable)

            yield from normalize_to_list_of_substitutions(element)

    return list(inner_flatten(nested))
