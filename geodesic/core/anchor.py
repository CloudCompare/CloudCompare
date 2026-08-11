"""Anchor points that label stable positions on a manifold."""

from __future__ import annotations

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Mapping


@dataclass(frozen=True)
class Anchor:
    """A named point with optional immutable metadata."""

    name: str
    coordinates: tuple[float, ...]
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("anchor name must not be empty")
        if not self.coordinates:
            raise ValueError("anchor coordinates must not be empty")
        object.__setattr__(self, "coordinates", tuple(float(value) for value in self.coordinates))
        object.__setattr__(self, "metadata", MappingProxyType(dict(self.metadata)))
