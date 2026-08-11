"""Distance metrics used by geodesic manifolds."""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence

Vector = Sequence[float]
DistanceFunction = Callable[[Vector, Vector], float]


class Metric:
    """A named distance function with validation helpers."""

    def __init__(self, name: str = "euclidean", distance_fn: DistanceFunction | None = None) -> None:
        self.name = name
        self._distance_fn = distance_fn or self.euclidean

    def distance(self, a: Vector, b: Vector) -> float:
        """Return the distance between vectors ``a`` and ``b``."""

        self._validate_pair(a, b)
        value = float(self._distance_fn(a, b))
        if value < 0:
            raise ValueError("metric distance cannot be negative")
        return value

    @staticmethod
    def euclidean(a: Vector, b: Vector) -> float:
        """Return the Euclidean distance between two equal-length vectors."""

        Metric._validate_pair(a, b)
        return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b, strict=True)))

    @staticmethod
    def manhattan(a: Vector, b: Vector) -> float:
        """Return the Manhattan/L1 distance between two equal-length vectors."""

        Metric._validate_pair(a, b)
        return sum(abs(x - y) for x, y in zip(a, b, strict=True))

    @staticmethod
    def _validate_pair(a: Vector, b: Vector) -> None:
        if len(a) != len(b):
            raise ValueError("vectors must have the same dimensionality")
        if len(a) == 0:
            raise ValueError("vectors must not be empty")
