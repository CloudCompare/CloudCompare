"""Distance metrics used by geodesic manifolds."""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence

Vector = Sequence[float]
DistanceFunction = Callable[[Vector, Vector], float]

_P_NORMS = {
    "euclidean": 2.0,
    "l2": 2.0,
    "manhattan": 1.0,
    "l1": 1.0,
}


def named_p_norm(name: str) -> float:
    """Return the ``torch.cdist`` p-norm that matches a named kernel metric."""

    key = str(name).strip().lower()
    try:
        return _P_NORMS[key]
    except KeyError as exc:
        supported = ", ".join(sorted(_P_NORMS))
        raise ValueError(f"unsupported geodesic metric '{name}'; expected one of: {supported}") from exc


class Metric:
    """A named distance function with validation helpers."""

    def __init__(self, name: str = "euclidean", distance_fn: DistanceFunction | None = None) -> None:
        self.name = name
        self._distance_fn = distance_fn or self.euclidean

    @classmethod
    def from_name(cls, name: str) -> "Metric":
        """Build a kernel metric from a documented name."""

        key = str(name).strip().lower()
        if key in {"euclidean", "l2"}:
            return cls("euclidean", cls.euclidean)
        if key in {"manhattan", "l1"}:
            return cls("manhattan", cls.manhattan)
        raise ValueError(f"unsupported geodesic metric: {name}")
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
