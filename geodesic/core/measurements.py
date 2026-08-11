"""Measurement helpers for geodesic experiments."""

from __future__ import annotations

from collections.abc import Iterable

from geodesic.core.manifold import Manifold
from geodesic.core.metric import Vector


def path_length(manifold: Manifold, path: Iterable[str | Vector]) -> float:
    """Return the cumulative metric length of a path."""

    points = list(path)
    if len(points) < 2:
        return 0.0
    return sum(manifold.distance(left, right) for left, right in zip(points, points[1:], strict=False))


def distortion(manifold: Manifold, path: Iterable[str | Vector]) -> float:
    """Return path length divided by direct endpoint distance."""

    points = list(path)
    if len(points) < 2:
        return 1.0
    direct = manifold.distance(points[0], points[-1])
    if direct == 0:
        return 0.0
    return path_length(manifold, points) / direct
