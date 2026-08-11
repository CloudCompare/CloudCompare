"""A small in-memory manifold for geodesic experiments."""

from __future__ import annotations

from dataclasses import dataclass, field

from geodesic.core.anchor import Anchor
from geodesic.core.metric import Metric, Vector


@dataclass
class Manifold:
    """Collection of anchors equipped with a distance metric."""

    dimensions: int
    metric: Metric = field(default_factory=Metric)
    anchors: dict[str, Anchor] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.dimensions <= 0:
            raise ValueError("manifold dimensions must be positive")

    def add_anchor(self, anchor: Anchor) -> Anchor:
        """Add an anchor after validating its dimensionality."""

        self._validate_vector(anchor.coordinates)
        self.anchors[anchor.name] = anchor
        return anchor

    def add_point(self, name: str, coordinates: Vector, **metadata: object) -> Anchor:
        """Create and add an anchor from raw coordinates."""

        return self.add_anchor(Anchor(name=name, coordinates=tuple(coordinates), metadata=metadata))

    def distance(self, left: str | Vector, right: str | Vector) -> float:
        """Measure distance between anchor names or raw vectors."""

        return self.metric.distance(self._resolve(left), self._resolve(right))

    def nearest_anchor(self, coordinates: Vector) -> Anchor:
        """Return the nearest anchor to ``coordinates``."""

        self._validate_vector(coordinates)
        if not self.anchors:
            raise ValueError("cannot query an empty manifold")
        return min(self.anchors.values(), key=lambda anchor: self.metric.distance(anchor.coordinates, coordinates))

    def _resolve(self, point: str | Vector) -> Vector:
        if isinstance(point, str):
            try:
                return self.anchors[point].coordinates
            except KeyError as exc:
                raise KeyError(f"unknown anchor: {point}") from exc
        self._validate_vector(point)
        return point

    def _validate_vector(self, coordinates: Vector) -> None:
        if len(coordinates) != self.dimensions:
            raise ValueError(f"expected {self.dimensions} dimensions, received {len(coordinates)}")
