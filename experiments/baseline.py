"""Baseline experiment that measures direct Euclidean distances."""

from geodesic.core.manifold import Manifold


def run() -> float:
    manifold = Manifold(dimensions=2)
    manifold.add_point("origin", (0, 0))
    manifold.add_point("unit", (1, 1))
    return manifold.distance("origin", "unit")


if __name__ == "__main__":
    print(run())
