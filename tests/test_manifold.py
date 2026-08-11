import pytest

from geodesic.core.manifold import Manifold


def test_manifold_adds_and_measures_anchors():
    manifold = Manifold(dimensions=2)
    manifold.add_point("a", (0, 0))
    manifold.add_point("b", (0, 2))
    assert manifold.distance("a", "b") == 2


def test_manifold_finds_nearest_anchor():
    manifold = Manifold(dimensions=2)
    manifold.add_point("near", (1, 1))
    manifold.add_point("far", (10, 10))
    assert manifold.nearest_anchor((1.2, 1.1)).name == "near"


def test_manifold_rejects_wrong_dimensions():
    manifold = Manifold(dimensions=2)
    with pytest.raises(ValueError):
        manifold.add_point("bad", (1, 2, 3))
