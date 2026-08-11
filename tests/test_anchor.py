import pytest

from geodesic.core.anchor import Anchor


def test_anchor_normalizes_coordinates_and_freezes_metadata():
    anchor = Anchor("a", (1, 2), {"kind": "seed"})
    assert anchor.coordinates == (1.0, 2.0)
    assert anchor.metadata["kind"] == "seed"
    with pytest.raises(TypeError):
        anchor.metadata["kind"] = "changed"


def test_anchor_requires_name_and_coordinates():
    with pytest.raises(ValueError):
        Anchor("", (1,))
    with pytest.raises(ValueError):
        Anchor("empty", ())
