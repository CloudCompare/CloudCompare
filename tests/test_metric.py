import pytest

from geodesic.core.metric import Metric


def test_euclidean_distance():
    assert Metric().distance((0, 0), (3, 4)) == 5


def test_manhattan_distance():
    metric = Metric("manhattan", Metric.manhattan)
    assert metric.distance((1, 2), (4, -2)) == 7


def test_metric_rejects_mismatched_dimensions():
    with pytest.raises(ValueError):
        Metric().distance((1,), (1, 2))
