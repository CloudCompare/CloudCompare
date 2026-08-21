import pytest

from geodesic.chess import ChessBoard
from geodesic.core.metric import Metric
from geodesic.pipeline import GeodesicPipeline


def test_metric_from_name_supports_kernel_aliases():
    assert Metric.from_name("l1").distance((1, 2), (4, -2)) == 7
    with pytest.raises(ValueError):
        Metric.from_name("hyperbolic")


def test_pipeline_kernel_path_is_available_without_transformer():
    report = GeodesicPipeline(metric="manhattan").kernel_path(
        [("a", (0.0, 0.0)), ("b", (3.0, 4.0)), ("c", (3.0, 4.0))]
    )
    assert report.path_length == 7
    assert report.metric == "manhattan"


def test_pipeline_analyzes_starting_position():
    result = GeodesicPipeline(k=3).analyze(ChessBoard())
    assert len(result.candidates) == 3
    assert all(candidate["legality"] for candidate in result.candidates)
    if result.status == "PASS":
        assert result.transformer is not None
        assert result.transformer["parameter_count"] > 0
    else:
        assert result.status == "VALIDATION_BLOCKED"
