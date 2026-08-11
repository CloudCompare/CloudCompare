import math

import pytest


def reference_softmax(values):
    shifted = [value - max(values) for value in values]
    exps = [math.exp(value) for value in shifted]
    total = sum(exps)
    return [value / total for value in exps]


def test_attention_equation_reference_weights_are_normalized_and_non_negative():
    local = [1.0, 0.0]
    neighborhood = [[1.0, 0.0], [0.0, 1.0], [0.5, 0.5]]
    scores = [sum(x * y for x, y in zip(local, neighbor, strict=True)) / math.sqrt(2) for neighbor in neighborhood]
    weights = reference_softmax(scores)
    assert math.isclose(sum(weights), 1.0, rel_tol=0.0, abs_tol=1e-12)
    assert all(weight >= 0.0 for weight in weights)


def test_point_attention_preserves_shape_and_normalizes_weights_when_torch_available():
    torch = pytest.importorskip("torch")
    from geodesic.point_attention import PointAttentionAssistant

    torch.manual_seed(11)
    local_states = torch.randn(2, 5, 4)
    neighborhoods = torch.randn(2, 5, 3, 4)
    output = PointAttentionAssistant()(local_states, neighborhoods)
    assert output.points.shape == local_states.shape
    assert output.attention_weights.shape == (2, 5, 3)
    assert torch.allclose(output.attention_weights.sum(dim=-1), torch.ones(2, 5), atol=1e-6)
    assert torch.all(output.attention_weights >= 0)
    assert torch.isfinite(output.points).all()


def test_point_attention_rejects_nan_inf_when_torch_available():
    torch = pytest.importorskip("torch")
    from geodesic.point_attention import PointAttentionAssistant

    local_states = torch.zeros(1, 1, 2)
    neighborhoods = torch.zeros(1, 1, 2, 2)
    neighborhoods[0, 0, 0, 0] = float("nan")
    with pytest.raises(ValueError):
        PointAttentionAssistant()(local_states, neighborhoods)
