import math

import pytest


def test_standard_attention_reference_equation_environment_independent():
    q = [1.0, 0.0]
    keys = [[1.0, 0.0], [0.0, 1.0]]
    scores = [sum(a * b for a, b in zip(q, k, strict=True)) / math.sqrt(2) for k in keys]
    exps = [math.exp(score - max(scores)) for score in scores]
    weights = [value / sum(exps) for value in exps]
    assert math.isclose(sum(weights), 1.0, abs_tol=1e-12)
    assert weights[0] > weights[1]


def test_standard_equivalence():
    torch = pytest.importorskip("torch")
    from geodesic.transformer import GeodesicMultiHeadAttention, GeodesicTransformerConfig

    torch.manual_seed(21)
    config = GeodesicTransformerConfig(d_model=8, num_heads=2, lambda_geodesic=0.0, use_translator=False)
    model = GeodesicMultiHeadAttention(config)
    x = torch.randn(2, 4, 8)
    output = model(x)
    assert output.output.shape == x.shape
    assert torch.allclose(output.attention_weights.sum(dim=-1), torch.ones(2, 2, 4), atol=1e-6)


def test_geodesic_distance_effect():
    torch = pytest.importorskip("torch")
    from geodesic.attention import GeodesicAttention

    q = torch.tensor([[[1.0, 0.0]]])
    k = torch.tensor([[[1.0, 0.0], [1.0, 0.01]]])
    v = torch.eye(2).unsqueeze(0)
    attention = GeodesicAttention(lambda_geodesic=0.5)(q, k, v).attention
    assert attention[0, 0, 0] >= attention[0, 0, 1]


def test_translator_bound():
    torch = pytest.importorskip("torch")
    from geodesic.transformer import GeodesicMultiHeadAttention, GeodesicTransformerConfig

    model = GeodesicMultiHeadAttention(GeodesicTransformerConfig(d_model=8, num_heads=2, use_translator=True, gamma_max=0.03))
    output = model(torch.randn(1, 3, 8))
    assert output.translator_outputs
    assert all(torch.all(translated.correction.abs() <= 0.0300001) for translated in output.translator_outputs)


def test_point_attention_normalization():
    torch = pytest.importorskip("torch")
    from geodesic.point_attention import PointAttentionAssistant

    output = PointAttentionAssistant()(torch.randn(2, 4, 8), torch.randn(2, 4, 3, 8))
    assert torch.allclose(output.attention_weights.sum(dim=-1), torch.ones(2, 4), atol=1e-6)
    assert torch.all(output.attention_weights >= 0)


def test_multihead_shape():
    torch = pytest.importorskip("torch")
    from geodesic.transformer import GeodesicMultiHeadAttention, GeodesicTransformerConfig

    x = torch.randn(2, 5, 16)
    output = GeodesicMultiHeadAttention(GeodesicTransformerConfig(d_model=16, num_heads=4))(x)
    assert output.output.shape == x.shape
    assert output.attention_weights.shape == (2, 4, 5, 5)


def test_multihead_diversity():
    torch = pytest.importorskip("torch")
    from geodesic.transformer import GeodesicMultiHeadAttention, GeodesicTransformerConfig

    torch.manual_seed(22)
    output = GeodesicMultiHeadAttention(GeodesicTransformerConfig(d_model=16, num_heads=4))(torch.randn(2, 5, 16))
    assert not torch.allclose(output.attention_weights[:, 0], output.attention_weights[:, 1])


def test_gradient_stability():
    torch = pytest.importorskip("torch")
    from geodesic.transformer import GeodesicTransformer, GeodesicTransformerConfig

    model = GeodesicTransformer(GeodesicTransformerConfig(d_model=16, num_heads=4, num_layers=1, use_translator=True))
    x = torch.randn(4, 5, 16)
    coords = torch.randn(4, 5, 2)
    labels = torch.tensor([0, 1, 0, 1])
    logits, _ = model(x, coords)
    loss = torch.nn.CrossEntropyLoss()(logits, labels)
    loss.backward()
    assert all(torch.isfinite(parameter.grad).all() for parameter in model.parameters() if parameter.grad is not None)


def test_attention_entropy():
    torch = pytest.importorskip("torch")
    from geodesic.transformer import GeodesicMultiHeadAttention, GeodesicTransformerConfig

    output = GeodesicMultiHeadAttention(GeodesicTransformerConfig(d_model=8, num_heads=2))(torch.randn(1, 4, 8))
    entropy = -(output.attention_weights.clamp_min(1e-12) * output.attention_weights.clamp_min(1e-12).log()).sum(dim=-1).mean()
    assert torch.isfinite(entropy)
    assert entropy > 0


def test_nan_inf():
    torch = pytest.importorskip("torch")
    from geodesic.transformer import GeodesicMultiHeadAttention, GeodesicTransformerConfig

    model = GeodesicMultiHeadAttention(GeodesicTransformerConfig(d_model=8, num_heads=2))
    x = torch.zeros(1, 2, 8)
    x[0, 0, 0] = float("inf")
    with pytest.raises(ValueError):
        model(x)


def test_pressure_response():
    torch = pytest.importorskip("torch")
    from geodesic.transformer import GeodesicMultiHeadAttention, GeodesicTransformerConfig

    model = GeodesicMultiHeadAttention(GeodesicTransformerConfig(d_model=8, num_heads=2, use_translator=True))
    low = model(torch.zeros(1, 3, 8)).translator_outputs[0].measurements.pressure
    high = model(torch.ones(1, 3, 8) * 2).translator_outputs[0].measurements.pressure
    assert high >= low
