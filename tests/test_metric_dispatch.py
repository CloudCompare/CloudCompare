import pytest

from geodesic.core.metric import named_p_norm


def test_named_p_norm_maps_kernel_aliases():
    assert named_p_norm("euclidean") == 2
    assert named_p_norm("manhattan") == 1
    with pytest.raises(ValueError):
        named_p_norm("chebyshev")


def test_attention_and_transformer_dispatch_metrics_when_torch_available():
    torch = pytest.importorskip("torch")
    from geodesic.attention import GeodesicAttention
    from geodesic.transformer import GeodesicMultiHeadAttention, GeodesicTransformerConfig

    queries = torch.tensor([[[0.0, 0.0]]])
    keys = torch.tensor([[[3.0, 4.0]]])
    euclidean = GeodesicAttention(geodesic_metric="euclidean").geodesic_distances(queries, keys)
    manhattan = GeodesicAttention(geodesic_metric="manhattan").geodesic_distances(queries, keys)
    assert torch.isclose(euclidean.squeeze(), torch.tensor(5.0))
    assert torch.isclose(manhattan.squeeze(), torch.tensor(7.0))

    model = GeodesicMultiHeadAttention(GeodesicTransformerConfig(d_model=8, num_heads=2, geodesic_metric="manhattan"))
    output = model(torch.randn(1, 3, 8), torch.tensor([[[0.0, 0.0], [3.0, 4.0], [1.0, 1.0]]]))
    assert output.geodesic_distances.shape == (1, 2, 3, 3)
    assert torch.isfinite(output.geodesic_distances).all()
