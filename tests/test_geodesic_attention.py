import pytest

torch = pytest.importorskip("torch")

from geodesic.attention import GeodesicAttention, scaled_dot_product_attention
from geodesic.translator import GeodesicTranslator


def test_baseline_equivalence():
    torch.manual_seed(1)
    q, k, v = torch.randn(2, 3, 4), torch.randn(2, 3, 4), torch.randn(2, 3, 4)
    actual = GeodesicAttention(lambda_geodesic=0)(q, k, v).output
    expected = scaled_dot_product_attention(q, k, v)
    assert torch.allclose(actual, expected, atol=1e-6)


def test_geodesic_sensitivity():
    q = torch.tensor([[[1.0, 0.0]]])
    k = torch.tensor([[[1.0, 0.0], [1.0, 0.001]]])
    v = torch.eye(2).unsqueeze(0)
    attention = GeodesicAttention(lambda_geodesic=1.0)(q, k, v).attention
    assert attention[0, 0, 0] >= attention[0, 0, 1]


def test_translator_boundedness_in_attention():
    q = torch.randn(1, 3, 4)
    k = torch.randn(1, 3, 4)
    distances = torch.cdist(q, k)
    gamma_max = 0.05
    output = GeodesicTranslator(gamma_max=gamma_max)(q, k, distances)
    assert torch.all(output.correction.abs() <= gamma_max + 1e-7)
