import pytest


def test_single_key_uncertainty_is_stable_when_torch_available():
    torch = pytest.importorskip("torch")
    from geodesic.translator import GeodesicTranslator

    translator = GeodesicTranslator(gamma_max=0.1)
    queries = torch.zeros(1, 1, 3)
    keys = torch.zeros(1, 1, 3)
    distances = torch.zeros(1, 1, 1)
    output = translator(queries, keys, distances)
    assert output.measurements.uncertainty == 0.0
    assert torch.isfinite(output.correction).all()


def test_learnable_g_theta_is_bounded_and_has_gradients():
    torch = pytest.importorskip("torch")
    from geodesic.translator import GThetaController, GeodesicTranslator

    controller = GThetaController(hidden=8)
    translator = GeodesicTranslator(gamma_max=0.05, controller=controller)
    queries = torch.randn(2, 3, 4, requires_grad=True)
    keys = torch.randn(2, 3, 4)
    distances = torch.cdist(queries, keys)
    output = translator(queries, keys, distances)
    assert torch.all(output.correction.abs() <= 0.05 + 1e-6)
    output.correction.mean().backward()
    assert any(parameter.grad is not None and torch.isfinite(parameter.grad).all() for parameter in controller.parameters())


def test_attention_rejects_non_finite_inputs_when_torch_available():
    torch = pytest.importorskip("torch")
    from geodesic.attention import GeodesicAttention

    queries = torch.zeros(1, 2, 3)
    queries[0, 0, 0] = float("nan")
    with pytest.raises(ValueError):
        GeodesicAttention()(queries, torch.zeros(1, 2, 3), torch.zeros(1, 2, 3))
