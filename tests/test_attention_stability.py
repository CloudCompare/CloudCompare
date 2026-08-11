import pytest

torch = pytest.importorskip("torch")

from geodesic.attention import GeodesicAttention
from geodesic.translator import GeodesicTranslator
from experiments.translator_v01 import run


def test_attention_stability_under_increasing_perturbation():
    torch.manual_seed(3)
    model = GeodesicAttention(lambda_geodesic=0.25, translator=GeodesicTranslator(gamma_max=0.1))
    pressures = []
    for scale in [0.0, 0.1, 0.25, 0.5, 1.0]:
        q = (torch.randn(1, 4, 6) * (1 + scale)).requires_grad_(True)
        k = torch.randn(1, 4, 6)
        v = torch.randn(1, 4, 6)
        output = model(q, k, v, previous_queries=q.detach() - scale)
        loss = output.output.square().mean()
        loss.backward()
        assert torch.isfinite(output.attention).all()
        assert torch.isfinite(output.scores).all()
        assert torch.isfinite(q.grad).all()
        assert output.translator is not None
        pressures.append(output.translator.measurements.pressure)
    assert pressures[-1] > pressures[0]


def test_lambda_sweep_and_ablation_generates_results():
    payload = run()
    assert payload["test_results"]["lambda_sweep_completed"] is True
    assert payload["test_results"]["ablation_generated"] is True
    assert payload["test_results"]["no_nan_or_inf"] is True
