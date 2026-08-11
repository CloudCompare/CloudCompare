"""Step 3 synthetic benchmark for Geodesic Transformer v0.1."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from typing import Any

if importlib.util.find_spec("torch") is None:  # pragma: no cover - environment dependent
    torch = None  # type: ignore[assignment]
else:
    import torch

from geodesic.transformer import GeodesicTransformer, GeodesicTransformerConfig, timed_forward_backward

RESULTS_PATH = Path(__file__).with_name("results") / "geodesic_transformer_v01.json"
LAMBDAS = [0, 0.05, 0.1, 0.25, 0.5, 1.0]


def _blocked_payload() -> dict[str, Any]:
    return {
        "configuration": {"seed": 2027, "lambdas": LAMBDAS, "ablations": ["A_standard", "B_geodesic", "C_geodesic_translator", "D_geodesic_translator_point"]},
        "status": "IMPLEMENTED_VALIDATION_BLOCKED",
        "blocked_reason": "PyTorch is unavailable; critical Step 3 tests and benchmarks cannot execute.",
        "results": [],
    }


def run() -> dict[str, Any]:
    if torch is None:
        payload = _blocked_payload()
        RESULTS_PATH.parent.mkdir(parents=True, exist_ok=True)
        RESULTS_PATH.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        return payload

    torch.manual_seed(2027)
    batch, tokens, d_model = 8, 6, 32
    x = torch.randn(batch, tokens, d_model)
    coordinates = torch.randn(batch, tokens, 2)
    labels = (x.mean(dim=(1, 2)) > 0).long()
    ablations = {
        "A_standard": {"lambda_geodesic": 0.0, "use_translator": False, "use_point_attention": False},
        "B_geodesic": {"use_translator": False, "use_point_attention": False},
        "C_geodesic_translator": {"use_translator": True, "use_point_attention": False},
        "D_geodesic_translator_point": {"use_translator": True, "use_point_attention": True},
    }
    results: list[dict[str, Any]] = []
    for lambda_value in LAMBDAS:
        for name, overrides in ablations.items():
            effective_lambda = overrides.get("lambda_geodesic", lambda_value)
            config = GeodesicTransformerConfig(lambda_geodesic=effective_lambda, use_translator=overrides["use_translator"], use_point_attention=overrides["use_point_attention"])
            model = GeodesicTransformer(config)
            metrics = timed_forward_backward(model, x.clone(), coordinates.clone(), labels)
            results.append({"ablation": name, "lambda": effective_lambda, "metrics": metrics, "pass": not metrics["nan_or_inf"]})
    payload = {
        "configuration": {"seed": 2027, "lambdas": LAMBDAS, "d_model": d_model, "num_heads": 4, "num_layers": 2, "gamma_max": 0.1},
        "status": "PASS" if all(row["pass"] for row in results) else "FAIL",
        "results": results,
        "summary": {
            "all_ablations_execute": len({row["ablation"] for row in results}) == 4,
            "lambda_sweep_completed": len({row["lambda"] for row in results}) >= len(LAMBDAS),
            "nan_or_inf": any(row["metrics"]["nan_or_inf"] for row in results),
        },
    }
    RESULTS_PATH.parent.mkdir(parents=True, exist_ok=True)
    RESULTS_PATH.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return payload


if __name__ == "__main__":
    print(json.dumps(run(), indent=2))
