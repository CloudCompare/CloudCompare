"""Deterministic Step 2 benchmark for Geodesic Translator v0.1."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import importlib.util

if importlib.util.find_spec("torch") is None:  # pragma: no cover - environment dependent
    torch = None  # type: ignore[assignment]
else:
    import torch

from geodesic.attention import GeodesicAttention
from geodesic.translator import GeodesicTranslator

LAMBDAS = [0, 0.05, 0.1, 0.25, 0.5, 1.0]
GAMMA_MAX = 0.1
RESULTS_PATH = Path(__file__).with_name("results") / "translator_v01.json"


def entropy(attention: "torch.Tensor") -> float:
    probs = attention.clamp_min(1e-12)
    return float((-(probs * probs.log()).sum(dim=-1).mean()).detach().cpu())


def score_range(scores: "torch.Tensor") -> float:
    return float((scores.max() - scores.min()).detach().cpu())


def run() -> dict[str, Any]:
    if torch is None:
        raise ModuleNotFoundError("translator_v01 benchmark requires PyTorch")
    torch.manual_seed(1729)
    q = torch.randn(2, 4, 8, requires_grad=True)
    k = torch.randn(2, 4, 8)
    v = torch.randn(2, 4, 8)
    previous = q.detach() - 0.05
    results: list[dict[str, Any]] = []
    for lambda_value in LAMBDAS:
        modes = {
            "baseline": GeodesicAttention(lambda_geodesic=0),
            "geodesic": GeodesicAttention(lambda_geodesic=lambda_value),
            "geodesic_translator": GeodesicAttention(lambda_geodesic=lambda_value, translator=GeodesicTranslator(gamma_max=GAMMA_MAX)),
        }
        for mode, attention_model in modes.items():
            q_run = q.clone().detach().requires_grad_(True)
            output = attention_model(q_run, k, v, previous_queries=previous)
            loss = output.output.square().mean()
            loss.backward()
            finite = torch.isfinite(output.scores).all() and torch.isfinite(output.attention).all() and torch.isfinite(q_run.grad).all()
            measurements = output.translator.measurements if output.translator else None
            pressure = measurements.pressure if measurements else 0.0
            geodesic_error = measurements.geodesic_error if measurements else float(output.geodesic_distances.mean().detach().cpu())
            mirror_error = measurements.mirror_error if measurements else 0.0
            results.append({
                "mode": mode,
                "lambda": lambda_value,
                "gamma_max": GAMMA_MAX if output.translator else 0.0,
                "metrics": {
                    "attention_entropy": entropy(output.attention),
                    "gradient_norm": float(q_run.grad.norm().detach().cpu()),
                    "score_range": score_range(output.scores),
                    "nan_or_inf": not bool(finite),
                    "geodesic_error": geodesic_error,
                    "mirror_error": mirror_error,
                    "pressure": pressure,
                },
                "translator_state": output.translator.translator_state.value if output.translator else None,
                "pass": bool(finite),
            })
    stable = [r["lambda"] for r in results if r["mode"] == "geodesic_translator" and r["pass"] and r["metrics"]["attention_entropy"] > 0.5]
    payload = {
        "configuration": {"seed": 1729, "lambdas": LAMBDAS, "gamma_max": GAMMA_MAX, "attention": "softmax(S)"},
        "lambda_sweep": results,
        "summary": {
            "stable_operating_region": [min(stable), max(stable)] if stable else [],
            "attention_collapse_region": [r["lambda"] for r in results if r["mode"] == "geodesic_translator" and r["metrics"]["attention_entropy"] <= 0.5],
            "excessive_geodesic_constraint_region": [r["lambda"] for r in results if r["mode"] == "geodesic_translator" and r["metrics"]["score_range"] > 10.0],
        },
        "test_results": {"lambda_sweep_completed": True, "ablation_generated": True, "no_nan_or_inf": all(r["pass"] for r in results)},
        "pass": all(r["pass"] for r in results),
    }
    RESULTS_PATH.parent.mkdir(parents=True, exist_ok=True)
    RESULTS_PATH.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return payload


if __name__ == "__main__":
    print(json.dumps(run(), indent=2))
