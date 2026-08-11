"""Geodesic attention v0.1 implemented with PyTorch."""

from __future__ import annotations

import math
from dataclasses import dataclass

import importlib.util

if importlib.util.find_spec("torch") is None:  # pragma: no cover - environment dependent
    torch = None  # type: ignore[assignment]
else:
    import torch

from geodesic.translator import GeodesicTranslator, TranslatorOutput


@dataclass(frozen=True)
class AttentionOutput:
    scores: "torch.Tensor"
    attention: "torch.Tensor"
    output: "torch.Tensor"
    geodesic_distances: "torch.Tensor"
    translator: TranslatorOutput | None


class GeodesicAttention:
    """Scaled dot-product attention with geodesic penalty and bounded correction."""

    def __init__(self, lambda_geodesic: float = 0.0, translator: GeodesicTranslator | None = None) -> None:
        if torch is None:
            raise ModuleNotFoundError("GeodesicAttention requires PyTorch")
        if lambda_geodesic < 0:
            raise ValueError("lambda_geodesic must be non-negative")
        self.lambda_geodesic = float(lambda_geodesic)
        self.translator = translator

    def geodesic_distances(self, queries: "torch.Tensor", keys: "torch.Tensor") -> "torch.Tensor":
        return torch.cdist(queries, keys, p=2)

    def __call__(self, queries: "torch.Tensor", keys: "torch.Tensor", values: "torch.Tensor", previous_queries: "torch.Tensor | None" = None) -> AttentionOutput:
        dim = queries.shape[-1]
        content_scores = queries @ keys.transpose(-2, -1) / math.sqrt(dim)
        distances = self.geodesic_distances(queries, keys)
        scores = content_scores - self.lambda_geodesic * distances
        translator_output = None
        if self.translator is not None:
            translator_output = self.translator(queries, keys, distances, previous_queries)
            scores = scores + translator_output.correction
        attention = torch.softmax(scores, dim=-1)
        return AttentionOutput(scores=scores, attention=attention, output=attention @ values, geodesic_distances=distances, translator=translator_output)


def scaled_dot_product_attention(queries: "torch.Tensor", keys: "torch.Tensor", values: "torch.Tensor") -> "torch.Tensor":
    scores = queries @ keys.transpose(-2, -1) / math.sqrt(queries.shape[-1])
    return torch.softmax(scores, dim=-1) @ values
