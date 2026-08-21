"""Geodesic attention v0.1 implemented with PyTorch."""

from __future__ import annotations

import math
from dataclasses import dataclass

import importlib.util

if importlib.util.find_spec("torch") is None:  # pragma: no cover - environment dependent
    torch = None  # type: ignore[assignment]
else:
    import torch

from geodesic.core.metric import named_p_norm
from geodesic.distances import pairwise_geodesic_distances
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

    def __init__(
        self,
        lambda_geodesic: float = 0.0,
        translator: GeodesicTranslator | None = None,
        geodesic_metric: str = "euclidean",
    ) -> None:
        if torch is None:
            raise ModuleNotFoundError("GeodesicAttention requires PyTorch")
        if lambda_geodesic < 0:
            raise ValueError("lambda_geodesic must be non-negative")
        named_p_norm(geodesic_metric)
        self.lambda_geodesic = float(lambda_geodesic)
        self.translator = translator
        self.geodesic_metric = geodesic_metric

    def geodesic_distances(self, queries: "torch.Tensor", keys: "torch.Tensor") -> "torch.Tensor":
        return pairwise_geodesic_distances(queries, keys, self.geodesic_metric)

    def __call__(
        self,
        queries: "torch.Tensor",
        keys: "torch.Tensor",
        values: "torch.Tensor",
        previous_queries: "torch.Tensor | None" = None,
    ) -> AttentionOutput:
        if not torch.isfinite(queries).all() or not torch.isfinite(keys).all() or not torch.isfinite(values).all():
            raise ValueError("GeodesicAttention rejects NaN/Inf inputs")
        dim = queries.shape[-1]
        content_scores = queries @ keys.transpose(-2, -1) / math.sqrt(dim)
        distances = self.geodesic_distances(queries, keys)
        scores = content_scores - self.lambda_geodesic * distances
        translator_output = None
        if self.translator is not None:
            translator_output = self.translator(queries, keys, distances, previous_queries)
            scores = scores + translator_output.correction
        attention = torch.softmax(scores, dim=-1)
        if not torch.isfinite(attention).all():
            raise ValueError("GeodesicAttention produced non-finite attention")
        return AttentionOutput(scores=scores, attention=attention, output=attention @ values, geodesic_distances=distances, translator=translator_output)


def scaled_dot_product_attention(queries: "torch.Tensor", keys: "torch.Tensor", values: "torch.Tensor") -> "torch.Tensor":
    scores = queries @ keys.transpose(-2, -1) / math.sqrt(queries.shape[-1])
    return torch.softmax(scores, dim=-1) @ values
