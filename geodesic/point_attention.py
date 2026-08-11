"""Point Attention Assistant v0.1 isolated from full Transformer stacks."""

from __future__ import annotations

import importlib.util
import math
from dataclasses import dataclass

if importlib.util.find_spec("torch") is None:  # pragma: no cover - environment dependent
    torch = None  # type: ignore[assignment]
else:
    import torch


@dataclass(frozen=True)
class PointAttentionOutput:
    """Point attention output and exposed normalized weights."""

    points: "torch.Tensor"
    attention_weights: "torch.Tensor"
    scores: "torch.Tensor"


class PointAttentionAssistant:
    """Local point attention over batched point neighborhoods.

    Given local states ``x_i`` with shape ``(batch, points, dim)`` and
    neighborhoods ``N_i`` with shape ``(batch, points, neighbors, dim)``, the
    assistant computes normalized weights over each local neighborhood and
    returns ``p_i`` with the same shape as ``x_i``.
    """

    def __init__(self) -> None:
        if torch is None:
            raise ModuleNotFoundError("PointAttentionAssistant requires PyTorch")

    def __call__(self, local_states: "torch.Tensor", neighborhoods: "torch.Tensor") -> PointAttentionOutput:
        if local_states.ndim != 3:
            raise ValueError("local_states must have shape (batch, points, dim)")
        if neighborhoods.ndim != 4:
            raise ValueError("neighborhoods must have shape (batch, points, neighbors, dim)")
        if local_states.shape[:2] != neighborhoods.shape[:2] or local_states.shape[-1] != neighborhoods.shape[-1]:
            raise ValueError("local_states and neighborhoods must agree on batch, point, and feature dimensions")
        if not torch.isfinite(local_states).all() or not torch.isfinite(neighborhoods).all():
            raise ValueError("PointAttentionAssistant rejects NaN/Inf inputs")

        dim = local_states.shape[-1]
        scores = (local_states.unsqueeze(-2) * neighborhoods).sum(dim=-1) / math.sqrt(dim)
        weights = torch.softmax(scores, dim=-1)
        if not torch.isfinite(weights).all():
            raise ValueError("PointAttentionAssistant produced non-finite attention weights")
        points = (weights.unsqueeze(-1) * neighborhoods).sum(dim=-2)
        return PointAttentionOutput(points=points, attention_weights=weights, scores=scores)
