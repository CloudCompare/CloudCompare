"""Shared geodesic distance helpers for tensor modules."""

from __future__ import annotations

import importlib.util

from geodesic.core.metric import named_p_norm

if importlib.util.find_spec("torch") is None:  # pragma: no cover - environment dependent
    torch = None  # type: ignore[assignment]
else:
    import torch


def pairwise_geodesic_distances(left: "torch.Tensor", right: "torch.Tensor", metric: str = "euclidean") -> "torch.Tensor":
    """Return pairwise distances using the named kernel metric."""

    if torch is None:
        raise ModuleNotFoundError("pairwise geodesic distances require PyTorch")
    return torch.cdist(left, right, p=named_p_norm(metric))
