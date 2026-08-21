"""Optional learned translator controller ``g_theta``.

The controller only proposes an unbounded raw state. Callers must still apply
the frozen bound ``Γ = γ_max * tanh(g_theta(state))``.
"""

from __future__ import annotations

import importlib.util

if importlib.util.find_spec("torch") is None:  # pragma: no cover - environment dependent
    torch = None  # type: ignore[assignment]
    nn = None  # type: ignore[assignment]
    TorchModule = object
else:
    import torch
    from torch import nn
    TorchModule = nn.Module


class GThetaController(TorchModule):
    """Small MLP that maps translator feature vectors to a raw scalar."""

    def __init__(self, hidden: int = 8, feature_dim: int = 6) -> None:
        if torch is None or nn is None:
            raise ModuleNotFoundError("GThetaController requires PyTorch")
        if hidden <= 0 or feature_dim <= 0:
            raise ValueError("hidden and feature_dim must be positive")
        super().__init__()
        self.net = nn.Sequential(nn.Linear(feature_dim, hidden), nn.Tanh(), nn.Linear(hidden, 1))

    def forward(self, features: "torch.Tensor") -> "torch.Tensor":
        if features.ndim == 1:
            features = features.unsqueeze(0)
            return self.net(features).squeeze(-1).squeeze(0)
        return self.net(features).squeeze(-1)
