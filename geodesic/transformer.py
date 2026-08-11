"""Production-oriented Geodesic Transformer v0.1 components."""

from __future__ import annotations

import importlib.util
import math
import time
from dataclasses import dataclass
from typing import Any

if importlib.util.find_spec("torch") is None:  # pragma: no cover - environment dependent
    torch = None  # type: ignore[assignment]
    nn = None  # type: ignore[assignment]
    TorchModule = object
else:
    import torch
    from torch import nn
    TorchModule = nn.Module

from geodesic.point_attention import PointAttentionAssistant
from geodesic.translator import GeodesicTranslator, TranslatorOutput


@dataclass(frozen=True)
class GeodesicTransformerConfig:
    """Configuration for Geodesic Transformer v0.1."""

    d_model: int = 32
    num_heads: int = 4
    num_layers: int = 2
    lambda_geodesic: float = 0.1
    gamma_max: float = 0.1
    dropout: float = 0.0
    geodesic_metric: str = "euclidean"
    use_point_attention: bool = False
    use_translator: bool = False
    ffn_multiplier: int = 4
    radial_dim: int = 1


@dataclass(frozen=True)
class GeodesicMultiHeadAttentionOutput:
    output: "torch.Tensor"
    attention_weights: "torch.Tensor"
    scores: "torch.Tensor"
    geodesic_distances: "torch.Tensor"
    translator_outputs: tuple[TranslatorOutput, ...]


class GeodesicPositionalRepresentation(TorchModule):
    """Combine token content with compact normalized geodesic coordinates."""

    def __init__(self, d_model: int, coordinate_dim: int = 2, radial_dim: int = 1) -> None:
        if torch is None or nn is None:
            raise ModuleNotFoundError("GeodesicPositionalRepresentation requires PyTorch")
        super().__init__()
        self.coordinate_projection = nn.Linear(coordinate_dim + radial_dim, d_model)

    def forward(self, content: "torch.Tensor", coordinates: "torch.Tensor", radial: "torch.Tensor | None" = None) -> "torch.Tensor":
        if not torch.isfinite(content).all() or not torch.isfinite(coordinates).all():
            raise ValueError("Geodesic positional representation rejects NaN/Inf inputs")
        norm = coordinates.norm(dim=-1, keepdim=True).clamp_min(1e-12)
        normalized = coordinates / norm
        if radial is None:
            radial = norm
        features = torch.cat([normalized, radial], dim=-1)
        return content + self.coordinate_projection(features)


class GeodesicMultiHeadAttention(TorchModule):
    """Multi-head attention with per-head geodesic penalties and bounded correction."""

    def __init__(self, config: GeodesicTransformerConfig) -> None:
        if torch is None or nn is None:
            raise ModuleNotFoundError("GeodesicMultiHeadAttention requires PyTorch")
        super().__init__()
        if config.d_model % config.num_heads != 0:
            raise ValueError("d_model must be divisible by num_heads")
        if config.lambda_geodesic < 0:
            raise ValueError("lambda_geodesic must be non-negative")
        self.config = config
        self.head_dim = config.d_model // config.num_heads
        self.q_proj = nn.Linear(config.d_model, config.d_model)
        self.k_proj = nn.Linear(config.d_model, config.d_model)
        self.v_proj = nn.Linear(config.d_model, config.d_model)
        self.out_proj = nn.Linear(config.d_model, config.d_model)
        self.dropout = nn.Dropout(config.dropout)
        self.translator = GeodesicTranslator(config.gamma_max) if config.use_translator else None
        self.point_attention = PointAttentionAssistant() if config.use_point_attention else None

    def _split_heads(self, tensor: "torch.Tensor") -> "torch.Tensor":
        batch, tokens, _ = tensor.shape
        return tensor.view(batch, tokens, self.config.num_heads, self.head_dim).transpose(1, 2)

    def _merge_heads(self, tensor: "torch.Tensor") -> "torch.Tensor":
        batch, heads, tokens, dim = tensor.shape
        return tensor.transpose(1, 2).contiguous().view(batch, tokens, heads * dim)

    def forward(self, x: "torch.Tensor", geodesic_coordinates: "torch.Tensor | None" = None) -> GeodesicMultiHeadAttentionOutput:
        if not torch.isfinite(x).all():
            raise ValueError("GeodesicMultiHeadAttention rejects NaN/Inf inputs")
        q = self._split_heads(self.q_proj(x))
        k = self._split_heads(self.k_proj(x))
        v = self._split_heads(self.v_proj(x))
        content_scores = q @ k.transpose(-2, -1) / math.sqrt(self.head_dim)
        distances = torch.cdist(q.flatten(0, 1), k.flatten(0, 1), p=2).view(x.shape[0], self.config.num_heads, x.shape[1], x.shape[1])
        if geodesic_coordinates is not None:
            if not torch.isfinite(geodesic_coordinates).all():
                raise ValueError("GeodesicMultiHeadAttention rejects NaN/Inf coordinates")
            coordinate_distances = torch.cdist(geodesic_coordinates, geodesic_coordinates, p=2).unsqueeze(1)
            distances = 0.5 * (distances + coordinate_distances)
        scores = content_scores - self.config.lambda_geodesic * distances
        translator_outputs: list[TranslatorOutput] = []
        if self.translator is not None:
            translator_distances = distances
            if self.point_attention is not None:
                neighborhoods = k.transpose(1, 2).unsqueeze(2).expand(-1, -1, x.shape[1], -1, -1)
                neighborhoods = neighborhoods.reshape(x.shape[0] * x.shape[1], x.shape[1], self.config.num_heads * self.head_dim)
                local = x.reshape(x.shape[0] * x.shape[1], 1, self.config.d_model)
                point_signal = self.point_attention(local, neighborhoods.unsqueeze(1)).points.norm(dim=-1).view(x.shape[0], 1, x.shape[1], 1)
                translator_distances = distances + point_signal.clamp_min(0.0)
            for head in range(self.config.num_heads):
                translated = self.translator(q[:, head], k[:, head], translator_distances[:, head])
                translator_outputs.append(translated)
                scores[:, head] = scores[:, head] + translated.correction
        scores = scores.clamp(min=-80.0, max=80.0)
        attention = torch.softmax(scores, dim=-1)
        if not torch.isfinite(attention).all():
            raise ValueError("GeodesicMultiHeadAttention produced non-finite attention")
        head_output = self.dropout(attention) @ v
        output = self.out_proj(self._merge_heads(head_output))
        return GeodesicMultiHeadAttentionOutput(output, attention, scores, distances, tuple(translator_outputs))


class GeodesicFFN(TorchModule):
    """Feed-forward network used inside the geodesic block."""

    def __init__(self, config: GeodesicTransformerConfig) -> None:
        if torch is None or nn is None:
            raise ModuleNotFoundError("GeodesicFFN requires PyTorch")
        super().__init__()
        hidden = config.d_model * config.ffn_multiplier
        self.network = nn.Sequential(nn.Linear(config.d_model, hidden), nn.GELU(), nn.Dropout(config.dropout), nn.Linear(hidden, config.d_model))

    def forward(self, x: "torch.Tensor") -> "torch.Tensor":
        return self.network(x)


class GeodesicTransformerBlock(TorchModule):
    """Residual geodesic Transformer block with LayerNorm and geodesic FFN."""

    def __init__(self, config: GeodesicTransformerConfig) -> None:
        if torch is None or nn is None:
            raise ModuleNotFoundError("GeodesicTransformerBlock requires PyTorch")
        super().__init__()
        self.attention = GeodesicMultiHeadAttention(config)
        self.attention_norm = nn.LayerNorm(config.d_model)
        self.ffn = GeodesicFFN(config)
        self.ffn_norm = nn.LayerNorm(config.d_model)
        self.dropout = nn.Dropout(config.dropout)

    def forward(self, x: "torch.Tensor", geodesic_coordinates: "torch.Tensor | None" = None) -> tuple["torch.Tensor", GeodesicMultiHeadAttentionOutput]:
        attention_output = self.attention(x, geodesic_coordinates)
        x = self.attention_norm(x + self.dropout(attention_output.output))
        x = self.ffn_norm(x + self.dropout(self.ffn(x)))
        return x, attention_output


class GeodesicTransformer(TorchModule):
    """Stack of Geodesic Transformer blocks for synthetic validation."""

    def __init__(self, config: GeodesicTransformerConfig) -> None:
        if torch is None or nn is None:
            raise ModuleNotFoundError("GeodesicTransformer requires PyTorch")
        super().__init__()
        self.config = config
        self.layers = nn.ModuleList([GeodesicTransformerBlock(config) for _ in range(config.num_layers)])
        self.classifier = nn.Linear(config.d_model, 2)

    def forward(self, x: "torch.Tensor", geodesic_coordinates: "torch.Tensor | None" = None) -> tuple["torch.Tensor", tuple[GeodesicMultiHeadAttentionOutput, ...]]:
        traces: list[GeodesicMultiHeadAttentionOutput] = []
        for layer in self.layers:
            x, trace = layer(x, geodesic_coordinates)
            traces.append(trace)
        pooled = x.mean(dim=1)
        return self.classifier(pooled), tuple(traces)


def count_parameters(module: "nn.Module") -> int:
    """Return trainable parameter count."""

    return sum(parameter.numel() for parameter in module.parameters() if parameter.requires_grad)


def timed_forward_backward(model: "GeodesicTransformer", x: "torch.Tensor", coordinates: "torch.Tensor", labels: "torch.Tensor") -> dict[str, Any]:
    """Run one deterministic synthetic training step and collect stability metrics."""

    criterion = nn.CrossEntropyLoss()
    start = time.perf_counter()
    logits, traces = model(x, coordinates)
    loss = criterion(logits, labels)
    loss.backward()
    runtime = time.perf_counter() - start
    prediction = logits.argmax(dim=-1)
    first = traces[0]
    pressure_values = [out.measurements.pressure for trace in traces for out in trace.translator_outputs]
    return {
        "loss": float(loss.detach().cpu()),
        "accuracy": float((prediction == labels).float().mean().detach().cpu()),
        "attention_entropy": float((-(first.attention_weights.clamp_min(1e-12) * first.attention_weights.clamp_min(1e-12).log()).sum(dim=-1).mean()).detach().cpu()),
        "gradient_norm": float(sum((p.grad.norm().detach().cpu() for p in model.parameters() if p.grad is not None), torch.tensor(0.0)).item()),
        "geodesic_error": float(first.geodesic_distances.mean().detach().cpu()),
        "curvature_energy": float(((first.geodesic_distances - first.geodesic_distances.mean(dim=-1, keepdim=True)).square().mean()).detach().cpu()),
        "translator_pressure": float(sum(pressure_values) / len(pressure_values)) if pressure_values else 0.0,
        "runtime_seconds": runtime,
        "parameter_count": count_parameters(model),
        "nan_or_inf": not bool(torch.isfinite(logits).all() and all(torch.isfinite(p.grad).all() for p in model.parameters() if p.grad is not None)),
    }
