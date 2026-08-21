"""Translate records and regulate geodesic attention with bounded corrections."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from collections.abc import Iterable, Mapping, Sequence
from typing import Any

from geodesic.core.anchor import Anchor
from geodesic.core.manifold import Manifold

import importlib.util

if importlib.util.find_spec("torch") is None:  # pragma: no cover - environment dependent
    torch = None  # type: ignore[assignment]
else:
    import torch


class TranslatorState(str, Enum):
    """Deterministic regulation states for the Step 2 translator."""

    NORMAL = "NORMAL"
    CAUTION = "CAUTION"
    PRESSURE = "PRESSURE"
    RECOVERY = "RECOVERY"
    REJECT = "REJECT"


@dataclass(frozen=True)
class TranslatorMeasurements:
    """Scalar summary of geometric and state measurements."""

    geodesic_error: float
    mirror_error: float
    curvature_energy: float
    state_velocity: float
    uncertainty: float
    pressure: float


@dataclass(frozen=True)
class TranslatorOutput:
    """Bounded translator correction and state."""

    translator_state: TranslatorState
    correction: "torch.Tensor"
    measurements: TranslatorMeasurements


class Translator:
    """Build anchors and manifolds from dictionaries or coordinate sequences."""

    def anchor_from_record(self, record: Mapping[str, Any]) -> Anchor:
        name = str(record.get("name") or record.get("id") or "anchor")
        coordinates = record.get("coordinates")
        if coordinates is None:
            raise ValueError("record must contain coordinates")
        metadata = {key: value for key, value in record.items() if key not in {"name", "id", "coordinates"}}
        return Anchor(name=name, coordinates=tuple(coordinates), metadata=metadata)

    def manifold_from_records(self, records: Iterable[Mapping[str, Any]], dimensions: int | None = None) -> Manifold:
        anchors = [self.anchor_from_record(record) for record in records]
        if not anchors and dimensions is None:
            raise ValueError("dimensions are required when records are empty")
        resolved_dimensions = dimensions or len(anchors[0].coordinates)
        manifold = Manifold(dimensions=resolved_dimensions)
        for anchor in anchors:
            manifold.add_anchor(anchor)
        return manifold

    def encode_text(self, text: str) -> Sequence[float]:
        """Encode text as deterministic normalized character statistics."""

        if not text:
            return (0.0, 0.0, 0.0)
        letters = sum(character.isalpha() for character in text)
        digits = sum(character.isdigit() for character in text)
        spaces = sum(character.isspace() for character in text)
        length = float(len(text))
        return (letters / length, digits / length, spaces / length)


class GeodesicTranslator:
    """Bounded classical control signal for geodesic attention.

    The frozen bound is always ``Γ = γ_max * tanh(g_theta(state))``.
    ``g_theta`` defaults to the handcrafted Step 2 formula and may optionally be
    a learned controller. Measurements used for discrete state remain detached.
    """

    def __init__(
        self,
        gamma_max: float = 0.1,
        pressure_thresholds: tuple[float, float, float, float] = (0.25, 0.5, 0.8, 1.2),
        controller: Any | None = None,
    ) -> None:
        if torch is None:
            raise ModuleNotFoundError("GeodesicTranslator requires PyTorch")
        if gamma_max < 0:
            raise ValueError("gamma_max must be non-negative")
        self.gamma_max = float(gamma_max)
        self.pressure_thresholds = pressure_thresholds
        self.controller = controller

    def measure(
        self,
        queries: "torch.Tensor",
        keys: "torch.Tensor",
        geodesic_distances: "torch.Tensor",
        previous_queries: "torch.Tensor | None" = None,
    ) -> TranslatorMeasurements:
        if not torch.isfinite(queries).all() or not torch.isfinite(keys).all() or not torch.isfinite(geodesic_distances).all():
            raise ValueError("GeodesicTranslator rejects NaN/Inf inputs")
        geodesic_error = float(geodesic_distances.mean().detach().cpu())
        mirror_error = float((queries.mean(dim=-2) - keys.mean(dim=-2)).norm(dim=-1).mean().detach().cpu())
        centered = geodesic_distances - geodesic_distances.mean(dim=-1, keepdim=True)
        curvature_energy = float((centered.square().mean()).detach().cpu())
        if previous_queries is None:
            state_velocity = 0.0
        else:
            state_velocity = float((queries - previous_queries).norm(dim=-1).mean().detach().cpu())
        key_count = int(geodesic_distances.shape[-1])
        if key_count <= 1:
            uncertainty = 0.0
        else:
            probabilities = torch.softmax(-(geodesic_distances + centered.abs()), dim=-1).clamp_min(1e-12)
            entropy = -(probabilities * probabilities.log()).sum(dim=-1).mean()
            normalizer = torch.log(torch.tensor(float(key_count), device=probabilities.device, dtype=probabilities.dtype))
            # Distance from the uniform / maximum-entropy field. A calm equal-distance
            # neighborhood stays NORMAL instead of inheriting pressure from entropy=1.
            uncertainty = float((1.0 - (entropy / normalizer)).abs().detach().cpu())
        pressure = geodesic_error + mirror_error + curvature_energy + state_velocity + uncertainty
        return TranslatorMeasurements(geodesic_error, mirror_error, curvature_energy, state_velocity, uncertainty, float(pressure))

    def state_for_pressure(self, pressure: float, previous_state: TranslatorState | None = None) -> TranslatorState:
        normal, caution, high, reject = self.pressure_thresholds
        del normal
        if pressure >= reject:
            return TranslatorState.REJECT
        if pressure >= high:
            return TranslatorState.PRESSURE
        if previous_state in {TranslatorState.PRESSURE, TranslatorState.REJECT} and pressure < caution:
            return TranslatorState.RECOVERY
        if pressure >= caution:
            return TranslatorState.CAUTION
        return TranslatorState.NORMAL

    def _raw_state(
        self,
        queries: "torch.Tensor",
        keys: "torch.Tensor",
        geodesic_distances: "torch.Tensor",
        previous_queries: "torch.Tensor | None",
        measurements: TranslatorMeasurements,
        device: "torch.device | None",
        dtype: "torch.dtype | None",
    ) -> "torch.Tensor":
        resolved_dtype = dtype or torch.float32
        if self.controller is None:
            raw = measurements.pressure + 0.5 * measurements.geodesic_error + 0.25 * measurements.mirror_error
            return torch.as_tensor(raw, device=device, dtype=resolved_dtype)

        geo = geodesic_distances.mean()
        mirror = (queries.mean(dim=-2) - keys.mean(dim=-2)).norm(dim=-1).mean()
        centered = geodesic_distances - geodesic_distances.mean(dim=-1, keepdim=True)
        curvature = centered.square().mean()
        if previous_queries is None:
            velocity = torch.zeros((), device=geodesic_distances.device, dtype=geodesic_distances.dtype)
        else:
            velocity = (queries - previous_queries).norm(dim=-1).mean()
        key_count = int(geodesic_distances.shape[-1])
        if key_count <= 1:
            uncertainty = torch.zeros((), device=geodesic_distances.device, dtype=geodesic_distances.dtype)
        else:
            probabilities = torch.softmax(-(geodesic_distances + centered.abs()), dim=-1).clamp_min(1e-12)
            entropy = -(probabilities * probabilities.log()).sum(dim=-1).mean()
            normalized = entropy / torch.log(torch.tensor(float(key_count), device=geodesic_distances.device, dtype=geodesic_distances.dtype))
            uncertainty = (1.0 - normalized).abs()
        pressure = geo + mirror + curvature + velocity + uncertainty
        features = torch.stack([geo.reshape(()), mirror.reshape(()), curvature.reshape(()), velocity.reshape(()), uncertainty.reshape(()), pressure.reshape(())])
        return self.controller(features.to(dtype=resolved_dtype))

    def correction(
        self,
        scores_shape: tuple[int, ...],
        measurements: TranslatorMeasurements,
        device: "torch.device | None" = None,
        dtype: "torch.dtype | None" = None,
        raw: "torch.Tensor | None" = None,
    ) -> "torch.Tensor":
        resolved_dtype = dtype or torch.float32
        if raw is None:
            raw = torch.as_tensor(
                measurements.pressure + 0.5 * measurements.geodesic_error + 0.25 * measurements.mirror_error,
                device=device,
                dtype=resolved_dtype,
            )
        value = self.gamma_max * torch.tanh(raw)
        return torch.zeros(scores_shape, device=device, dtype=resolved_dtype) + value

    def __call__(
        self,
        queries: "torch.Tensor",
        keys: "torch.Tensor",
        geodesic_distances: "torch.Tensor",
        previous_queries: "torch.Tensor | None" = None,
        previous_state: TranslatorState | None = None,
    ) -> TranslatorOutput:
        measurements = self.measure(queries, keys, geodesic_distances, previous_queries)
        state = self.state_for_pressure(measurements.pressure, previous_state)
        raw = self._raw_state(queries, keys, geodesic_distances, previous_queries, measurements, geodesic_distances.device, geodesic_distances.dtype)
        gamma = self.correction(tuple(geodesic_distances.shape), measurements, geodesic_distances.device, geodesic_distances.dtype, raw=raw)
        return TranslatorOutput(state, gamma, measurements)
