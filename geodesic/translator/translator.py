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
    """Bounded classical control signal for geodesic attention."""

    def __init__(self, gamma_max: float = 0.1, pressure_thresholds: tuple[float, float, float, float] = (0.25, 0.5, 0.8, 1.2)) -> None:
        if torch is None:
            raise ModuleNotFoundError("GeodesicTranslator requires PyTorch")
        if gamma_max < 0:
            raise ValueError("gamma_max must be non-negative")
        self.gamma_max = float(gamma_max)
        self.pressure_thresholds = pressure_thresholds

    def measure(self, queries: "torch.Tensor", keys: "torch.Tensor", geodesic_distances: "torch.Tensor", previous_queries: "torch.Tensor | None" = None) -> TranslatorMeasurements:
        geodesic_error = float(geodesic_distances.mean().detach().cpu())
        mirror_error = float((queries.mean(dim=-2) - keys.mean(dim=-2)).norm(dim=-1).mean().detach().cpu())
        centered = geodesic_distances - geodesic_distances.mean(dim=-1, keepdim=True)
        curvature_energy = float((centered.square().mean()).detach().cpu())
        if previous_queries is None:
            state_velocity = 0.0
        else:
            state_velocity = float((queries - previous_queries).norm(dim=-1).mean().detach().cpu())
        probabilities = torch.softmax(-(geodesic_distances + centered.abs()), dim=-1).clamp_min(1e-12)
        entropy = -(probabilities * probabilities.log()).sum(dim=-1).mean()
        uncertainty = float((entropy / torch.log(torch.tensor(float(probabilities.shape[-1]), device=probabilities.device))).detach().cpu())
        pressure = geodesic_error + mirror_error + curvature_energy + state_velocity + uncertainty
        return TranslatorMeasurements(geodesic_error, mirror_error, curvature_energy, state_velocity, uncertainty, float(pressure))

    def state_for_pressure(self, pressure: float, previous_state: TranslatorState | None = None) -> TranslatorState:
        normal, caution, high, reject = self.pressure_thresholds
        if pressure >= reject:
            return TranslatorState.REJECT
        if pressure >= high:
            return TranslatorState.PRESSURE
        if previous_state in {TranslatorState.PRESSURE, TranslatorState.REJECT} and pressure < caution:
            return TranslatorState.RECOVERY
        if pressure >= caution:
            return TranslatorState.CAUTION
        return TranslatorState.NORMAL

    def correction(self, scores_shape: tuple[int, ...], measurements: TranslatorMeasurements, device: "torch.device | None" = None, dtype: "torch.dtype | None" = None) -> "torch.Tensor":
        raw = measurements.pressure + 0.5 * measurements.geodesic_error + 0.25 * measurements.mirror_error
        value = self.gamma_max * torch.tanh(torch.tensor(raw, device=device, dtype=dtype or torch.float32))
        return torch.full(scores_shape, value.item(), device=device, dtype=dtype or torch.float32)

    def __call__(self, queries: "torch.Tensor", keys: "torch.Tensor", geodesic_distances: "torch.Tensor", previous_queries: "torch.Tensor | None" = None, previous_state: TranslatorState | None = None) -> TranslatorOutput:
        measurements = self.measure(queries, keys, geodesic_distances, previous_queries)
        state = self.state_for_pressure(measurements.pressure, previous_state)
        gamma = self.correction(tuple(geodesic_distances.shape), measurements, geodesic_distances.device, geodesic_distances.dtype)
        return TranslatorOutput(state, gamma, measurements)
