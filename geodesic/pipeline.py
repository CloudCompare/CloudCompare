"""End-to-end Geodesic path: Kernel → Attention → Translator → Transformer → Chess."""

from __future__ import annotations

import importlib.util
from dataclasses import asdict, dataclass, field
from typing import Any

from geodesic.chess import CandidateHypothesis, ChessBoard, ChessProjection, TransformerChessProjection
from geodesic.core.manifold import Manifold
from geodesic.core.measurements import distortion, path_length
from geodesic.core.metric import Metric

if importlib.util.find_spec("torch") is None:  # pragma: no cover - environment dependent
    torch = None  # type: ignore[assignment]
else:
    import torch


@dataclass(frozen=True)
class KernelReport:
    path_length: float
    distortion: float
    metric: str
    anchors: list[str]


@dataclass(frozen=True)
class PipelineResult:
    status: str
    blocked_reason: str | None
    fen: str
    kernel: KernelReport
    candidates: list[dict[str, Any]]
    translator_state: str | None
    transformer: dict[str, Any] | None = None
    extras: dict[str, Any] = field(default_factory=dict)


class GeodesicPipeline:
    """Executable graph connecting the previously separate geodesic components."""

    def __init__(self, metric: str = "euclidean", k: int = 3, seed: int = 42) -> None:
        self.metric_name = metric
        self.metric = Metric.from_name(metric)
        self.k = k
        self.seed = seed
        self.projection = ChessProjection(metric=self.metric, top_k=k)
        self.transformer = None
        if torch is not None:
            from geodesic.transformer import GeodesicTransformer, GeodesicTransformerConfig

            torch.manual_seed(seed)
            config = GeodesicTransformerConfig(
                d_model=16,
                num_heads=4,
                num_layers=1,
                lambda_geodesic=0.25,
                use_translator=True,
                use_point_attention=True,
                geodesic_metric=metric,
            )
            self.transformer = GeodesicTransformer(config)

    def kernel_path(self, points: list[tuple[str, tuple[float, ...]]]) -> KernelReport:
        if not points:
            raise ValueError("kernel path requires at least one named point")
        manifold = Manifold(dimensions=len(points[0][1]), metric=self.metric)
        names = []
        for name, coordinates in points:
            manifold.add_point(name, coordinates)
            names.append(name)
        return KernelReport(
            path_length=path_length(manifold, names),
            distortion=distortion(manifold, names),
            metric=self.metric.name,
            anchors=names,
        )

    def analyze(self, board: ChessBoard | None = None) -> PipelineResult:
        board = board or ChessBoard()
        legal = board.legal_moves()
        points = [("start", (0.0, 0.0))]
        if legal:
            first = legal[0]
            points.append(("move", (first.target[0] / 7.0, first.target[1] / 7.0)))
        kernel = self.kernel_path(points)
        if self.transformer is None:
            candidates = self.projection.top_k(board, self.k)
            return PipelineResult(
                status="VALIDATION_BLOCKED",
                blocked_reason="PyTorch is unavailable; Transformer stage cannot execute.",
                fen=board.to_fen(),
                kernel=kernel,
                candidates=[_serialize_candidate(candidate) for candidate in candidates],
                translator_state="NORMAL",
            )

        wired = TransformerChessProjection(self.projection, self.transformer)
        candidates = wired.top_k(board, self.k)
        from geodesic.transformer import count_parameters

        return PipelineResult(
            status="PASS",
            blocked_reason=None,
            fen=board.to_fen(),
            kernel=kernel,
            candidates=[_serialize_candidate(candidate) for candidate in candidates],
            translator_state="NORMAL",
            transformer={
                "d_model": self.transformer.config.d_model,
                "num_heads": self.transformer.config.num_heads,
                "lambda_geodesic": self.transformer.config.lambda_geodesic,
                "geodesic_metric": self.transformer.config.geodesic_metric,
                "use_translator": self.transformer.config.use_translator,
                "use_point_attention": self.transformer.config.use_point_attention,
                "parameter_count": count_parameters(self.transformer),
            },
        )

    def as_dict(self, board: ChessBoard | None = None) -> dict[str, Any]:
        result = self.analyze(board)
        payload = asdict(result)
        payload["kernel"] = asdict(result.kernel)
        return payload


def _serialize_candidate(candidate: CandidateHypothesis) -> dict[str, Any]:
    return {
        "move": candidate.move,
        "fen": candidate.board.to_fen(),
        "legality": candidate.legality,
        "geodesic_error": candidate.geodesic_error,
        "pressure": candidate.pressure,
        "control": candidate.channels.control,
        "threat": candidate.channels.threat,
        "mobility": candidate.channels.mobility,
        "stability": candidate.channels.stability,
        "overall_score": candidate.overall_score,
        "transformer_quality": candidate.transformer_quality,
        "ablation": candidate.ablation,
    }
