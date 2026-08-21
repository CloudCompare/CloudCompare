"""Logical and Transformer-backed chess projection."""

from __future__ import annotations

import importlib.util
import math
from dataclasses import dataclass
from time import perf_counter
from typing import Any

from geodesic.chess.board import PIECE_VALUES, ChessBoard, ChessMove
from geodesic.core.metric import Metric
from geodesic.translator import TranslatorState

if importlib.util.find_spec("torch") is None:  # pragma: no cover - environment dependent
    torch = None  # type: ignore[assignment]
else:
    import torch


@dataclass(frozen=True)
class ProjectionChannels:
    control: float
    threat: float
    mobility: float
    stability: float


@dataclass(frozen=True)
class CandidateHypothesis:
    move: str
    board: ChessBoard
    legality: bool
    geodesic_error: float
    pressure: float
    channels: ProjectionChannels
    overall_score: float
    transformer_quality: float = 0.0
    ablation: str = "logical"


class ChessProjection:
    """Logical projection, consistency filter, and transparent scorer."""

    def __init__(self, metric: Metric | None = None, gamma_max: float = 0.1, top_k: int = 3) -> None:
        if gamma_max < 0:
            raise ValueError("gamma_max must be non-negative")
        self.metric = metric or Metric()
        self.gamma_max = gamma_max
        self.top_k_default = top_k

    def project_channels(self, board: ChessBoard) -> ProjectionChannels:
        legal = board.legal_moves()
        own = [piece for piece in board.squares.values() if piece.color == board.side_to_move]
        enemy = [piece for piece in board.squares.values() if piece.color != board.side_to_move]
        captures = [move for move in legal if board.piece_at(move.target) is not None]
        material = sum(PIECE_VALUES[piece.kind] for piece in own) - sum(PIECE_VALUES[piece.kind] for piece in enemy)
        return ProjectionChannels(
            control=len(legal) / 64.0,
            threat=sum(PIECE_VALUES[board.piece_at(move.target).kind] for move in captures if board.piece_at(move.target)) / 39.0,
            mobility=len({move.target for move in legal}) / 64.0,
            stability=1.0 / (1.0 + math.exp(-material / 10.0)),
        )

    def geodesic_error(self, move: ChessMove) -> float:
        source = (move.source[0] / 7.0, move.source[1] / 7.0)
        target = (move.target[0] / 7.0, move.target[1] / 7.0)
        return self.metric.distance(source, target)

    def pressure(self, channels: ProjectionChannels, geodesic_error: float) -> float:
        raw = geodesic_error + channels.threat + (1.0 - channels.stability)
        return self.gamma_max * math.tanh(raw)

    def score(self, channels: ProjectionChannels, geodesic_error: float, pressure: float) -> float:
        return channels.control + channels.threat + channels.mobility + channels.stability - geodesic_error - pressure

    def top_k(self, board: ChessBoard, k: int | None = None) -> list[CandidateHypothesis]:
        limit = k or self.top_k_default
        candidates = []
        for move in board.legal_moves():
            next_board = board.apply(move)
            channels = self.project_channels(next_board)
            error = self.geodesic_error(move)
            pressure = self.pressure(channels, error)
            candidates.append(
                CandidateHypothesis(
                    move.to_uci(),
                    next_board,
                    True,
                    error,
                    pressure,
                    channels,
                    self.score(channels, error, pressure),
                )
            )
        candidates.sort(key=lambda candidate: (candidate.overall_score, candidate.move), reverse=True)
        return candidates[:limit]


def board_to_tensors(board: ChessBoard, d_model: int) -> tuple["torch.Tensor", "torch.Tensor"]:
    if torch is None:
        raise ModuleNotFoundError("board_to_tensors requires PyTorch")
    encoded = board.encode(d_model)
    tokens = []
    coordinates = []
    for row in range(8):
        for col in range(8):
            tokens.append(encoded[row][col])
            coordinates.append([row / 7.0, col / 7.0])
    return torch.tensor([tokens], dtype=torch.float32), torch.tensor([coordinates], dtype=torch.float32)


def combine_scores(
    ablation: str,
    logical: float,
    kernel_error: float,
    transformer_error: float,
    pressure: float,
    quality: float,
) -> float:
    if ablation == "A_transformer":
        return quality - transformer_error
    if ablation == "B_transformer_chess":
        return logical + quality
    if ablation == "C_transformer_geodesic_chess":
        return logical - kernel_error - transformer_error
    if ablation == "D_transformer_geodesic_translator_chess":
        return logical - kernel_error - transformer_error - pressure + 0.25 * quality
    return logical


class TransformerChessProjection:
    """Score legal chess hypotheses with actual Transformer traces."""

    def __init__(self, projection: ChessProjection, transformer: Any, ablation: str = "D_transformer_geodesic_translator_chess") -> None:
        if torch is None:
            raise ModuleNotFoundError("TransformerChessProjection requires PyTorch")
        self.projection = projection
        self.transformer = transformer
        self.ablation = ablation

    def _signals(self, boards: list[ChessBoard]) -> tuple[list[float], list[float], list[float]]:
        if not boards:
            return [], [], []
        tokens = []
        coordinates = []
        for board in boards:
            token, coord = board_to_tensors(board, self.transformer.config.d_model)
            tokens.append(token)
            coordinates.append(coord)
        x = torch.cat(tokens, dim=0)
        coords = torch.cat(coordinates, dim=0)
        self.transformer.eval()
        with torch.no_grad():
            logits, traces = self.transformer(x, coords)
        quality = torch.softmax(logits, dim=-1)[:, 1]
        last = traces[-1]
        transformer_error = last.geodesic_distances.mean(dim=(1, 2, 3))
        pressures = self.transformer.config.gamma_max * torch.tanh(transformer_error)
        return quality.tolist(), transformer_error.tolist(), pressures.tolist()

    def top_k(self, board: ChessBoard, k: int | None = None) -> list[CandidateHypothesis]:
        limit = k or self.projection.top_k_default
        legal = board.legal_moves()
        next_boards = [board.apply(move) for move in legal]
        qualities, transformer_errors, pressures = self._signals(next_boards)
        candidates = []
        for move, next_board, quality, transformer_error, transformer_pressure in zip(
            legal, next_boards, qualities, transformer_errors, pressures, strict=True
        ):
            channels = self.projection.project_channels(next_board)
            kernel_error = self.projection.geodesic_error(move)
            logical = self.projection.score(channels, kernel_error, self.projection.pressure(channels, kernel_error))
            pressure = transformer_pressure if "translator" in self.ablation else self.projection.pressure(channels, kernel_error)
            overall = combine_scores(self.ablation, logical, kernel_error, transformer_error, pressure, quality)
            candidates.append(
                CandidateHypothesis(
                    move.to_uci(),
                    next_board,
                    True,
                    kernel_error,
                    pressure,
                    channels,
                    overall,
                    transformer_quality=quality,
                    ablation=self.ablation,
                )
            )
        candidates.sort(key=lambda candidate: (candidate.overall_score, candidate.move), reverse=True)
        return candidates[:limit]


def _blocked_payload(candidates: list[CandidateHypothesis], latency: float) -> dict[str, object]:
    ablations = ["A_transformer", "B_transformer_chess", "C_transformer_geodesic_chess", "D_transformer_geodesic_translator_chess"]
    return {
        "status": "VALIDATION_BLOCKED",
        "blocked_reason": "PyTorch is unavailable; Step 4 Transformer-backed ablation cannot fully execute.",
        "ablations": [
            {
                "name": name,
                "candidate_validity": all(candidate.legality for candidate in candidates),
                "hypothesis_diversity": len({candidate.move for candidate in candidates}),
                "geodesic_error": sum(candidate.geodesic_error for candidate in candidates) / len(candidates),
                "pressure": sum(candidate.pressure for candidate in candidates) / len(candidates),
                "stability": sum(candidate.channels.stability for candidate in candidates) / len(candidates),
                "inference_latency_seconds": latency,
                "parameter_count": None,
            }
            for name in ablations
        ],
        "top_k": [_candidate_row(candidate) for candidate in candidates],
        "translator_state": TranslatorState.NORMAL.value,
    }


def _candidate_row(candidate: CandidateHypothesis) -> dict[str, object]:
    return {
        "move": candidate.move,
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


def benchmark_chess_projection() -> dict[str, object]:
    start = perf_counter()
    board = ChessBoard()
    projection = ChessProjection()
    if torch is None:
        candidates = projection.top_k(board)
        return _blocked_payload(candidates, perf_counter() - start)

    from geodesic.transformer import GeodesicTransformer, GeodesicTransformerConfig, count_parameters

    torch.manual_seed(42)
    configs = {
        "A_transformer": GeodesicTransformerConfig(d_model=16, num_heads=4, num_layers=1, lambda_geodesic=0.0, use_translator=False, use_point_attention=False),
        "B_transformer_chess": GeodesicTransformerConfig(d_model=16, num_heads=4, num_layers=1, lambda_geodesic=0.0, use_translator=False, use_point_attention=False),
        "C_transformer_geodesic_chess": GeodesicTransformerConfig(d_model=16, num_heads=4, num_layers=1, lambda_geodesic=0.25, use_translator=False, use_point_attention=False),
        "D_transformer_geodesic_translator_chess": GeodesicTransformerConfig(
            d_model=16, num_heads=4, num_layers=1, lambda_geodesic=0.25, use_translator=True, use_point_attention=True
        ),
    }
    rows = []
    displayed: list[CandidateHypothesis] = []
    finite = True
    for name, config in configs.items():
        model = GeodesicTransformer(config)
        wired = TransformerChessProjection(projection, model, ablation=name)
        ablation_start = perf_counter()
        candidates = wired.top_k(board)
        latency = perf_counter() - ablation_start
        values = [candidate.overall_score for candidate in candidates]
        finite = finite and all(math.isfinite(value) for value in values)
        rows.append(
            {
                "name": name,
                "candidate_validity": all(candidate.legality for candidate in candidates),
                "hypothesis_diversity": len({candidate.move for candidate in candidates}),
                "geodesic_error": sum(candidate.geodesic_error for candidate in candidates) / len(candidates),
                "pressure": sum(candidate.pressure for candidate in candidates) / len(candidates),
                "stability": sum(candidate.channels.stability for candidate in candidates) / len(candidates),
                "inference_latency_seconds": latency,
                "parameter_count": count_parameters(model),
            }
        )
        if name == "D_transformer_geodesic_translator_chess":
            displayed = candidates
    return {
        "status": "PASS" if finite and all(row["candidate_validity"] for row in rows) else "FAIL",
        "blocked_reason": None,
        "ablations": rows,
        "top_k": [_candidate_row(candidate) for candidate in displayed],
        "translator_state": TranslatorState.NORMAL.value,
        "wired_to_transformer": True,
    }
