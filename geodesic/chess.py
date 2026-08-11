"""Classical chess projection layer for geodesic hypothesis generation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from time import perf_counter
from typing import Iterable

from geodesic.core.metric import Metric
from geodesic.translator import TranslatorState

FILES = "abcdefgh"
RANKS = "12345678"
PIECE_TYPES = {"p", "n", "b", "r", "q", "k"}
PIECE_VALUES = {"p": 1.0, "n": 3.0, "b": 3.0, "r": 5.0, "q": 9.0, "k": 0.0}
DIRECTIONS = {
    "n": [(-2, -1), (-2, 1), (-1, -2), (-1, 2), (1, -2), (1, 2), (2, -1), (2, 1)],
    "k": [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)],
    "b": [(-1, -1), (-1, 1), (1, -1), (1, 1)],
    "r": [(-1, 0), (1, 0), (0, -1), (0, 1)],
    "q": [(-1, -1), (-1, 1), (1, -1), (1, 1), (-1, 0), (1, 0), (0, -1), (0, 1)],
}


@dataclass(frozen=True)
class ChessPiece:
    kind: str
    color: str

    def __post_init__(self) -> None:
        if self.kind not in PIECE_TYPES:
            raise ValueError(f"unknown chess piece kind: {self.kind}")
        if self.color not in {"white", "black"}:
            raise ValueError(f"unknown chess color: {self.color}")

    @property
    def symbol(self) -> str:
        return self.kind.upper() if self.color == "white" else self.kind


@dataclass(frozen=True)
class ChessMove:
    source: tuple[int, int]
    target: tuple[int, int]

    @classmethod
    def from_uci(cls, move: str) -> "ChessMove":
        if len(move) < 4:
            raise ValueError("move must contain source and target squares")
        return cls(square_to_index(move[:2]), square_to_index(move[2:4]))

    def to_uci(self) -> str:
        return index_to_square(self.source) + index_to_square(self.target)


@dataclass(frozen=True)
class ProjectionChannels:
    control: float
    threat: float
    mobility: float
    stability: float


@dataclass(frozen=True)
class CandidateHypothesis:
    move: str
    board: "ChessBoard"
    legality: bool
    geodesic_error: float
    pressure: float
    channels: ProjectionChannels
    overall_score: float


class ChessBoard:
    """Minimal board state with deterministic pseudo-legal move validation."""

    def __init__(self, squares: dict[tuple[int, int], ChessPiece] | None = None, side_to_move: str = "white") -> None:
        if side_to_move not in {"white", "black"}:
            raise ValueError("side_to_move must be white or black")
        self.squares = dict(squares or starting_position())
        self.side_to_move = side_to_move
        self.validate_state()

    def piece_at(self, square: tuple[int, int]) -> ChessPiece | None:
        return self.squares.get(square)

    def encode(self, d_model: int = 12) -> list[list[list[float]]]:
        if d_model < 12:
            raise ValueError("d_model must be at least 12 for chess projection encoding")
        encoded = [[[0.0 for _ in range(d_model)] for _ in range(8)] for _ in range(8)]
        for row in range(8):
            for col in range(8):
                piece = self.piece_at((row, col))
                vector = encoded[row][col]
                if piece is not None:
                    vector[0] = 1.0
                    vector[1 + sorted(PIECE_TYPES).index(piece.kind)] = 1.0
                    vector[7] = 1.0 if piece.color == "white" else -1.0
                vector[8] = row / 7.0
                vector[9] = col / 7.0
                vector[10] = 1.0 if self.side_to_move == "white" else -1.0
                vector[11] = math.sqrt((row / 7.0) ** 2 + (col / 7.0) ** 2)
        return encoded

    @classmethod
    def decode(cls, encoded: list[list[list[float]]], side_to_move: str = "white") -> "ChessBoard":
        if len(encoded) != 8 or any(len(row) != 8 for row in encoded):
            raise ValueError("encoded board must have shape 8x8xd")
        kinds = sorted(PIECE_TYPES)
        squares: dict[tuple[int, int], ChessPiece] = {}
        for row in range(8):
            for col in range(8):
                vector = encoded[row][col]
                if len(vector) < 12:
                    raise ValueError("encoded square vector must have at least 12 features")
                if vector[0] >= 0.5:
                    kind_index = max(range(len(kinds)), key=lambda index: vector[1 + index])
                    color = "white" if vector[7] >= 0 else "black"
                    squares[(row, col)] = ChessPiece(kinds[kind_index], color)
        return cls(squares, side_to_move)

    def validate_state(self) -> bool:
        kings = {"white": 0, "black": 0}
        for (row, col), piece in self.squares.items():
            if not in_bounds((row, col)):
                raise ValueError("piece outside board")
            if piece.kind == "k":
                kings[piece.color] += 1
        if kings != {"white": 1, "black": 1}:
            raise ValueError("board must contain exactly one king per side")
        return True

    def legal_moves(self) -> list[ChessMove]:
        moves: list[ChessMove] = []
        for square, piece in sorted(self.squares.items()):
            if piece.color == self.side_to_move:
                moves.extend(self._piece_moves(square, piece))
        return moves

    def is_legal_move(self, move: ChessMove) -> bool:
        return any(candidate == move for candidate in self.legal_moves())

    def apply(self, move: ChessMove) -> "ChessBoard":
        if not self.is_legal_move(move):
            raise ValueError(f"illegal move: {move.to_uci()}")
        squares = dict(self.squares)
        piece = squares.pop(move.source)
        squares[move.target] = piece
        return ChessBoard(squares, "black" if self.side_to_move == "white" else "white")

    def _piece_moves(self, square: tuple[int, int], piece: ChessPiece) -> list[ChessMove]:
        if piece.kind == "p":
            return pawn_moves(self, square, piece)
        if piece.kind in {"n", "k"}:
            return [ChessMove(square, target) for target in step_targets(square, DIRECTIONS[piece.kind]) if can_occupy(self, target, piece.color)]
        moves: list[ChessMove] = []
        for direction in DIRECTIONS[piece.kind]:
            for target in ray_targets(square, direction):
                occupant = self.piece_at(target)
                if occupant is None:
                    moves.append(ChessMove(square, target))
                    continue
                if occupant.color != piece.color:
                    moves.append(ChessMove(square, target))
                break
        return moves


def square_to_index(square: str) -> tuple[int, int]:
    if len(square) != 2 or square[0] not in FILES or square[1] not in RANKS:
        raise ValueError(f"malformed square: {square}")
    return int(square[1]) - 1, FILES.index(square[0])


def index_to_square(index: tuple[int, int]) -> str:
    row, col = index
    if not in_bounds(index):
        raise ValueError(f"square index out of bounds: {index}")
    return FILES[col] + RANKS[row]


def in_bounds(square: tuple[int, int]) -> bool:
    row, col = square
    return 0 <= row < 8 and 0 <= col < 8


def can_occupy(board: ChessBoard, square: tuple[int, int], color: str) -> bool:
    return in_bounds(square) and (board.piece_at(square) is None or board.piece_at(square).color != color)


def step_targets(square: tuple[int, int], deltas: Iterable[tuple[int, int]]) -> list[tuple[int, int]]:
    row, col = square
    return [(row + dr, col + dc) for dr, dc in deltas if in_bounds((row + dr, col + dc))]


def ray_targets(square: tuple[int, int], delta: tuple[int, int]) -> list[tuple[int, int]]:
    row, col = square
    dr, dc = delta
    targets = []
    row += dr
    col += dc
    while in_bounds((row, col)):
        targets.append((row, col))
        row += dr
        col += dc
    return targets


def pawn_moves(board: ChessBoard, square: tuple[int, int], piece: ChessPiece) -> list[ChessMove]:
    row, col = square
    direction = 1 if piece.color == "white" else -1
    start_row = 1 if piece.color == "white" else 6
    moves = []
    one = (row + direction, col)
    if in_bounds(one) and board.piece_at(one) is None:
        moves.append(ChessMove(square, one))
        two = (row + 2 * direction, col)
        if row == start_row and board.piece_at(two) is None:
            moves.append(ChessMove(square, two))
    for target in [(row + direction, col - 1), (row + direction, col + 1)]:
        occupant = board.piece_at(target) if in_bounds(target) else None
        if occupant is not None and occupant.color != piece.color:
            moves.append(ChessMove(square, target))
    return moves


def starting_position() -> dict[tuple[int, int], ChessPiece]:
    pieces: dict[tuple[int, int], ChessPiece] = {}
    back = ["r", "n", "b", "q", "k", "b", "n", "r"]
    for col, kind in enumerate(back):
        pieces[(0, col)] = ChessPiece(kind, "white")
        pieces[(1, col)] = ChessPiece("p", "white")
        pieces[(6, col)] = ChessPiece("p", "black")
        pieces[(7, col)] = ChessPiece(kind, "black")
    return pieces


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
        material = sum(PIECE_VALUES[p.kind] for p in own) - sum(PIECE_VALUES[p.kind] for p in enemy)
        return ProjectionChannels(control=len(legal) / 64.0, threat=sum(PIECE_VALUES[board.piece_at(move.target).kind] for move in captures if board.piece_at(move.target)) / 39.0, mobility=len({move.target for move in legal}) / 64.0, stability=1.0 / (1.0 + math.exp(-material / 10.0)))

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
            candidates.append(CandidateHypothesis(move.to_uci(), next_board, True, error, pressure, channels, self.score(channels, error, pressure)))
        candidates.sort(key=lambda candidate: (candidate.overall_score, candidate.move), reverse=True)
        return candidates[:limit]


def benchmark_chess_projection() -> dict[str, object]:
    start = perf_counter()
    board = ChessBoard()
    projection = ChessProjection()
    ablations = ["A_transformer", "B_transformer_chess", "C_transformer_geodesic_chess", "D_transformer_geodesic_translator_chess"]
    candidates = projection.top_k(board)
    latency = perf_counter() - start
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
        "top_k": [
            {
                "move": candidate.move,
                "geodesic_error": candidate.geodesic_error,
                "pressure": candidate.pressure,
                "control": candidate.channels.control,
                "threat": candidate.channels.threat,
                "mobility": candidate.channels.mobility,
                "stability": candidate.channels.stability,
                "overall_score": candidate.overall_score,
            }
            for candidate in candidates
        ],
        "translator_state": TranslatorState.NORMAL.value,
    }
