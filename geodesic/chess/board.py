"""Chess board state with deterministic legal-move generation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

FILES = "abcdefgh"
RANKS = "12345678"
PIECE_TYPES = {"p", "n", "b", "r", "q", "k"}
PIECE_VALUES = {"p": 1.0, "n": 3.0, "b": 3.0, "r": 5.0, "q": 9.0, "k": 0.0}
PROMOTION_TYPES = ("q", "r", "b", "n")
FEN_PIECES = {
    "p": ("p", "black"),
    "n": ("n", "black"),
    "b": ("b", "black"),
    "r": ("r", "black"),
    "q": ("q", "black"),
    "k": ("k", "black"),
    "P": ("p", "white"),
    "N": ("n", "white"),
    "B": ("b", "white"),
    "R": ("r", "white"),
    "Q": ("q", "white"),
    "K": ("k", "white"),
}
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

    @property
    def fen_symbol(self) -> str:
        return self.symbol


@dataclass(frozen=True)
class ChessMove:
    source: tuple[int, int]
    target: tuple[int, int]
    promotion: str | None = None

    def __post_init__(self) -> None:
        if self.promotion is not None and self.promotion not in PROMOTION_TYPES:
            raise ValueError(f"invalid promotion piece: {self.promotion}")

    @classmethod
    def from_uci(cls, move: str) -> "ChessMove":
        if len(move) < 4:
            raise ValueError("move must contain source and target squares")
        promotion = move[4].lower() if len(move) > 4 else None
        return cls(square_to_index(move[:2]), square_to_index(move[2:4]), promotion)

    def to_uci(self) -> str:
        text = index_to_square(self.source) + index_to_square(self.target)
        return text + self.promotion if self.promotion else text


@dataclass(frozen=True)
class CastlingRights:
    white_king: bool = True
    white_queen: bool = True
    black_king: bool = True
    black_queen: bool = True

    def can(self, color: str, side: str) -> bool:
        if color == "white" and side == "king":
            return self.white_king
        if color == "white" and side == "queen":
            return self.white_queen
        if color == "black" and side == "king":
            return self.black_king
        if color == "black" and side == "queen":
            return self.black_queen
        raise ValueError("side must be king or queen")

    def after_king_move(self, color: str) -> "CastlingRights":
        if color == "white":
            return CastlingRights(False, False, self.black_king, self.black_queen)
        return CastlingRights(self.white_king, self.white_queen, False, False)

    def after_square_change(self, square: tuple[int, int]) -> "CastlingRights":
        white_king, white_queen, black_king, black_queen = self.white_king, self.white_queen, self.black_king, self.black_queen
        if square == (0, 7):
            white_king = False
        elif square == (0, 0):
            white_queen = False
        elif square == (7, 7):
            black_king = False
        elif square == (7, 0):
            black_queen = False
        elif square == (0, 4):
            white_king = False
            white_queen = False
        elif square == (7, 4):
            black_king = False
            black_queen = False
        return CastlingRights(white_king, white_queen, black_king, black_queen)

    def fen(self) -> str:
        text = ""
        if self.white_king:
            text += "K"
        if self.white_queen:
            text += "Q"
        if self.black_king:
            text += "k"
        if self.black_queen:
            text += "q"
        return text or "-"

    @classmethod
    def from_fen(cls, token: str) -> "CastlingRights":
        if token == "-":
            return cls(False, False, False, False)
        return cls("K" in token, "Q" in token, "k" in token, "q" in token)


class ChessBoard:
    """Board state with legal move validation, including check and special moves."""

    def __init__(
        self,
        squares: dict[tuple[int, int], ChessPiece] | None = None,
        side_to_move: str = "white",
        castling: CastlingRights | None = None,
        en_passant: tuple[int, int] | None = None,
        *,
        validate: bool = True,
    ) -> None:
        if side_to_move not in {"white", "black"}:
            raise ValueError("side_to_move must be white or black")
        self.squares = dict(squares or starting_position())
        self.side_to_move = side_to_move
        self.castling = castling if castling is not None else CastlingRights()
        self.en_passant = en_passant
        if validate:
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
        if self.en_passant is not None and not in_bounds(self.en_passant):
            raise ValueError("en passant square out of bounds")
        return True

    def king_square(self, color: str) -> tuple[int, int]:
        for square, piece in self.squares.items():
            if piece.kind == "k" and piece.color == color:
                return square
        raise ValueError(f"missing {color} king")

    def in_check(self, color: str | None = None) -> bool:
        side = color or self.side_to_move
        enemy = "black" if side == "white" else "white"
        return self.is_square_attacked(self.king_square(side), enemy)

    def is_checkmate(self) -> bool:
        return self.in_check() and not self.legal_moves()

    def is_stalemate(self) -> bool:
        return not self.in_check() and not self.legal_moves()

    def is_square_attacked(self, square: tuple[int, int], by_color: str) -> bool:
        row, col = square
        pawn_dir = 1 if by_color == "white" else -1
        for dc in (-1, 1):
            source = (row - pawn_dir, col + dc)
            occupant = self.piece_at(source) if in_bounds(source) else None
            if occupant is not None and occupant.kind == "p" and occupant.color == by_color:
                return True
        for kind in ("n", "k"):
            for target in step_targets(square, DIRECTIONS[kind]):
                occupant = self.piece_at(target)
                if occupant is not None and occupant.kind == kind and occupant.color == by_color:
                    return True
        for kind, deltas in (("b", DIRECTIONS["b"]), ("r", DIRECTIONS["r"])):
            for delta in deltas:
                for target in ray_targets(square, delta):
                    occupant = self.piece_at(target)
                    if occupant is None:
                        continue
                    if occupant.color == by_color and occupant.kind in {kind, "q"}:
                        return True
                    break
        return False

    def legal_moves(self) -> list[ChessMove]:
        legal: list[ChessMove] = []
        for move in self.pseudo_legal_moves():
            next_board = self.apply_unchecked(move)
            if not next_board.in_check(self.side_to_move):
                legal.append(move)
        return legal

    def pseudo_legal_moves(self) -> list[ChessMove]:
        moves: list[ChessMove] = []
        for square, piece in sorted(self.squares.items()):
            if piece.color == self.side_to_move:
                moves.extend(self._piece_moves(square, piece))
        moves.extend(self._castling_moves(self.side_to_move))
        return moves

    def is_legal_move(self, move: ChessMove) -> bool:
        legal = self.legal_moves()
        if move in legal:
            return True
        if move.promotion is None:
            return ChessMove(move.source, move.target, "q") in legal
        return False

    def apply(self, move: ChessMove) -> "ChessBoard":
        resolved = self._resolve_move(move)
        if resolved not in self.legal_moves():
            raise ValueError(f"illegal move: {move.to_uci()}")
        return self.apply_unchecked(resolved)

    def apply_unchecked(self, move: ChessMove) -> "ChessBoard":
        squares = dict(self.squares)
        piece = squares.pop(move.source)
        captured = squares.pop(move.target, None)
        moving = piece
        if piece.kind == "p" and self.en_passant is not None and move.target == self.en_passant and captured is None:
            squares.pop((move.source[0], move.target[1]), None)
        if piece.kind == "p" and move.promotion:
            moving = ChessPiece(move.promotion, piece.color)
        if piece.kind == "k" and abs(move.target[1] - move.source[1]) == 2:
            rook_from = (move.source[0], 7 if move.target[1] == 6 else 0)
            rook_to = (move.source[0], 5 if move.target[1] == 6 else 3)
            rook = squares.pop(rook_from)
            squares[rook_to] = rook
        squares[move.target] = moving
        castling = self.castling.after_square_change(move.source).after_square_change(move.target)
        if piece.kind == "k":
            castling = castling.after_king_move(piece.color)
        en_passant = None
        if piece.kind == "p" and abs(move.target[0] - move.source[0]) == 2:
            en_passant = ((move.source[0] + move.target[0]) // 2, move.source[1])
        return ChessBoard(squares, "black" if self.side_to_move == "white" else "white", castling, en_passant, validate=False)

    def _resolve_move(self, move: ChessMove) -> ChessMove:
        piece = self.piece_at(move.source)
        if piece is not None and piece.kind == "p" and move.promotion is None:
            last_rank = 7 if piece.color == "white" else 0
            if move.target[0] == last_rank:
                return ChessMove(move.source, move.target, "q")
        return move

    def _piece_moves(self, square: tuple[int, int], piece: ChessPiece) -> list[ChessMove]:
        if piece.kind == "p":
            return expand_promotions(pawn_moves(self, square, piece), piece)
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

    def _castling_moves(self, color: str) -> list[ChessMove]:
        row = 0 if color == "white" else 7
        king_square = (row, 4)
        king = self.piece_at(king_square)
        if king != ChessPiece("k", color) or self.in_check(color):
            return []
        enemy = "black" if color == "white" else "white"
        moves: list[ChessMove] = []
        if self.castling.can(color, "king") and self.piece_at((row, 7)) == ChessPiece("r", color):
            if all(self.piece_at((row, col)) is None for col in (5, 6)) and not any(self.is_square_attacked((row, col), enemy) for col in (5, 6)):
                moves.append(ChessMove(king_square, (row, 6)))
        if self.castling.can(color, "queen") and self.piece_at((row, 0)) == ChessPiece("r", color):
            if all(self.piece_at((row, col)) is None for col in (1, 2, 3)) and not any(self.is_square_attacked((row, col), enemy) for col in (2, 3)):
                moves.append(ChessMove(king_square, (row, 2)))
        return moves

    def to_fen(self) -> str:
        ranks = []
        for row in range(7, -1, -1):
            empty = 0
            token = ""
            for col in range(8):
                piece = self.piece_at((row, col))
                if piece is None:
                    empty += 1
                    continue
                if empty:
                    token += str(empty)
                    empty = 0
                token += piece.fen_symbol
            if empty:
                token += str(empty)
            ranks.append(token)
        side = "w" if self.side_to_move == "white" else "b"
        ep = index_to_square(self.en_passant) if self.en_passant is not None else "-"
        return f"{'/'.join(ranks)} {side} {self.castling.fen()} {ep} 0 1"

    @classmethod
    def from_fen(cls, fen: str) -> "ChessBoard":
        parts = fen.split()
        if len(parts) < 4:
            raise ValueError("FEN must contain placement, side, castling, and en passant fields")
        placement, side_token, castling_token, ep_token = parts[:4]
        ranks = placement.split("/")
        if len(ranks) != 8:
            raise ValueError("FEN placement must contain 8 ranks")
        squares: dict[tuple[int, int], ChessPiece] = {}
        for fen_row, token in enumerate(ranks):
            row = 7 - fen_row
            col = 0
            for char in token:
                if char.isdigit():
                    col += int(char)
                    continue
                if char not in FEN_PIECES:
                    raise ValueError(f"invalid FEN piece: {char}")
                kind, color = FEN_PIECES[char]
                squares[(row, col)] = ChessPiece(kind, color)
                col += 1
            if col != 8:
                raise ValueError("FEN rank does not cover 8 files")
        side = "white" if side_token == "w" else "black"
        if side_token not in {"w", "b"}:
            raise ValueError("FEN side must be w or b")
        en_passant = None if ep_token == "-" else square_to_index(ep_token)
        return cls(squares, side, CastlingRights.from_fen(castling_token), en_passant)

    def ascii(self) -> str:
        rows = []
        for row in range(7, -1, -1):
            cells = []
            for col in range(8):
                piece = self.piece_at((row, col))
                cells.append(piece.symbol if piece else ".")
            rows.append(" ".join(cells))
        return "\n".join(rows)


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
    occupant = board.piece_at(square)
    return in_bounds(square) and (occupant is None or occupant.color != color)


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
    moves: list[ChessMove] = []
    one = (row + direction, col)
    if in_bounds(one) and board.piece_at(one) is None:
        moves.append(ChessMove(square, one))
        two = (row + 2 * direction, col)
        if row == start_row and board.piece_at(two) is None:
            moves.append(ChessMove(square, two))
    for target in [(row + direction, col - 1), (row + direction, col + 1)]:
        if not in_bounds(target):
            continue
        occupant = board.piece_at(target)
        if occupant is not None and occupant.color != piece.color:
            moves.append(ChessMove(square, target))
        elif board.en_passant is not None and target == board.en_passant:
            moves.append(ChessMove(square, target))
    return moves


def expand_promotions(moves: Iterable[ChessMove], piece: ChessPiece) -> list[ChessMove]:
    last_rank = 7 if piece.color == "white" else 0
    expanded: list[ChessMove] = []
    for move in moves:
        if move.target[0] == last_rank:
            expanded.extend(ChessMove(move.source, move.target, promotion) for promotion in PROMOTION_TYPES)
        else:
            expanded.append(move)
    return expanded


def starting_position() -> dict[tuple[int, int], ChessPiece]:
    pieces: dict[tuple[int, int], ChessPiece] = {}
    back = ["r", "n", "b", "q", "k", "b", "n", "r"]
    for col, kind in enumerate(back):
        pieces[(0, col)] = ChessPiece(kind, "white")
        pieces[(1, col)] = ChessPiece("p", "white")
        pieces[(6, col)] = ChessPiece("p", "black")
        pieces[(7, col)] = ChessPiece(kind, "black")
    return pieces
