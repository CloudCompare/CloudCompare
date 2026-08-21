"""Chess projection package for geodesic hypothesis generation."""

from geodesic.chess.board import CastlingRights, ChessBoard, ChessMove, ChessPiece, index_to_square, square_to_index
from geodesic.chess.projection import CandidateHypothesis, ChessProjection, ProjectionChannels, TransformerChessProjection, benchmark_chess_projection, board_to_tensors

__all__ = [
    "CastlingRights",
    "ChessBoard",
    "ChessMove",
    "ChessPiece",
    "CandidateHypothesis",
    "ChessProjection",
    "ProjectionChannels",
    "TransformerChessProjection",
    "benchmark_chess_projection",
    "board_to_tensors",
    "index_to_square",
    "square_to_index",
]
