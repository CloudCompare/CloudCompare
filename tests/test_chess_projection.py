import json
import math

import pytest

from geodesic.chess import ChessBoard, ChessMove, ChessPiece, ChessProjection, square_to_index
from experiments.chess_projection_v01 import run


def test_board_encode_decode_consistency():
    board = ChessBoard()
    decoded = ChessBoard.decode(board.encode(), board.side_to_move)
    assert {square: piece.symbol for square, piece in decoded.squares.items()} == {square: piece.symbol for square, piece in board.squares.items()}


def test_legal_move_validation():
    board = ChessBoard()
    assert board.is_legal_move(ChessMove.from_uci("e2e4"))


def test_illegal_move_rejection():
    board = ChessBoard()
    with pytest.raises(ValueError):
        board.apply(ChessMove.from_uci("e2e5"))


def test_geodesic_consistency():
    projection = ChessProjection()
    move = ChessMove.from_uci("e2e4")
    assert math.isclose(projection.geodesic_error(move), 2 / 7)


def test_top_k_diversity():
    candidates = ChessProjection(top_k=3).top_k(ChessBoard())
    assert len(candidates) == 3
    assert len({candidate.move for candidate in candidates}) == 3


def test_translator_pressure_response():
    projection = ChessProjection(gamma_max=0.1)
    channels = projection.project_channels(ChessBoard())
    low = projection.pressure(channels, 0.1)
    high = projection.pressure(channels, 1.0)
    assert 0 <= low <= projection.gamma_max
    assert 0 <= high <= projection.gamma_max
    assert high > low


def test_candidate_stability_and_no_nan_inf():
    candidates = ChessProjection().top_k(ChessBoard())
    for candidate in candidates:
        values = [candidate.geodesic_error, candidate.pressure, candidate.channels.control, candidate.channels.threat, candidate.channels.mobility, candidate.channels.stability, candidate.overall_score]
        assert all(math.isfinite(value) for value in values)
        assert candidate.legality


def test_deterministic_replay():
    first = [candidate.move for candidate in ChessProjection().top_k(ChessBoard())]
    second = [candidate.move for candidate in ChessProjection().top_k(ChessBoard())]
    assert first == second


def test_ablation_comparison(tmp_path):
    payload = run()
    assert len(payload["ablations"]) == 4
    assert all(row["candidate_validity"] for row in payload["ablations"])
    assert payload["top_k"]
    torch = pytest.importorskip("torch")
    del torch
    assert payload["status"] == "PASS"
    assert all(row["parameter_count"] for row in payload["ablations"])


def test_malformed_candidate_state_rejection():
    pieces = {square_to_index("e1"): ChessPiece("k", "white")}
    with pytest.raises(ValueError):
        ChessBoard(pieces)
