import pytest

from geodesic.chess import ChessBoard, ChessMove, ChessPiece, square_to_index


def test_starting_position_has_twenty_legal_moves():
    assert len(ChessBoard().legal_moves()) == 20


def test_cannot_move_into_check():
    board = ChessBoard.from_fen("k3r3/8/8/8/8/8/8/4K3 w - - 0 1")
    assert not board.is_legal_move(ChessMove.from_uci("e1e2"))
    assert board.is_legal_move(ChessMove.from_uci("e1d1"))
    assert board.is_legal_move(ChessMove.from_uci("e1f1"))


def test_pinned_piece_cannot_leave_the_pin_line():
    board = ChessBoard.from_fen("4r2k/8/8/8/8/8/4R3/4K3 w - - 0 1")
    assert not board.is_legal_move(ChessMove.from_uci("e2d2"))
    assert board.is_legal_move(ChessMove.from_uci("e2e3"))


def test_kingside_castling_moves_king_and_rook():
    board = ChessBoard.from_fen("4k3/8/8/8/8/8/8/4K2R w K - 0 1")
    assert board.is_legal_move(ChessMove.from_uci("e1g1"))
    next_board = board.apply(ChessMove.from_uci("e1g1"))
    assert next_board.piece_at(square_to_index("g1")) == ChessPiece("k", "white")
    assert next_board.piece_at(square_to_index("f1")) == ChessPiece("r", "white")
    assert next_board.piece_at(square_to_index("e1")) is None
    assert next_board.piece_at(square_to_index("h1")) is None


def test_cannot_castle_out_of_check():
    board = ChessBoard.from_fen("4k3/8/8/8/8/8/4r3/4K2R w K - 0 1")
    assert not board.is_legal_move(ChessMove.from_uci("e1g1"))


def test_cannot_castle_through_attacked_square():
    board = ChessBoard.from_fen("4k3/8/8/8/8/8/5r2/4K2R w K - 0 1")
    assert not board.is_legal_move(ChessMove.from_uci("e1g1"))


def test_en_passant_capture_removes_the_passed_pawn():
    board = ChessBoard.from_fen("4k3/8/8/3pP3/8/8/8/4K3 w - d6 0 1")
    assert board.is_legal_move(ChessMove.from_uci("e5d6"))
    next_board = board.apply(ChessMove.from_uci("e5d6"))
    assert next_board.piece_at(square_to_index("d6")) == ChessPiece("p", "white")
    assert next_board.piece_at(square_to_index("d5")) is None


def test_promotion_defaults_to_queen_and_accepts_explicit_piece():
    board = ChessBoard.from_fen("4k3/P7/8/8/8/8/8/4K3 w - - 0 1")
    assert board.is_legal_move(ChessMove.from_uci("a7a8q"))
    assert board.is_legal_move(ChessMove.from_uci("a7a8"))
    queen_board = board.apply(ChessMove.from_uci("a7a8q"))
    rook_board = board.apply(ChessMove.from_uci("a7a8r"))
    assert queen_board.piece_at(square_to_index("a8")) == ChessPiece("q", "white")
    assert rook_board.piece_at(square_to_index("a8")) == ChessPiece("r", "white")


def test_fen_round_trip_preserves_castling_and_en_passant():
    fen = "4k3/8/8/3pP3/8/8/8/4K3 w - d6 0 1"
    assert ChessBoard.from_fen(fen).to_fen().startswith("4k3/8/8/3pP3/8/8/8/4K3 w - d6")


def test_checkmate_and_stalemate_detection():
    mate = ChessBoard.from_fen("7k/5KQ1/8/8/8/8/8/8 b - - 0 1")
    stale = ChessBoard.from_fen("7k/5K2/6Q1/8/8/8/8/8 b - - 0 1")
    assert mate.is_checkmate()
    assert stale.is_stalemate()
