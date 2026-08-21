"""Local research console for the Geodesic path-B stack."""

from __future__ import annotations

import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

from geodesic.chess import ChessBoard, ChessMove
from geodesic.pipeline import GeodesicPipeline
from geodesic.tools.envcheck import collect_environment

WEB_ROOT = Path(__file__).with_name("web")
MIME_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".svg": "image/svg+xml",
    ".png": "image/png",
    ".ico": "image/x-icon",
}


def parse_board(payload: dict) -> ChessBoard:
    fen = str(payload.get("fen") or "").strip()
    return ChessBoard.from_fen(fen) if fen else ChessBoard()


def serialize_board(board: ChessBoard) -> dict:
    squares = []
    for row in range(8):
        for col in range(8):
            piece = board.piece_at((row, col))
            squares.append(
                {
                    "square": "abcdefgh"[col] + str(row + 1),
                    "row": row,
                    "col": col,
                    "piece": None if piece is None else {"kind": piece.kind, "color": piece.color, "symbol": piece.symbol},
                }
            )
    return {
        "fen": board.to_fen(),
        "side_to_move": board.side_to_move,
        "in_check": board.in_check(),
        "checkmate": board.is_checkmate(),
        "stalemate": board.is_stalemate(),
        "legal_moves": [move.to_uci() for move in board.legal_moves()],
        "squares": squares,
        "ascii": board.ascii(),
    }


class GeodesicBoardHandler(BaseHTTPRequestHandler):
    server_version = "GeodesicBoard/1.0"

    def log_message(self, format: str, *args: object) -> None:
        print(f"[board] {self.address_string()} {format % args}")

    def _send(self, status: int, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, payload: object, status: int = 200) -> None:
        self._send(status, json.dumps(payload, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8")

    def _read_json(self) -> dict:
        length = int(self.headers.get("Content-Length", "0") or 0)
        raw = self.rfile.read(length) if length else b"{}"
        if not raw:
            return {}
        data = json.loads(raw.decode("utf-8"))
        if not isinstance(data, dict):
            raise ValueError("JSON body must be an object")
        return data

    def do_OPTIONS(self) -> None:  # noqa: N802
        self._send(204, b"", "text/plain")

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path == "/api/health":
            self._json({"ok": True, "service": "geodesic-board"})
            return
        if path == "/api/environment":
            self._json(collect_environment())
            return
        if path == "/api/start":
            self._json(serialize_board(ChessBoard()))
            return
        self._serve_static(path)

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        try:
            payload = self._read_json()
            if path == "/api/analyze":
                board = parse_board(payload)
                pipeline = GeodesicPipeline(metric=str(payload.get("metric") or "euclidean"), k=int(payload.get("k") or 3))
                result = pipeline.as_dict(board)
                result["board"] = serialize_board(board)
                self._json(result)
                return
            if path == "/api/legal":
                self._json(serialize_board(parse_board(payload)))
                return
            if path == "/api/apply":
                board = parse_board(payload)
                move = ChessMove.from_uci(str(payload.get("move") or ""))
                self._json(serialize_board(board.apply(move)))
                return
            if path == "/api/kernel":
                points = payload.get("points") or []
                named = [(str(item["name"]), tuple(float(value) for value in item["coordinates"])) for item in points]
                report = GeodesicPipeline(metric=str(payload.get("metric") or "euclidean")).kernel_path(named)
                self._json({"path_length": report.path_length, "distortion": report.distortion, "metric": report.metric, "anchors": report.anchors})
                return
        except Exception as exc:  # noqa: BLE001 - API boundary
            self._json({"error": str(exc)}, status=400)
            return
        self._json({"error": "unknown endpoint"}, status=404)

    def _serve_static(self, path: str) -> None:
        relative = "index.html" if path in {"", "/"} else path.lstrip("/")
        target = (WEB_ROOT / relative).resolve()
        if WEB_ROOT.resolve() not in target.parents and target != WEB_ROOT.resolve():
            self._json({"error": "invalid path"}, status=403)
            return
        if not target.is_file():
            self._json({"error": "not found"}, status=404)
            return
        content_type = MIME_TYPES.get(target.suffix, "application/octet-stream")
        self._send(200, target.read_bytes(), content_type)


def serve(host: str = "0.0.0.0", port: int = 8765) -> None:
    httpd = ThreadingHTTPServer((host, port), GeodesicBoardHandler)
    print(f"Geodesic board listening on http://{host}:{port}")
    httpd.serve_forever()


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the Geodesic research console")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    serve(args.host, args.port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
