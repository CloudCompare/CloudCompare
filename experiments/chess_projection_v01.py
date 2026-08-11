"""Step 4 benchmark writer for Geodesic Chess Projection v0.1."""

from __future__ import annotations

import json
from pathlib import Path

from geodesic.chess import benchmark_chess_projection

RESULTS_PATH = Path(__file__).with_name("results") / "chess_projection_v01.json"


def run() -> dict[str, object]:
    payload = benchmark_chess_projection()
    RESULTS_PATH.parent.mkdir(parents=True, exist_ok=True)
    RESULTS_PATH.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return payload


if __name__ == "__main__":
    print(json.dumps(run(), indent=2))
