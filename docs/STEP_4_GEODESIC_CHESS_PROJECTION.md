# Step 4 — Geodesic Chess Projection v0.1

## Role of Chess

Chess is not the governing law of geodesic space. In Step 4 it is a logical projection, structured hypothesis generator, consistency filter, and experimental environment layered on top of the preserved Geodesic Transformer stack.

## Architecture

```text
Geodesic Transformer
↓
Point Attention
↓
Translator
↓
Chess Projection
↓
Logical State Representation
↓
Candidate Generation
↓
Legal State Validation
↓
Geodesic Scoring
↓
Top-K hypotheses
```

The frozen Geodesic Kernel v0.1, Geodesic Attention equation, Translator v0.1, Point Attention Assistant, and Geodesic Transformer v0.1 are not modified.

## Board Representation

A board is encoded as `X ∈ R^(8×8×d)` with `d >= 12`. Each square encodes occupancy, piece type, piece color, normalized position, side-to-move state, and geodesic/radial coordinates compatible with the existing Euclidean geodesic kernel.

## Projection Channels

Chess Projection returns separate measurable channels and does not collapse them immediately:

- `control`: normalized legal move count
- `threat`: normalized capturable material
- `mobility`: normalized unique target squares
- `stability`: bounded material-balance stability proxy

## Legality vs Quality

Legality is checked before scoring. Illegal board states, illegal moves, impossible transitions, and malformed candidate states are rejected. A legal state is only eligible for quality scoring; legality does not imply that the state is good.

## Transparent Geodesic Scoring

For each Top-K candidate, the projection reports `geodesic_error`, `pressure`, `control`, `threat`, `mobility`, `stability`, and `overall_score`. The score is transparent:

```text
overall_score = control + threat + mobility + stability - geodesic_error - pressure
pressure = gamma_max * tanh(geodesic_error + threat + (1 - stability))
```

The geodesic error uses the existing Step 1 Euclidean metric over normalized board coordinates.

## Ablation

The benchmark records four ablations:

- A = Transformer
- B = Transformer + Chess
- C = Transformer + Geodesic + Chess
- D = Transformer + Geodesic + Translator + Chess

Measured fields include candidate validity, hypothesis diversity, geodesic error, pressure, stability, inference latency, and parameter count.

## Wiring

`TransformerChessProjection` now consumes real Transformer outputs. Legal candidates are generated first, encoded as `64` tokens, then scored with Transformer quality, geodesic traces, translator pressure, and the frozen logical channels. `GeodesicPipeline` is the executable Kernel → Transformer → Chess graph.

## Chess rules

The board now validates check, pins, castling, en passant, and promotion. It remains a hypothesis filter, not a tournament engine. Draw-by-repetition and the fifty-move rule are still out of scope.

## Validation Status

If PyTorch is available, `experiments.chess_projection_v01` runs Transformer-backed ablations and may report `PASS`. If PyTorch is missing, the status remains `VALIDATION_BLOCKED`.

## Results

Results are saved to `experiments/results/chess_projection_v01.json`.

## Limitations

- The legal move validator is deterministic and pseudo-legal; it does not yet evaluate check, castling, en passant, promotion, repetition, or draw rules.
- The scoring function is intentionally transparent and hand-authored; it is not an opaque neural quality model.
- The objective is state consistency and hypothesis control, not proving chess improves general intelligence.
