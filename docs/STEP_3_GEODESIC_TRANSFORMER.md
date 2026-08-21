# Step 3 — Geodesic Transformer v0.1

## Scope

Step 3 adds a production-oriented classical Geodesic Transformer while preserving the frozen Geodesic Kernel v0.1, Geodesic metric definition, Translator v0.1, bounded translator correction, Point Attention Assistant, and the validated attention equation from Step 2.

## Equations

Each attention head computes:

```text
S_h = (Q_h K_h^T) / sqrt(d_h) - λ_h D_g,h + Γ_h
A_h = softmax(S_h)
O = Concat(A_h V_h) W_o
```

The final normalization remains `A_h = softmax(S_h)`. The translator remains bounded by `Γ = gamma_max * tanh(g_theta(state))`, so `abs(Γ) <= gamma_max`.

## Architecture

`GeodesicTransformerBlock` follows:

```text
Input
→ Geodesic Multi-Head Attention
→ Residual
→ LayerNorm
→ Geodesic FFN
→ Residual
→ LayerNorm
```

`GeodesicPositionalRepresentation` combines content embeddings with normalized geodesic coordinates and an optional radial coordinate without introducing unnecessary dimensionality.

## Interfaces

Configuration exposes `d_model`, `num_heads`, `num_layers`, `lambda_geodesic`, `gamma_max`, `dropout`, `geodesic_metric`, `use_point_attention`, `use_translator`, `learnable_translator`, and `g_theta_hidden`. `geodesic_metric` now dispatches Euclidean (`p=2`) and Manhattan (`p=1`) distances in both Attention and Transformer.

## Point Attention Integration

`PointAttentionAssistant` is used only as a local geometric signal. Its output influences translator measurements through the bounded translator path and does not bypass the translator correction mechanism.

## Experiments

The synthetic benchmark evaluates the same generated dataset under four ablations:

- A: standard attention
- B: geodesic attention
- C: geodesic + translator
- D: geodesic + translator + point attention

The benchmark measures loss, accuracy, attention entropy, gradient norm, geodesic error, curvature energy, translator pressure, runtime, and parameter count across `λ ∈ {0, 0.05, 0.1, 0.25, 0.5, 1.0}`.

## Ablation Results

Results are saved to `experiments/results/geodesic_transformer_v01.json`. In the current environment the status is `IMPLEMENTED_VALIDATION_BLOCKED` because PyTorch is not installed, so critical Step 3 tests and benchmark execution cannot run.

## Stability Observations

The implementation clamps logits before softmax, rejects NaN/Inf inputs, checks non-finite attention values, and records NaN/Inf occurrence during the synthetic benchmark when PyTorch is available.

## Limitations

- This remains a classical numerical manifold representation, not quantum computation.
- No Transformer-scale training claim is made from the synthetic benchmark.
- No preferred lambda is selected without measurements.
- PASS must not be reported while PyTorch tests are skipped.
