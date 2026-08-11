# Step 2 — Geodesic Translator v0.1

## Scope

Step 2 keeps the Step 1 Geodesic Kernel frozen and adds an independently testable PyTorch attention layer plus a bounded translator regulator. The representation is a classical numerical manifold representation; this document makes no claim of quantum computation.

## Core equations

The attention score is computed as:

```text
S_ij = (Q_i · K_j) / sqrt(d_k) - λ * d_g(Q_i, K_j) + Γ_translator,ij
A = softmax(S)
```

The geodesic distance term uses the existing kernel metric semantics: Euclidean distance over the same coordinate space. In the PyTorch attention implementation this is evaluated with `torch.cdist(..., p=2)`, matching the Step 1 Euclidean metric.

The translator correction is bounded by construction:

```text
Γ_translator = γ_max * tanh(g_theta(state))
|Γ_translator| <= γ_max
```

The translator does not replace attention. It only contributes the additive bounded control term `Γ_translator,ij` before the required final normalization `A = softmax(S)`.

## Architecture

- `GeodesicAttention` computes standard scaled dot-product content scores, subtracts the geodesic penalty, adds the optional translator correction, then applies softmax.
- `GeodesicTranslator` receives geometric and state measurements and produces a deterministic state plus bounded correction.
- Supported translator states are `NORMAL`, `CAUTION`, `PRESSURE`, `RECOVERY`, and `REJECT`.
- Experimental modes are:
  - Baseline: `λ = 0`, `Γ = 0`.
  - Geodesic: `λ > 0`, `Γ = 0`.
  - Geodesic + Translator: `λ > 0`, `Γ != 0`.

## Measurements

The translator computes:

- `geodesic_error`: mean geodesic distance.
- `mirror_error`: distance between mean query and key states.
- `curvature_energy`: variance-like energy of centered geodesic distances.
- `state_velocity`: mean movement relative to the previous query state.
- `uncertainty`: normalized entropy of a geodesic-derived distribution.
- `pressure`: deterministic aggregate of the preceding measurements.

## Test methodology

The unit tests validate:

1. Baseline equivalence to standard scaled dot-product attention when `λ=0` and `Γ=0`.
2. Geodesic sensitivity when content similarity is approximately equal.
3. Translator boundedness, checking `abs(Γ) <= γ_max`.
4. Deterministic translator pressure response.
5. Stability metrics under increasing perturbation: entropy, gradient norm, score range, NaN/Inf occurrence, geodesic error, mirror error, and pressure.
6. Ablation benchmark generation across baseline, geodesic, and geodesic + translator modes.

## Lambda sweep

The benchmark evaluates:

```text
λ ∈ {0, 0.05, 0.1, 0.25, 0.5, 1.0}
```

The generated JSON records measured stable, collapse, and excessive-constraint regions from the run. No best lambda is hard-coded.

## Results

The benchmark writes `experiments/results/translator_v01.json` with configuration, lambda values, gamma maximum, metrics, test results, and pass/fail status. Results are deterministic when PyTorch is available because the benchmark sets a fixed seed.

## Limitations

- This is a classical numerical prototype, not quantum computation.
- The translator uses deterministic handcrafted measurements rather than a learned neural controller.
- Transformer, SNN, STDP, MCTS, and chess integrations are intentionally out of scope for Step 2.
- The benchmark is small and should not be interpreted as evidence of model-quality improvement; it only validates behavior and stability for the defined formulation.

## Validation Status and Environment Constraints

Step 2.1 separates implementation status from runtime validation status. The codebase now writes `experiments/results/environment_report.json` with Python version, OS, PyTorch availability, CUDA availability, pytest version, and the resulting validation status.

PyTorch is an explicit dependency in `requirements.txt`, but validation does not attempt to bypass network restrictions or install packages automatically. If PyTorch is unavailable, PyTorch-specific tests are marked environment-blocked/skipped and the final project status must be `IMPLEMENTED_VALIDATION_BLOCKED`, not `PASS`.

`PointAttentionAssistant` is isolated from Transformer stacks. It accepts batched local states and neighborhoods, rejects NaN/Inf inputs, exposes normalized non-negative attention weights, and preserves the local-state tensor shape. An environment-independent mathematical reference test validates the softmax normalization properties without requiring PyTorch; the synthetic PyTorch tests execute automatically when `torch` becomes available.

Generated reports:

- `experiments/results/environment_report.json`
- `experiments/results/point_attention_v01.json`
