# Reproducible Validation Environment

## Purpose

Step 3.1 provides validation infrastructure for the Geodesic Transformer without changing the Step 3 mathematics. The repository remains usable when PyTorch is unavailable, but the Step 3 scientific gate cannot pass until mandatory PyTorch tests actually execute.

## Dependencies

Runtime dependencies are declared in `requirements.txt`; development and validation dependencies are declared in `requirements-dev.txt`. Installation is an explicit environment setup operation and is not performed by validation commands.

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-dev.txt
make validate
make board
```

The validation tooling does not require internet access at runtime and does not use cloud services or external APIs.

## Environment Diagnostic

Run:

```bash
python -m geodesic.tools.envcheck
```

The command writes `experiments/results/environment_report.json` and reports:

- Python version
- Python executable
- operating system
- PyTorch version
- CUDA availability
- GPU information
- pytest version
- dependency status
- machine-readable status

## Validation Pipeline

Run:

```bash
make validate
```

The pipeline performs:

1. environment check
2. compileall
3. unit tests
4. integration tests
5. stability tests
6. ablation tests
7. lambda sweep
8. benchmark generation

When PyTorch is unavailable, the pipeline still runs compileall and non-PyTorch static/reference tests, writes blocked benchmark metadata, and reports `VALIDATION_BLOCKED`.

## Strict Validation Policy

Machine-readable statuses are:

- `PASS`: PyTorch is available, all mandatory PyTorch tests execute, no mandatory test is skipped, and all benchmarks complete.
- `FAIL`: PyTorch is available, but a mandatory test fails, a mandatory test is skipped, or benchmark execution fails.
- `VALIDATION_BLOCKED`: PyTorch is unavailable, so mandatory Step 3 validation cannot execute.

Skipped PyTorch tests are never counted as passing tests. Git and PR workflows are separate from scientific validation.

## Generated Files

- `experiments/results/environment_report.json`
- `experiments/results/step3_validation.json`
- `experiments/results/geodesic_transformer_v01.json`

## Current Expected Status in This Environment

Because PyTorch is unavailable in the current execution environment, Step 3.1 infrastructure can pass, while Step 3 itself remains `IMPLEMENTED_VALIDATION_BLOCKED` until PyTorch tests and benchmarks execute.
