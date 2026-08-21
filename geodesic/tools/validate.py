"""Strict Step 3 validation pipeline."""

from __future__ import annotations

import json
import subprocess
import sys
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any

from geodesic.tools.envcheck import collect_environment, write_environment_report

VALIDATION_REPORT_PATH = Path("experiments/results/step3_validation.json")
MANDATORY_PYTORCH_TESTS = [
    "test_standard_equivalence",
    "test_geodesic_distance_effect",
    "test_translator_bound",
    "test_point_attention_normalization",
    "test_multihead_shape",
    "test_multihead_diversity",
    "test_gradient_stability",
    "test_attention_entropy",
    "test_nan_inf",
    "test_pressure_response",
]


@dataclass(frozen=True)
class CommandResult:
    name: str
    command: list[str]
    returncode: int
    stdout: str
    stderr: str


def run_command(name: str, command: list[str]) -> CommandResult:
    completed = subprocess.run(command, check=False, text=True, capture_output=True)
    return CommandResult(name, command, completed.returncode, completed.stdout, completed.stderr)


def determine_status(environment: dict[str, Any], command_results: list[CommandResult]) -> str:
    """Apply strict validation policy: PASS, FAIL, or VALIDATION_BLOCKED."""

    if not environment["torch_available"]:
        return "VALIDATION_BLOCKED"
    if any(result.returncode != 0 for result in command_results):
        return "FAIL"
    combined = "\n".join(result.stdout + result.stderr for result in command_results).lower()
    if " skipped" in combined or "skip" in combined:
        return "FAIL"
    return "PASS"


def run_validation() -> dict[str, Any]:
    environment = write_environment_report()
    commands = [run_command("compileall", [sys.executable, "-m", "compileall", "geodesic", "experiments", "tests"])]
    if environment["torch_available"]:
        commands.extend(
            [
                run_command("unit_integration_stability_ablation_tests", [sys.executable, "-m", "pytest", "tests"]),
                run_command("lambda_sweep_benchmark", [sys.executable, "-m", "experiments.geodesic_transformer_v01"]),
                run_command("translator_benchmark", [sys.executable, "-m", "experiments.translator_v01"]),
                run_command("chess_projection_benchmark", [sys.executable, "-m", "experiments.chess_projection_v01"]),
            ]
        )
    else:
        commands.append(run_command("non_pytorch_static_tests", [sys.executable, "-m", "pytest", "tests/test_anchor.py", "tests/test_metric.py", "tests/test_manifold.py", "tests/test_point_attention.py::test_attention_equation_reference_weights_are_normalized_and_non_negative", "tests/test_geodesic_transformer.py::test_standard_attention_reference_equation_environment_independent"]))
        blocked = {
            "configuration": {"mandatory_pytorch_tests": MANDATORY_PYTORCH_TESTS},
            "status": "VALIDATION_BLOCKED",
            "blocked_reason": "PyTorch is unavailable; mandatory Step 3 PyTorch tests were not executed.",
            "results": [],
        }
        benchmark_path = Path("experiments/results/geodesic_transformer_v01.json")
        benchmark_path.parent.mkdir(parents=True, exist_ok=True)
        benchmark_path.write_text(json.dumps(blocked, indent=2), encoding="utf-8")

    status = determine_status(environment, commands)
    payload = {
        "status": status,
        "strict_policy": {
            "pass_requires_pytorch": True,
            "skipped_pytorch_tests_count_as_pass": False,
            "blocked_if_pytorch_unavailable": True,
            "fail_if_pytorch_available_and_tests_fail_or_skip": True,
        },
        "environment": environment,
        "mandatory_pytorch_tests": MANDATORY_PYTORCH_TESTS,
        "commands": [asdict(result) for result in commands],
    }
    VALIDATION_REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    VALIDATION_REPORT_PATH.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps(payload, indent=2))
    return payload


def main() -> int:
    payload = run_validation()
    if payload["status"] == "FAIL":
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
