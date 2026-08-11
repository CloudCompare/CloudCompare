"""Environment diagnostics for GEODESIC validation."""

from __future__ import annotations

import importlib.util
import json
import platform
import sys
from pathlib import Path
from typing import Any

ENVIRONMENT_REPORT_PATH = Path("experiments/results/environment_report.json")


def collect_environment() -> dict[str, Any]:
    """Collect dependency and hardware status without requiring internet access."""

    pytest_available = importlib.util.find_spec("pytest") is not None
    pytest_version = None
    if pytest_available:
        import pytest

        pytest_version = pytest.__version__

    torch_available = importlib.util.find_spec("torch") is not None
    torch_version = None
    cuda_available = False
    gpu_info: list[dict[str, Any]] = []
    if torch_available:
        import torch

        torch_version = torch.__version__
        cuda_available = bool(torch.cuda.is_available())
        if cuda_available:
            gpu_info = [
                {
                    "index": index,
                    "name": torch.cuda.get_device_name(index),
                    "capability": list(torch.cuda.get_device_capability(index)),
                }
                for index in range(torch.cuda.device_count())
            ]

    dependency_status = {
        "torch": "available" if torch_available else "missing",
        "pytest": "available" if pytest_available else "missing",
    }
    mandatory_available = torch_available and pytest_available
    return {
        "python_version": sys.version,
        "python_executable": sys.executable,
        "os": platform.platform(),
        "pytorch_version": torch_version,
        "torch_available": torch_available,
        "cuda_available": cuda_available,
        "gpu_info": gpu_info,
        "pytest_version": pytest_version,
        "dependency_status": dependency_status,
        "machine_status": "PASS" if mandatory_available else "VALIDATION_BLOCKED",
        "blocked_reason": None if mandatory_available else "Mandatory PyTorch validation dependency is unavailable.",
    }


def write_environment_report(path: Path = ENVIRONMENT_REPORT_PATH) -> dict[str, Any]:
    """Write the environment report and return it."""

    report = collect_environment()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


def main() -> int:
    """CLI entry point for ``python -m geodesic.tools.envcheck``."""

    print(json.dumps(write_environment_report(), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
