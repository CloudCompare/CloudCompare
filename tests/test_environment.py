import json
from pathlib import Path

from geodesic.tools.envcheck import collect_environment, write_environment_report
from geodesic.tools.validate import determine_status


def test_envcheck_reports_required_fields(tmp_path):
    path = tmp_path / "environment_report.json"
    report = write_environment_report(path)
    saved = json.loads(path.read_text(encoding="utf-8"))
    for key in [
        "python_version",
        "os",
        "pytorch_version",
        "torch_available",
        "cuda_available",
        "gpu_info",
        "pytest_version",
        "dependency_status",
        "machine_status",
    ]:
        assert key in report
        assert key in saved


def test_validation_policy_blocks_without_torch():
    environment = collect_environment() | {"torch_available": False}
    assert determine_status(environment, []) == "VALIDATION_BLOCKED"
