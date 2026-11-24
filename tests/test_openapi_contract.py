"""OpenAPI export stays in-process and matches the committed contract."""

from __future__ import annotations

from pathlib import Path

import pytest
from robot_control_platform_api.openapi_export import (
    build_openapi_document,
    canonical_openapi_text,
    export_settings,
)
from robot_control_platform_common.config import RuntimeEnv

_REPO_ROOT = Path(__file__).resolve().parents[1]
_OPENAPI_PATH = _REPO_ROOT / "packages" / "contracts" / "openapi.json"

_REQUIRED_PATHS = (
    "/health/live",
    "/health/ready",
    "/api/v1/policies",
    "/api/v1/scenario-sets",
    "/api/v1/scenario-sets/{scenario_set_id}",
    "/api/v1/experiments",
    "/api/v1/experiments/{experiment_id}",
    "/api/v1/experiments/{experiment_id}/runs",
    "/api/v1/runs/{run_id}",
    "/api/v1/runs/{run_id}/cancel",
    "/api/v1/trials",
    "/api/v1/trials/{trial_id}",
    "/api/v1/trials/{trial_id}/events",
    "/api/v1/trials/{trial_id}/artifacts",
    "/api/v1/trials/{trial_id}/annotations",
    "/api/v1/artifacts/{artifact_id}/content",
    "/api/v1/annotations/{annotation_id}",
)


def test_export_uses_test_configuration_without_runtime_startup(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def fail_open(*_args: object, **_kwargs: object) -> None:
        raise AssertionError("runtime startup is not allowed during OpenAPI export")

    monkeypatch.setattr("robot_control_platform_api.main.open_runtime", fail_open)
    assert export_settings().env is RuntimeEnv.TEST
    document = build_openapi_document()
    assert document["info"]["title"] == "Robot Control Platform API"
    assert str(document["openapi"]).startswith("3.")
    paths = document["paths"]
    assert isinstance(paths, dict)
    for path in _REQUIRED_PATHS:
        assert path in paths
    assert "/__test__/errors/{code}" not in paths
    rendered = canonical_openapi_text(document)
    assert "openapi-export-placeholder" not in rendered
    assert "postgresql://" not in rendered
    assert "/Users/" not in rendered
    assert ".project-private" not in rendered
    assert "AGENTS.md" not in rendered


def test_committed_openapi_matches_export() -> None:
    exported = canonical_openapi_text(build_openapi_document())
    assert _OPENAPI_PATH.is_file()
    assert _OPENAPI_PATH.read_text(encoding="utf-8") == exported
