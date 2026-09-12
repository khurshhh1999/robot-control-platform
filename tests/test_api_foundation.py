"""API foundation tests: headers, health, CORS, docs, and error mapping."""

from __future__ import annotations

import json
import os
from collections.abc import Iterator
from pathlib import Path
from unittest.mock import AsyncMock

import pytest
from fastapi.testclient import TestClient
from pydantic import SecretStr
from robot_control_platform_api.dependencies import (
    AppRuntime,
    build_default_artifact_probe,
)
from robot_control_platform_api.errors import (
    PROBLEM_CONTENT_TYPE,
    ApiError,
    map_exception,
)
from robot_control_platform_api.main import create_app, docs_enabled_for
from robot_control_platform_api.middleware import normalize_request_id
from robot_control_platform_common.artifacts.base import ArtifactNotFoundError
from robot_control_platform_common.artifacts.filesystem import FilesystemArtifactStore
from robot_control_platform_common.config import (
    LogLevel,
    RuntimeEnv,
    Settings,
    load_settings,
)
from robot_control_platform_common.db.repositories.exceptions import EntityNotFoundError
from robot_control_platform_common.logging import configure_logging, get_logger
from sqlalchemy.ext.asyncio import AsyncEngine, AsyncSession, async_sessionmaker


def _clear_rcp_environment(monkeypatch: pytest.MonkeyPatch) -> None:
    for key in list(os.environ):
        if key.upper().startswith("RCP_"):
            monkeypatch.delenv(key, raising=False)


def _settings(tmp_path: Path, *, env: RuntimeEnv = RuntimeEnv.TEST) -> Settings:
    artifact_root = tmp_path / "artifacts"
    artifact_root.mkdir(parents=True, exist_ok=True)
    return Settings(
        env=env,
        log_level=LogLevel.WARNING,
        database_host="127.0.0.1",
        database_port=5432,
        database_name="robot_platform",
        database_user="robot_app",
        database_password=SecretStr("test-password"),
        artifact_root=artifact_root,
        api_base_url="http://127.0.0.1:8000",
        worker_id="api-foundation-test",
        run_lease_seconds=30,
        run_heartbeat_seconds=10,
        simulation_gui=False,
    )


def _runtime(
    settings: Settings,
    *,
    database_ok: bool = True,
    artifacts_ok: bool = True,
) -> AppRuntime:
    store = FilesystemArtifactStore(settings.artifact_root)

    async def check_database() -> None:
        if not database_ok:
            raise ApiError(
                "DEPENDENCY_UNAVAILABLE",
                status=503,
                detail="Database is unavailable",
            )

    def check_artifacts() -> None:
        if not artifacts_ok:
            raise ApiError(
                "DEPENDENCY_UNAVAILABLE",
                status=503,
                detail="Artifact store is unavailable",
            )
        build_default_artifact_probe(store)()

    engine = AsyncMock(spec=AsyncEngine)
    engine.dispose = AsyncMock()
    session_factory = AsyncMock(spec=async_sessionmaker[AsyncSession])
    return AppRuntime(
        settings=settings,
        engine=engine,
        session_factory=session_factory,
        artifact_store=store,
        check_database=check_database,
        check_artifacts=check_artifacts,
    )


@pytest.fixture
def client(tmp_path: Path) -> Iterator[TestClient]:
    settings = _settings(tmp_path)
    app = create_app(settings, runtime=_runtime(settings))
    with TestClient(app) as test_client:
        yield test_client


def test_live_is_dependency_free(client: TestClient) -> None:
    response = client.get("/health/live")
    assert response.status_code == 200
    assert response.json() == {"status": "ok"}
    assert "X-Request-Id" in response.headers


def test_ready_succeeds_when_dependencies_pass(client: TestClient) -> None:
    response = client.get("/health/ready")
    assert response.status_code == 200
    assert response.json() == {"status": "ok"}


def test_ready_fails_when_database_unavailable(tmp_path: Path) -> None:
    settings = _settings(tmp_path)
    app = create_app(settings, runtime=_runtime(settings, database_ok=False))
    with TestClient(app) as test_client:
        response = test_client.get("/health/ready")
    assert response.status_code == 503
    assert response.headers["content-type"].startswith(PROBLEM_CONTENT_TYPE)
    payload = response.json()
    assert payload["code"] == "DEPENDENCY_UNAVAILABLE"
    assert payload["status"] == 503
    assert "password" not in json.dumps(payload).lower()
    assert "127.0.0.1" not in json.dumps(payload)


def test_ready_fails_when_artifact_store_unavailable(tmp_path: Path) -> None:
    settings = _settings(tmp_path)
    app = create_app(settings, runtime=_runtime(settings, artifacts_ok=False))
    with TestClient(app) as test_client:
        response = test_client.get("/health/ready")
    assert response.status_code == 503
    payload = response.json()
    assert payload["code"] == "DEPENDENCY_UNAVAILABLE"
    assert str(settings.artifact_root) not in json.dumps(payload)


def test_request_id_echoes_valid_incoming_header(client: TestClient) -> None:
    response = client.get("/health/live", headers={"X-Request-Id": "req-test-1"})
    assert response.headers["X-Request-Id"] == "req-test-1"


def test_request_id_generated_when_missing_or_invalid(client: TestClient) -> None:
    missing = client.get("/health/live")
    assert missing.headers["X-Request-Id"]
    invalid = client.get("/health/live", headers={"X-Request-Id": "bad id with spaces"})
    assert invalid.headers["X-Request-Id"] != "bad id with spaces"
    assert normalize_request_id(None)
    assert normalize_request_id("ok-id-1") == "ok-id-1"


def test_request_id_middleware_binds_log_context(capsys: pytest.CaptureFixture[str]) -> None:
    from robot_control_platform_api.middleware import RequestIdMiddleware
    from robot_control_platform_common.logging import clear_log_context, configure_logging
    from starlette.applications import Starlette
    from starlette.responses import JSONResponse
    from starlette.routing import Route

    configure_logging("api", "INFO")
    clear_log_context()

    async def endpoint(request: object) -> JSONResponse:
        get_logger("api").info("during_request")
        return JSONResponse({"ok": True})

    inner = Starlette(routes=[Route("/probe", endpoint)])
    app = RequestIdMiddleware(inner)
    with TestClient(app) as test_client:
        response = test_client.get("/probe", headers={"X-Request-Id": "req-log-1"})
    assert response.status_code == 200
    assert response.headers["X-Request-Id"] == "req-log-1"
    lines = [line for line in capsys.readouterr().out.splitlines() if line.strip().startswith("{")]
    payloads = [json.loads(line) for line in lines]
    during = next(item for item in payloads if item.get("event") == "during_request")
    assert during["request_id"] == "req-log-1"


def test_cors_uses_explicit_origins_without_wildcard_credentials(client: TestClient) -> None:
    response = client.options(
        "/health/live",
        headers={
            "Origin": "http://localhost:3000",
            "Access-Control-Request-Method": "GET",
        },
    )
    assert response.status_code == 200
    assert response.headers["access-control-allow-origin"] == "http://localhost:3000"
    assert "access-control-allow-credentials" not in response.headers or response.headers.get(
        "access-control-allow-credentials"
    ) in {None, "false"}


def test_cors_rejects_unknown_origin(client: TestClient) -> None:
    response = client.get("/health/live", headers={"Origin": "http://evil.example"})
    assert response.status_code == 200
    assert "access-control-allow-origin" not in response.headers


def test_demo_mode_disables_docs_unless_explicitly_enabled(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    settings = _settings(tmp_path, env=RuntimeEnv.DEMO)
    monkeypatch.delenv("RCP_ENABLE_DOCS", raising=False)
    assert docs_enabled_for(settings) is False
    app = create_app(settings, runtime=_runtime(settings))
    with TestClient(app) as test_client:
        assert test_client.get("/docs").status_code == 404
        assert test_client.get("/openapi.json").status_code == 404
        assert test_client.get("/redoc").status_code == 404

    monkeypatch.setenv("RCP_ENABLE_DOCS", "true")
    assert docs_enabled_for(settings) is True
    enabled = create_app(settings, runtime=_runtime(settings))
    with TestClient(enabled) as test_client:
        assert test_client.get("/openapi.json").status_code == 200


def test_error_mapping_returns_problem_details(tmp_path: Path) -> None:
    settings = _settings(tmp_path)
    app = create_app(settings, runtime=_runtime(settings))
    cases: list[tuple[str, int, str]] = [
        ("RESOURCE_NOT_FOUND", 404, "RESOURCE_NOT_FOUND"),
        ("ARTIFACT_NOT_FOUND", 404, "ARTIFACT_NOT_FOUND"),
        ("ARTIFACT_INTEGRITY_FAILURE", 409, "ARTIFACT_INTEGRITY_FAILURE"),
        ("CONFLICT", 409, "CONFLICT"),
        ("VALIDATION_ERROR", 422, "VALIDATION_ERROR"),
        ("DEPENDENCY_UNAVAILABLE", 503, "DEPENDENCY_UNAVAILABLE"),
        ("INTERNAL_ERROR", 500, "INTERNAL_ERROR"),
    ]
    # Unexpected Exception handlers are re-raised by ServerErrorMiddleware after
    # producing a response; disable that for the INTERNAL_ERROR probe only.
    with TestClient(app, raise_server_exceptions=False) as client:
        for path_code, status, expected_code in cases:
            response = client.get(f"/__test__/errors/{path_code}")
            assert response.status_code == status, path_code
            assert response.headers["content-type"].startswith(PROBLEM_CONTENT_TYPE)
            payload = response.json()
            assert payload["code"] == expected_code
            assert payload["status"] == status
            assert payload["type"].startswith("urn:robot-control-platform:problem:")
            body = json.dumps(payload)
            assert "traceback" not in body.lower()
            assert "probe boom" not in body
            assert "test-password" not in body


def test_map_exception_covers_domain_errors() -> None:
    not_found = map_exception(EntityNotFoundError("x"))
    assert not_found.code == "RESOURCE_NOT_FOUND"
    artifact = map_exception(ArtifactNotFoundError("y"))
    assert artifact.code == "ARTIFACT_NOT_FOUND"
    unexpected = map_exception(RuntimeError("secret-path-/tmp/x"))
    assert unexpected.code == "INTERNAL_ERROR"
    assert "secret-path" not in unexpected.detail
    assert "/tmp" not in unexpected.detail


def test_sensitive_values_are_redacted_in_api_logs(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    configure_logging("api", "INFO")
    secret = "super-secret-password-value"
    get_logger("api").info(
        "api_foundation_redaction",
        database_password=secret,
        authorization=f"Bearer {secret}",
        database_dsn=f"postgresql://robot_app:{secret}@db:5432/robot_platform",
    )
    stdout = capsys.readouterr().out
    assert secret not in stdout
    payload = json.loads([line for line in stdout.splitlines() if line.strip()][-1])
    assert payload["database_password"] == "[REDACTED]"
    assert payload["authorization"] == "[REDACTED]"
    assert payload["database_dsn"] == "[REDACTED]"


def test_load_settings_still_validates_for_api_startup(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    _clear_rcp_environment(monkeypatch)
    artifact_root = tmp_path / "artifacts"
    artifact_root.mkdir()
    monkeypatch.setenv("RCP_ENV", "test")
    monkeypatch.setenv("RCP_LOG_LEVEL", "INFO")
    monkeypatch.setenv("RCP_DATABASE_HOST", "db")
    monkeypatch.setenv("RCP_DATABASE_PORT", "5432")
    monkeypatch.setenv("RCP_DATABASE_NAME", "robot_platform")
    monkeypatch.setenv("RCP_DATABASE_USER", "robot_app")
    monkeypatch.setenv("RCP_DATABASE_PASSWORD", "test-password")
    monkeypatch.setenv("RCP_ARTIFACT_ROOT", str(artifact_root))
    monkeypatch.setenv("RCP_API_BASE_URL", "http://api:8000")
    monkeypatch.setenv("RCP_WORKER_ID", "api-1")
    settings = load_settings()
    assert settings.env is RuntimeEnv.TEST
    app = create_app(settings, runtime=_runtime(settings))
    with TestClient(app) as test_client:
        assert test_client.get("/health/live").status_code == 200


def test_unknown_route_returns_problem_details(client: TestClient) -> None:
    response = client.get("/no-such-route")
    assert response.status_code == 404
    assert response.headers["content-type"].startswith(PROBLEM_CONTENT_TYPE)
    assert response.json()["code"] == "RESOURCE_NOT_FOUND"
