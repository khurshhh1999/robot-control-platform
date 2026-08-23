import os
from pathlib import Path

import pytest
from robot_control_platform_common.config import (
    ConfigurationError,
    LogLevel,
    RuntimeEnv,
    Settings,
    load_settings,
)


def _clear_rcp_environment(monkeypatch: pytest.MonkeyPatch) -> None:
    for key in list(os.environ):
        if key.upper().startswith("RCP_"):
            monkeypatch.delenv(key, raising=False)


def _set_required_environment(monkeypatch: pytest.MonkeyPatch, artifact_root: Path) -> None:
    _clear_rcp_environment(monkeypatch)
    monkeypatch.setenv("RCP_ENV", "test")
    monkeypatch.setenv("RCP_LOG_LEVEL", "INFO")
    monkeypatch.setenv("RCP_DATABASE_HOST", "db")
    monkeypatch.setenv("RCP_DATABASE_PORT", "5432")
    monkeypatch.setenv("RCP_DATABASE_NAME", "robot_platform")
    monkeypatch.setenv("RCP_DATABASE_USER", "robot_app")
    monkeypatch.setenv("RCP_DATABASE_PASSWORD", "test-password")
    monkeypatch.setenv("RCP_ARTIFACT_ROOT", str(artifact_root))
    monkeypatch.setenv("RCP_API_BASE_URL", "http://api:8000")
    monkeypatch.setenv("RCP_WORKER_ID", "local-simulator-1")
    monkeypatch.setenv("RCP_RUN_LEASE_SECONDS", "30")
    monkeypatch.setenv("RCP_RUN_HEARTBEAT_SECONDS", "10")
    monkeypatch.setenv("RCP_SIMULATION_GUI", "false")


def test_settings_load_from_rcp_aliases(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    artifact_root = tmp_path / "artifacts"
    _set_required_environment(monkeypatch, artifact_root)
    settings = load_settings()
    assert settings.env is RuntimeEnv.TEST
    assert settings.log_level is LogLevel.INFO
    assert settings.database_host == "db"
    assert settings.database_port == 5432
    assert settings.database_name == "robot_platform"
    assert settings.database_user == "robot_app"
    assert settings.database_password.get_secret_value() == "test-password"
    assert settings.artifact_root == artifact_root
    assert settings.api_base_url == "http://api:8000"
    assert settings.worker_id == "local-simulator-1"
    assert settings.run_lease_seconds == 30
    assert settings.run_heartbeat_seconds == 10
    assert settings.simulation_gui is False


def test_settings_use_documented_lease_defaults(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    artifact_root = tmp_path / "artifacts"
    _set_required_environment(monkeypatch, artifact_root)
    monkeypatch.delenv("RCP_RUN_LEASE_SECONDS")
    monkeypatch.delenv("RCP_RUN_HEARTBEAT_SECONDS")
    settings = load_settings()
    assert settings.run_lease_seconds == 30
    assert settings.run_heartbeat_seconds == 10


def test_missing_required_setting_fails_with_rcp_name(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    _set_required_environment(monkeypatch, tmp_path / "artifacts")
    monkeypatch.delenv("RCP_DATABASE_PASSWORD")
    with pytest.raises(ConfigurationError, match="missing RCP_DATABASE_PASSWORD"):
        load_settings()


def test_password_and_dsn_are_not_exposed_in_repr(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    _set_required_environment(monkeypatch, tmp_path / "artifacts")
    settings = load_settings()
    rendered = repr(settings)
    assert "test-password" not in rendered
    dsn = settings.database_dsn().get_secret_value()
    assert "test-password" in dsn
    assert "test-password" not in repr(settings.database_dsn())
    assert isinstance(settings, Settings)


def test_relative_artifact_root_is_rejected(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    _set_required_environment(monkeypatch, tmp_path / "artifacts")
    monkeypatch.setenv("RCP_ARTIFACT_ROOT", "artifacts")
    with pytest.raises(ConfigurationError, match="RCP_ARTIFACT_ROOT is invalid"):
        load_settings()
