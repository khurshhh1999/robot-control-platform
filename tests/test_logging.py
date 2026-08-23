import json
from collections.abc import Iterator
from typing import Any

import pytest
from pydantic import SecretStr
from robot_control_platform_common.logging import (
    bind_log_context,
    clear_log_context,
    configure_logging,
    get_logger,
)


def _parse_last_json(stdout: str) -> dict[str, Any]:
    lines = [line for line in stdout.splitlines() if line.strip()]
    assert lines, "expected JSON log output on stdout"
    payload = json.loads(lines[-1])
    assert isinstance(payload, dict)
    return payload


@pytest.fixture
def json_logger() -> Iterator[None]:
    configure_logging("api", "INFO")
    yield
    clear_log_context()


def test_configure_logging_writes_json_to_stdout(
    json_logger: None, capsys: pytest.CaptureFixture[str]
) -> None:
    get_logger().info("trial_started")
    payload = _parse_last_json(capsys.readouterr().out)
    assert payload["event"] == "trial_started"
    assert payload["service"] == "api"
    assert payload["level"] == "info"
    assert isinstance(payload["timestamp"], str)
    assert payload["timestamp"].endswith("Z")
    assert "+" not in payload["timestamp"]
    assert payload["request_id"] is None
    assert payload["experiment_id"] is None
    assert payload["run_id"] is None
    assert payload["trial_id"] is None


def test_log_context_binds_request_experiment_run_and_trial_ids(
    json_logger: None, capsys: pytest.CaptureFixture[str]
) -> None:
    bind_log_context(
        request_id="req-1",
        experiment_id="exp-1",
        run_id="run-1",
        trial_id="trial-1",
    )
    get_logger().info("context_bound")
    payload = _parse_last_json(capsys.readouterr().out)
    assert payload["request_id"] == "req-1"
    assert payload["experiment_id"] == "exp-1"
    assert payload["run_id"] == "run-1"
    assert payload["trial_id"] == "trial-1"


def test_sensitive_keys_are_redacted(json_logger: None, capsys: pytest.CaptureFixture[str]) -> None:
    secret_value = "literal-secret-value"
    get_logger().info(
        "sensitive_event",
        password=secret_value,
        database_password=secret_value,
        api_secret=secret_value,
        access_token=secret_value,
        Authorization=f"Bearer {secret_value}",
        set_cookie=secret_value,
        database_dsn=f"postgresql://robot_app:{secret_value}@db:5432/robot_platform",
        dsn=SecretStr(f"postgresql://robot_app:{secret_value}@db:5432/robot_platform"),
        headers={"Authorization": f"Bearer {secret_value}", "X-Request-Id": "req-1"},
        nested={"cookie": secret_value, "trial_id": "trial-1"},
        trial_id="trial-1",
    )
    stdout = capsys.readouterr().out
    payload = _parse_last_json(stdout)
    assert secret_value not in stdout
    assert payload["password"] == "[REDACTED]"
    assert payload["database_password"] == "[REDACTED]"
    assert payload["api_secret"] == "[REDACTED]"
    assert payload["access_token"] == "[REDACTED]"
    assert payload["Authorization"] == "[REDACTED]"
    assert payload["set_cookie"] == "[REDACTED]"
    assert payload["database_dsn"] == "[REDACTED]"
    assert payload["dsn"] == "[REDACTED]"
    headers = payload["headers"]
    assert isinstance(headers, dict)
    assert headers["Authorization"] == "[REDACTED]"
    assert headers["X-Request-Id"] == "req-1"
    nested = payload["nested"]
    assert isinstance(nested, dict)
    assert nested["cookie"] == "[REDACTED]"
    assert nested["trial_id"] == "trial-1"
    assert payload["trial_id"] == "trial-1"
