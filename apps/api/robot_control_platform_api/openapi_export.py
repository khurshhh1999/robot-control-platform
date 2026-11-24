"""Export the versioned OpenAPI document without starting the API process."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

from pydantic import SecretStr
from robot_control_platform_common.config import LogLevel, RuntimeEnv, Settings

from robot_control_platform_api.main import create_app

# Container artifact root from the runtime contract. Export does not touch it.
_EXPORT_ARTIFACT_ROOT = Path("/var/lib/robot-platform/artifacts")
_EXPORT_PASSWORD = "openapi-export-placeholder"


def export_settings() -> Settings:
    """Return in-process settings for a test-configuration schema export."""

    return Settings(
        env=RuntimeEnv.TEST,
        log_level=LogLevel.WARNING,
        database_host="db",
        database_port=5432,
        database_name="robot_platform",
        database_user="robot_app",
        database_password=SecretStr(_EXPORT_PASSWORD),
        artifact_root=_EXPORT_ARTIFACT_ROOT,
        api_base_url="http://api:8000",
        worker_id="openapi-export",
        run_lease_seconds=30,
        run_heartbeat_seconds=10,
        simulation_gui=False,
    )


def build_openapi_document() -> dict[str, Any]:
    """Build the OpenAPI document from the test application.

    This calls ``FastAPI.openapi()`` only. It does not enter lifespan, open
    database connections, or bind a socket.
    """

    app = create_app(export_settings())
    document = app.openapi()
    if not isinstance(document, dict):
        msg = "OpenAPI export did not return an object"
        raise TypeError(msg)
    return document


def canonical_openapi_text(document: dict[str, Any]) -> str:
    """Serialize ``document`` with sorted keys and a trailing newline."""

    return (
        json.dumps(
            document,
            indent=2,
            sort_keys=True,
            ensure_ascii=False,
            allow_nan=False,
        )
        + "\n"
    )


def main(argv: list[str] | None = None) -> int:
    """Write the OpenAPI document or fail when the committed copy is stale."""

    parser = argparse.ArgumentParser(description="Export or check the OpenAPI document.")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--output", type=Path)
    group.add_argument("--check", type=Path)
    args = parser.parse_args(argv)
    text = canonical_openapi_text(build_openapi_document())
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
        return 0
    committed = args.check.read_text(encoding="utf-8")
    if committed != text:
        print(f"committed OpenAPI document is stale: {args.check}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
