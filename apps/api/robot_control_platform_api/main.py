"""Placeholder API process for the four-service Compose shell."""

from __future__ import annotations

import signal
import sys

from robot_control_platform_common.config import ConfigurationError, load_settings
from robot_control_platform_common.logging import configure_logging, get_logger

from robot_control_platform_api.health import make_server


def serve(host: str = "0.0.0.0", port: int = 8000) -> None:
    """Load settings, then serve health endpoints until SIGTERM or SIGINT."""

    settings = load_settings()
    configure_logging("api", settings.log_level.value)
    logger = get_logger("api")
    logger.info("api_placeholder_started")
    httpd = make_server(host, port)

    def _stop(_signum: int, _frame: object | None) -> None:
        logger.info("api_placeholder_stopping")
        httpd.shutdown()

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    httpd.serve_forever()
    httpd.server_close()


def main() -> None:
    """Process entrypoint. Configuration errors exit without secret values."""

    try:
        serve()
    except ConfigurationError as exc:
        print(f"startup failed: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
