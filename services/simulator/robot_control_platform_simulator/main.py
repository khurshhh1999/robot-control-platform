"""Placeholder simulator process for the four-service Compose shell."""

from __future__ import annotations

import signal
import sys
import threading

from robot_control_platform_common.config import ConfigurationError, load_settings
from robot_control_platform_common.logging import configure_logging, get_logger

from robot_control_platform_simulator.worker import run_placeholder


def main() -> None:
    """Load settings, prove the artifact root is writable, then idle until SIGTERM."""

    try:
        settings = load_settings()
    except ConfigurationError as exc:
        print(f"startup failed: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc

    configure_logging("simulator", settings.log_level.value)
    logger = get_logger("simulator")
    try:
        settings.artifact_root.mkdir(parents=True, exist_ok=True)
        probe = settings.artifact_root / ".compose-shell-writable"
        probe.write_text("ok\n", encoding="utf-8")
        probe.unlink()
    except OSError as exc:
        logger.error("artifact_root_not_writable")
        raise SystemExit(1) from exc

    logger.info("simulator_placeholder_started")
    stop_event = threading.Event()

    def _stop(_signum: int, _frame: object | None) -> None:
        logger.info("simulator_placeholder_stopping")
        stop_event.set()

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    run_placeholder(stop_event)
