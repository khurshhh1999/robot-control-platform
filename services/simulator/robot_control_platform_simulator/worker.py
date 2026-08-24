"""Placeholder simulator loop for the Compose worker shell."""

from __future__ import annotations

import threading
from pathlib import Path

HEARTBEAT_PATH = Path("/tmp/robot-platform-healthy")


def run_placeholder(
    stop_event: threading.Event,
    heartbeat_path: Path = HEARTBEAT_PATH,
    interval_seconds: float = 2.0,
) -> None:
    """Write a heartbeat file until ``stop_event`` is set, then remove it."""

    while not stop_event.is_set():
        heartbeat_path.write_text("ok\n", encoding="utf-8")
        stop_event.wait(interval_seconds)
    heartbeat_path.unlink(missing_ok=True)
