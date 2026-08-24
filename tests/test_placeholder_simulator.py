from __future__ import annotations

import threading
import time
from pathlib import Path

from robot_control_platform_simulator.worker import run_placeholder


def test_placeholder_loop_writes_heartbeat_and_stops(tmp_path: Path) -> None:
    heartbeat = tmp_path / "healthy"
    stop_event = threading.Event()
    thread = threading.Thread(
        target=run_placeholder,
        kwargs={
            "stop_event": stop_event,
            "heartbeat_path": heartbeat,
            "interval_seconds": 0.05,
        },
        daemon=True,
    )
    thread.start()
    try:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not heartbeat.is_file():
            time.sleep(0.02)
        assert heartbeat.is_file()
        assert heartbeat.read_text(encoding="utf-8") == "ok\n"
    finally:
        stop_event.set()
        thread.join(timeout=2)
    assert not thread.is_alive()
    assert not heartbeat.exists()
