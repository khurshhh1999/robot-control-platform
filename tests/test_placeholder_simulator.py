"""Liveness file used by the simulator health check."""

from __future__ import annotations

from pathlib import Path

from robot_control_platform_simulator.worker import clear_liveness, write_liveness


def test_liveness_file_round_trip(tmp_path: Path) -> None:
    path = tmp_path / "healthy"
    write_liveness(path)
    assert path.is_file()
    assert path.read_text(encoding="utf-8") == "ok\n"
    clear_liveness(path)
    assert not path.exists()
    clear_liveness(path)
