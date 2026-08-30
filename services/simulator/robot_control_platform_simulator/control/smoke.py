"""Pinned-container smoke: centered-cube pick/place against the owner reliability gate."""

from __future__ import annotations

import argparse
import sys
from collections.abc import Sequence
from pathlib import Path

from robot_control_platform_simulator.control.reliability import (
    evaluate_reliability_gate,
    load_motion_reliability_gate,
)
from robot_control_platform_simulator.physics.client import SimulationError


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run centered-cube motion reliability.")
    parser.add_argument("--config", required=True, type=Path)
    args = parser.parse_args(list(argv) if argv is not None else None)
    try:
        gate = load_motion_reliability_gate(args.config)
        report = evaluate_reliability_gate(gate, gui=False)
    except (OSError, SimulationError, ValueError, TypeError) as exc:
        print(f"motion_reliability_failed: {exc}", file=sys.stderr)
        return 1
    if not report.passed:
        print(
            f"motion_reliability_failed successes={report.successes} trials={report.trial_count}",
            file=sys.stderr,
        )
        return 1
    print(f"motion_reliability_ok successes={report.successes} trials={report.trial_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
