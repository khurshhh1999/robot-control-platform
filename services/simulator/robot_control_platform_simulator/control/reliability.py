"""Load the owner reliability gate from ignored configuration. Never publish the threshold."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Final

from robot_control_platform_simulator.control.pick_place import (
    require_pick_place_primitives,
    run_centered_cube_pick_place,
)
from robot_control_platform_simulator.domain.models import require_finite
from robot_control_platform_simulator.physics.client import PhysicsClient, SimulationError
from robot_control_platform_simulator.physics.scene import WorkcellScene

RELIABILITY_SCHEMA_VERSION: Final[str] = "1"
_GATE_KEYS: Final[frozenset[str]] = frozenset({"schema_version", "success_rate_min", "trial_count"})


@dataclass(frozen=True)
class MotionReliabilityGate:
    """Owner-supplied centered-cube reliability threshold. Do not log the rate."""

    success_rate_min: float
    trial_count: int


@dataclass(frozen=True)
class ReliabilityReport:
    successes: int
    trial_count: int
    passed: bool


def load_motion_reliability_gate(path: Path) -> MotionReliabilityGate:
    """Read an ignored JSON gate file. Error messages must not include filesystem paths."""

    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise SimulationError("reliability configuration is not readable") from exc
    try:
        payload = json.loads(text)
    except json.JSONDecodeError as exc:
        raise SimulationError("reliability configuration is invalid") from exc
    if not isinstance(payload, dict):
        raise SimulationError("reliability configuration is invalid")
    if set(payload.keys()) != _GATE_KEYS:
        raise SimulationError("reliability configuration keys are invalid")
    if payload["schema_version"] != RELIABILITY_SCHEMA_VERSION:
        raise SimulationError("reliability configuration schema is unsupported")
    trial_count = payload["trial_count"]
    if isinstance(trial_count, bool) or not isinstance(trial_count, int) or trial_count <= 0:
        raise SimulationError("reliability configuration trial_count is invalid")
    try:
        rate = require_finite("success_rate_min", payload["success_rate_min"])
    except ValueError as exc:
        raise SimulationError("reliability configuration success_rate_min is invalid") from exc
    if rate <= 0.0 or rate > 1.0:
        raise SimulationError("reliability configuration success_rate_min is invalid")
    return MotionReliabilityGate(success_rate_min=rate, trial_count=trial_count)


def evaluate_reliability_gate(
    gate: MotionReliabilityGate, *, gui: bool = False
) -> ReliabilityReport:
    successes = 0
    with PhysicsClient(gui=gui) as client:
        scene = WorkcellScene(client)
        for _ in range(gate.trial_count):
            try:
                result = run_centered_cube_pick_place(client, scene)
                if result.succeeded:
                    require_pick_place_primitives(result.actions)
                    successes += 1
            except SimulationError:
                continue
    passed = successes / gate.trial_count >= gate.success_rate_min
    return ReliabilityReport(successes=successes, trial_count=gate.trial_count, passed=passed)
