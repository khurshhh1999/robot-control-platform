"""Pinned-container smoke: same seed must yield the same structured scenario hash."""

from __future__ import annotations

import sys
from collections.abc import Sequence

from robot_control_platform_simulator.scenarios.generator import (
    ScenarioGenerationError,
    generate_scenario,
)
from robot_control_platform_simulator.scenarios.golden import GOLDEN_CHECKSUMS, GOLDEN_SEEDS
from robot_control_platform_simulator.scenarios.serializer import (
    deserialize_scenario,
    scenario_checksum,
    serialize_scenario,
)


def main(argv: Sequence[str] | None = None) -> int:
    del argv
    if not GOLDEN_CHECKSUMS:
        print("scenario_smoke_failed: golden checksums are not recorded", file=sys.stderr)
        return 1
    try:
        for seed in GOLDEN_SEEDS:
            scenario = generate_scenario(seed)
            checksum = scenario_checksum(scenario)
            expected = GOLDEN_CHECKSUMS[seed]
            restored = deserialize_scenario(serialize_scenario(scenario))
            if checksum != expected:
                print(
                    f"scenario_smoke_failed seed={seed} checksum={checksum} expected={expected}",
                    file=sys.stderr,
                )
                return 1
            if restored.sha256_hex() != checksum:
                print(
                    f"scenario_smoke_failed seed={seed} round-trip checksum mismatch",
                    file=sys.stderr,
                )
                return 1
            print(
                "scenario_smoke_ok"
                f" seed={seed}"
                f" checksum={checksum}"
                f" object={scenario.object_type.name}"
                f" target={scenario.target_bin}"
            )
    except (KeyError, TypeError, ValueError, OSError, ScenarioGenerationError) as exc:
        print(f"scenario_smoke_failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
