"""Headless workcell smoke test for the simulator container."""

from __future__ import annotations

from robot_control_platform_simulator.physics.scene import RESET_SMOKE_COUNT, run_reset_smoke


def main() -> None:
    snapshot = run_reset_smoke(reset_count=RESET_SMOKE_COUNT, gui=False)
    print(
        "workcell_reset_smoke_ok"
        f" bodies={snapshot.body_count}"
        f" joints={len(snapshot.joint_names)}"
        f" resets={RESET_SMOKE_COUNT}"
    )


if __name__ == "__main__":
    main()
