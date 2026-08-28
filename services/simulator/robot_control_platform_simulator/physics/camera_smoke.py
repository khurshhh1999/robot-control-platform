"""Write one headless review-camera PNG outside the repository."""

from __future__ import annotations

import argparse
import sys
from collections.abc import Sequence
from pathlib import Path

from robot_control_platform_simulator.physics.camera import (
    CameraCapture,
    capture_rgb_frame,
    decode_rgb_png,
    default_camera_config,
    write_rgb_png,
)
from robot_control_platform_simulator.physics.client import PhysicsClient, SimulationError
from robot_control_platform_simulator.physics.scene import WorkcellScene


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Capture one review-camera RGB PNG.")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(list(argv) if argv is not None else None)
    try:
        capture = _capture()
        write_rgb_png(args.output, capture.png_bytes)
        decoded = decode_rgb_png(capture.png_bytes)
    except (OSError, SimulationError, ValueError) as exc:
        print(f"camera_smoke_failed: {exc}", file=sys.stderr)
        return 1
    print(
        "camera_smoke_ok"
        f" width={decoded.size[0]}"
        f" height={decoded.size[1]}"
        f" mode={decoded.mode}"
        f" renderer={capture.renderer}"
        f" checksum={capture.camera_checksum}"
        f" nonblank={capture.nonblank_pixel_count}"
    )
    return 0


def _capture() -> CameraCapture:
    config = default_camera_config()
    with PhysicsClient(gui=False) as client:
        WorkcellScene(client).reset()
        return capture_rgb_frame(client, config)


if __name__ == "__main__":
    raise SystemExit(main())
