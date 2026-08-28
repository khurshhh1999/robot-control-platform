from __future__ import annotations

from pathlib import Path

import pytest
from robot_control_platform_simulator.domain.models import Vector3
from robot_control_platform_simulator.physics.client import (
    PhysicsConfig,
    SimulationError,
    connection_mode_for_gui,
    default_physics_config,
)
from robot_control_platform_simulator.physics.scene import (
    ALLOWED_ASSETS,
    SCENE_BODY_NAMES,
    default_scene_config,
    resolve_allowlisted_asset,
)


def test_direct_is_the_default_connection_mode() -> None:
    assert connection_mode_for_gui(False) == "direct"
    assert connection_mode_for_gui(True) == "gui"


def test_allowlisted_assets_are_relative_and_named() -> None:
    assert set(ALLOWED_ASSETS) == {"plane", "table", "kuka_iiwa_gripper"}
    assert ALLOWED_ASSETS["plane"] == "plane.urdf"
    assert ALLOWED_ASSETS["table"] == "table/table.urdf"
    assert ALLOWED_ASSETS["kuka_iiwa_gripper"] == "kuka_iiwa/kuka_with_gripper2.sdf"
    for relative in ALLOWED_ASSETS.values():
        path = Path(relative)
        assert not path.is_absolute()
        assert ".." not in path.parts
    assert SCENE_BODY_NAMES == (
        "plane",
        "table",
        "kuka_iiwa",
        "pickup_region",
        "bin_red",
        "bin_green",
        "bin_blue",
        "bin_yellow",
    )


def test_resolve_allowlisted_asset_rejects_unknown_and_missing(tmp_path: Path) -> None:
    root = tmp_path / "pybullet_data"
    root.mkdir()
    with pytest.raises(SimulationError, match="asset is not allowlisted: cube"):
        resolve_allowlisted_asset("cube", root)
    with pytest.raises(SimulationError, match="allowlisted asset is missing: plane") as missing:
        resolve_allowlisted_asset("plane", root)
    assert str(root) not in str(missing.value)
    assert "/" not in str(missing.value)


def test_resolve_allowlisted_asset_stays_under_data_root(tmp_path: Path) -> None:
    root = tmp_path / "pybullet_data"
    (root / "table").mkdir(parents=True)
    (root / "table" / "table.urdf").write_text("table\n", encoding="utf-8")
    resolved = resolve_allowlisted_asset("table", root)
    assert resolved == (root / "table" / "table.urdf").resolve()
    assert resolved.is_relative_to(root.resolve())


def test_resolve_allowlisted_asset_rejects_symlink_escape(tmp_path: Path) -> None:
    root = tmp_path / "pybullet_data"
    outside = tmp_path / "outside.urdf"
    outside.write_text("nope\n", encoding="utf-8")
    (root / "table").mkdir(parents=True)
    (root / "table" / "table.urdf").symlink_to(outside)
    with pytest.raises(SimulationError, match="asset path is invalid: table") as escaped:
        resolve_allowlisted_asset("table", root)
    assert str(outside) not in str(escaped.value)
    assert str(root) not in str(escaped.value)


def test_physics_and_scene_checksums_are_stable() -> None:
    physics = default_physics_config()
    scene = default_scene_config()
    assert physics.sha256_hex() == default_physics_config().sha256_hex()
    assert scene.sha256_hex() == default_scene_config().sha256_hex()
    assert physics.sha256_hex() != scene.sha256_hex()
    assert physics.timestep_seconds == 1.0 / 240.0
    assert physics.solver_iterations == 150
    assert physics.deterministic_overlapping_pairs is True
    assert physics.gravity_meters_per_second_squared == Vector3(x=0.0, y=0.0, z=-9.81)


def test_invalid_physics_config_is_rejected() -> None:
    with pytest.raises(ValueError, match="timestep_seconds must be positive"):
        PhysicsConfig(
            gravity_meters_per_second_squared=Vector3(x=0.0, y=0.0, z=-9.81),
            timestep_seconds=0.0,
            solver_iterations=150,
            deterministic_overlapping_pairs=True,
        )
    with pytest.raises(ValueError, match="solver_iterations must be a positive integer"):
        PhysicsConfig(
            gravity_meters_per_second_squared=Vector3(x=0.0, y=0.0, z=-9.81),
            timestep_seconds=1.0 / 240.0,
            solver_iterations=0,
            deterministic_overlapping_pairs=True,
        )
