from __future__ import annotations

import pytest
from robot_control_platform_simulator.physics.client import PhysicsClient, SimulationError
from robot_control_platform_simulator.physics.robot import discover_controlled_joints
from robot_control_platform_simulator.physics.scene import (
    RESET_SMOKE_COUNT,
    ROBOT_BODY_NAME,
    SCENE_BODY_NAMES,
    WorkcellScene,
    WorkcellSnapshot,
    run_reset_smoke,
)


@pytest.fixture(autouse=True)
def require_pybullet() -> None:
    pytest.importorskip("pybullet")


def test_physics_client_uses_direct_mode_and_cleans_up() -> None:
    client = PhysicsClient(gui=False)
    assert client.gui is False
    with client:
        assert client.is_connected()
        assert client.connection_mode() == "direct"
        assert client.physics_client_id >= 0
    assert not client.is_connected()
    with pytest.raises(SimulationError, match="physics client is not connected"):
        _ = client.physics_client_id


def test_physics_client_disconnects_after_error() -> None:
    with pytest.raises(RuntimeError, match="boom"):
        with PhysicsClient(gui=False) as client:
            assert client.is_connected()
            raise RuntimeError("boom")
    with PhysicsClient(gui=False) as client:
        assert client.is_connected()
        assert client.connection_mode() == "direct"


def test_twenty_resets_preserve_bodies_names_joints_and_poses() -> None:
    snapshot = run_reset_smoke(reset_count=RESET_SMOKE_COUNT, gui=False)
    assert snapshot.body_count == len(SCENE_BODY_NAMES)
    assert snapshot.body_names == SCENE_BODY_NAMES
    assert snapshot.joint_names
    assert len(snapshot.joint_positions) == len(snapshot.joint_names)
    assert len(snapshot.body_poses) == len(SCENE_BODY_NAMES)
    assert "body_ids" not in WorkcellSnapshot.__dataclass_fields__
    assert "body_id" not in WorkcellSnapshot.__dataclass_fields__


def test_reset_restores_joint_commands_and_name_lookup() -> None:
    with PhysicsClient(gui=False) as client:
        scene = WorkcellScene(client)
        first = scene.reset()
        robot_id = scene.body_id(ROBOT_BODY_NAME)
        specs = discover_controlled_joints(client, robot_id)
        assert tuple(spec.name for spec in specs) == first.joint_names
        client.reset_joint_state(robot_id, specs[0].index, specs[0].rest_position + 0.2)
        perturbed = client.get_joint_position(robot_id, specs[0].index)
        assert perturbed != first.joint_positions[0]
        restored = scene.reset()
        assert restored.matches(first)
        assert scene.body_id("table") >= 0
        with pytest.raises(SimulationError, match="unknown workcell body: missing"):
            scene.body_id("missing")
