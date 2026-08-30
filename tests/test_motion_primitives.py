from __future__ import annotations

import json
import math
from pathlib import Path
from subprocess import check_output
from unittest.mock import MagicMock

import pytest
from robot_control_platform_simulator.control.actions import (
    PHYSICS_CONTROL_HZ,
    PHYSICS_STEPS_PER_CONTROL,
    ActionResult,
    ActionStatus,
    MotionCommand,
    MotionPrimitive,
    _compensated_pose_target,
    control_period_seconds,
)
from robot_control_platform_simulator.control.pick_place import (
    object_in_target_region,
    place_target_pose,
)
from robot_control_platform_simulator.control.reliability import load_motion_reliability_gate
from robot_control_platform_simulator.domain.enums import CanonicalUnit
from robot_control_platform_simulator.domain.models import Action, Pose, QuaternionXYZW, Vector3
from robot_control_platform_simulator.physics.client import (
    PHYSICS_TIMESTEP_SECONDS,
    WORLD_FRAME,
    JointRecord,
    SimulationError,
)
from robot_control_platform_simulator.physics.robot import (
    ARM_LINK_NAMES,
    ARM_WRIST_YAW_RADIANS,
    END_EFFECTOR_LINK_NAME,
    GRIPPER_FINGER_LINK_NAMES,
    GRIPPER_TIP_LINK_NAMES,
    JointRole,
    JointSpec,
    RobotLayout,
    apply_wrist_yaw,
    discover_and_validate_robot_layout,
    validate_ik_solution,
)

REPO_ROOT = Path(__file__).resolve().parents[1]


def _pose(x: float, y: float, z: float) -> Pose:
    return Pose(
        position_meters=Vector3(x=x, y=y, z=z),
        orientation_xyzw=QuaternionXYZW(x=0.0, y=0.0, z=0.0, w=1.0),
        frame=WORLD_FRAME,
    )


def _spec(
    index: int,
    *,
    lower: float = -1.0,
    upper: float = 1.0,
    rest: float = 0.0,
    role: JointRole = JointRole.ARM,
) -> JointSpec:
    return JointSpec(
        index=index,
        name=f"joint_{index}",
        link_name=f"link_{index}",
        rest_position=rest,
        force_newtons=50.0,
        position_unit=CanonicalUnit.RADIANS,
        lower_limit=lower,
        upper_limit=upper,
        role=role,
    )


def _record(
    index: int,
    link_name: str,
    *,
    lower: float = -2.0,
    upper: float = 2.0,
    fixed: bool = False,
    force: float = 80.0,
) -> JointRecord:
    return JointRecord(
        index=index,
        name=f"joint_{link_name}",
        link_name=link_name,
        joint_type=4 if fixed else 0,
        lower_limit=lower,
        upper_limit=upper,
        max_force_newtons=force,
        rest_position=0.0,
        is_prismatic=False,
    )


def test_control_update_uses_four_physics_steps() -> None:
    assert PHYSICS_STEPS_PER_CONTROL == 4
    assert PHYSICS_CONTROL_HZ == 60
    assert PHYSICS_STEPS_PER_CONTROL * PHYSICS_CONTROL_HZ == 240
    assert control_period_seconds() == pytest.approx(1.0 / 60.0)
    assert PHYSICS_TIMESTEP_SECONDS * PHYSICS_STEPS_PER_CONTROL == pytest.approx(
        control_period_seconds()
    )


def test_motion_primitives_are_the_typed_actions() -> None:
    assert set(MotionPrimitive) == {
        "move_end_effector",
        "open",
        "close",
        "hold",
        "retract",
        "settle",
    }
    for primitive in MotionPrimitive:
        command = MotionCommand(primitive, timeout_seconds=1.0, tolerance=0.01)
        assert command.timeout_seconds == 1.0
        assert command.tolerance == 0.01
        assert command.primitive is primitive


def test_motion_command_requires_timeout_and_tolerance() -> None:
    with pytest.raises(ValueError, match="timeout_seconds must be positive"):
        MotionCommand(MotionPrimitive.HOLD, timeout_seconds=0.0, tolerance=0.01)
    with pytest.raises(ValueError, match="tolerance must be positive"):
        MotionCommand(MotionPrimitive.HOLD, timeout_seconds=1.0, tolerance=0.0)
    with pytest.raises(ValueError, match="timeout_seconds must be a finite number"):
        MotionCommand(MotionPrimitive.OPEN, timeout_seconds=math.inf, tolerance=0.01)


def test_action_status_is_typed() -> None:
    assert set(ActionStatus) == {"succeeded", "timeout", "ik_rejected"}


def test_action_result_records_commanded_and_observed_simulation_time() -> None:
    result = ActionResult(
        primitive=MotionPrimitive.HOLD,
        status=ActionStatus.SUCCEEDED,
        commanded=Action(
            name=MotionPrimitive.HOLD.value,
            simulation_time_seconds=0.25,
            target_pose=None,
            joint_targets=(),
        ),
        observed_joints=(),
        observed_ee_pose=_pose(0.1, 0.0, 0.9),
        observed_simulation_time_seconds=0.50,
        control_updates=15,
    )
    assert result.commanded.simulation_time_seconds == 0.25
    assert result.observed_simulation_time_seconds == 0.50
    assert result.observed_simulation_time_seconds > result.commanded.simulation_time_seconds
    assert result.control_updates == 15
    assert result.succeeded


def test_ik_solution_rejects_nonfinite_and_out_of_limit_values() -> None:
    specs = (_spec(0, lower=-0.5, upper=0.5), _spec(1, lower=-1.0, upper=1.0))
    assert validate_ik_solution((0.1, -0.2), specs) == (0.1, -0.2)
    with pytest.raises(SimulationError, match="nonfinite"):
        validate_ik_solution((math.nan, 0.0), specs)
    with pytest.raises(SimulationError, match="nonfinite"):
        validate_ik_solution((math.inf, 0.0), specs)
    with pytest.raises(SimulationError, match="out of joint limits"):
        validate_ik_solution((-0.9, 0.0), specs)
    with pytest.raises(SimulationError, match="out of joint limits"):
        validate_ik_solution((0.0, 1.5), specs)


def test_ik_solution_within_limit_tolerance_is_clipped() -> None:
    specs = (_spec(0, lower=-0.5, upper=0.5),)
    clipped = validate_ik_solution((0.5 + 1e-9,), specs)
    assert clipped[0] == 0.5


def test_wrist_yaw_aligns_parallel_jaws() -> None:
    layout = RobotLayout(
        arm_joints=tuple(_spec(index) for index in range(7)),
        gripper_joints=(_spec(8, role=JointRole.GRIPPER),),
        end_effector_link_index=6,
        end_effector_link_name=END_EFFECTOR_LINK_NAME,
        ik_damping=(0.1,) * 8,
        ik_rest_poses=(0.0,) * 8,
        ik_max_iterations=100,
        ik_residual_threshold=1e-4,
    )
    commanded = apply_wrist_yaw({index: 0.1 * index for index in range(7)}, layout)
    assert commanded[6] == ARM_WRIST_YAW_RADIANS
    assert commanded[0] == 0.0
    assert commanded[5] == 0.5


def test_pose_tracking_requests_residual_error_compensation() -> None:
    target = _pose(0.4, 0.1, 0.9)
    observed = _pose(0.3, 0.1, 0.8)
    compensated = _compensated_pose_target(target, observed)
    assert compensated.position_meters.x == pytest.approx(0.5)
    assert compensated.position_meters.y == pytest.approx(0.1)
    assert compensated.position_meters.z == pytest.approx(1.0)
    assert compensated.orientation_xyzw == target.orientation_xyzw
    assert compensated.frame == target.frame


def test_layout_validation_requires_configured_arm_and_gripper_links() -> None:
    client = MagicMock()
    client.is_fixed_joint.side_effect = lambda joint_type: joint_type == 4
    client.joint_records.return_value = ()
    with pytest.raises(SimulationError, match="configured robot link is missing"):
        discover_and_validate_robot_layout(client, 1)


def test_layout_validation_rejects_unknown_controlled_joints() -> None:
    client = MagicMock()
    client.is_fixed_joint.side_effect = lambda joint_type: joint_type == 4
    records = _valid_layout_records() + (_record(20, "extra_dof", lower=-1.0, upper=1.0),)
    client.joint_records.return_value = records
    with pytest.raises(SimulationError, match="unexpected controlled joint: extra_dof"):
        discover_and_validate_robot_layout(client, 1)


def test_layout_validation_accepts_configured_arm_and_gripper() -> None:
    client = MagicMock()
    client.is_fixed_joint.side_effect = lambda joint_type: joint_type == 4
    client.joint_records.return_value = _valid_layout_records()
    layout = discover_and_validate_robot_layout(client, 1)
    assert tuple(spec.link_name for spec in layout.arm_joints) == ARM_LINK_NAMES
    assert layout.end_effector_link_name == END_EFFECTOR_LINK_NAME
    assert layout.end_effector_link_index == 6
    assert layout.ik_max_iterations > 0
    assert len(layout.ik_damping) == len(layout.controlled_joints)
    assert len(layout.ik_rest_poses) == len(layout.controlled_joints)
    assert tuple(spec.link_name for spec in layout.finger_joints) == GRIPPER_FINGER_LINK_NAMES
    assert tuple(spec.link_name for spec in layout.tip_joints) == GRIPPER_TIP_LINK_NAMES
    for spec in layout.arm_joints:
        assert spec.lower_limit < spec.upper_limit
        assert spec.lower_limit <= spec.rest_position <= spec.upper_limit


def test_centered_cube_place_region_uses_meters() -> None:
    start = _pose(0.5, 0.0, 0.65)
    target = place_target_pose(start)
    assert object_in_target_region(target, target)
    inside = _pose(
        target.position_meters.x + 0.01, target.position_meters.y, target.position_meters.z
    )
    assert object_in_target_region(inside, target)
    outside = _pose(
        target.position_meters.x + 0.2, target.position_meters.y, target.position_meters.z
    )
    assert not object_in_target_region(outside, target)


def test_reliability_gate_loader_reads_ignored_configuration(tmp_path: Path) -> None:
    path = tmp_path / "gate.json"
    path.write_text(
        json.dumps({"schema_version": "1", "success_rate_min": 0.25, "trial_count": 4}),
        encoding="utf-8",
    )
    gate = load_motion_reliability_gate(path)
    assert gate.trial_count == 4
    assert gate.success_rate_min == 0.25


def test_reliability_gate_loader_rejects_invalid_inputs(tmp_path: Path) -> None:
    missing = tmp_path / "missing.json"
    with pytest.raises(SimulationError, match="not readable") as unread:
        load_motion_reliability_gate(missing)
    assert str(missing) not in str(unread.value)

    broken = tmp_path / "broken.json"
    broken.write_text("{", encoding="utf-8")
    with pytest.raises(SimulationError, match="invalid"):
        load_motion_reliability_gate(broken)

    extra = tmp_path / "extra.json"
    extra.write_text(
        json.dumps(
            {
                "schema_version": "1",
                "success_rate_min": 0.25,
                "trial_count": 4,
                "extra": True,
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(SimulationError, match="keys are invalid"):
        load_motion_reliability_gate(extra)

    bad_count = tmp_path / "count.json"
    bad_count.write_text(
        json.dumps({"schema_version": "1", "success_rate_min": 0.25, "trial_count": True}),
        encoding="utf-8",
    )
    with pytest.raises(SimulationError, match="trial_count is invalid"):
        load_motion_reliability_gate(bad_count)

    bad_rate = tmp_path / "rate.json"
    bad_rate.write_text(
        json.dumps({"schema_version": "1", "success_rate_min": 0.0, "trial_count": 4}),
        encoding="utf-8",
    )
    with pytest.raises(SimulationError, match="success_rate_min is invalid"):
        load_motion_reliability_gate(bad_rate)


def test_reliability_modules_do_not_embed_private_paths() -> None:
    control_dir = (
        REPO_ROOT / "services" / "simulator" / "robot_control_platform_simulator" / "control"
    )
    for path in control_dir.glob("*.py"):
        text = path.read_text(encoding="utf-8")
        assert ".project-private" not in text
        assert "motion-reliability.json" not in text


def test_private_reliability_file_is_ignored() -> None:
    relative = ".project-private/motion-reliability.json"
    output = check_output(["git", "check-ignore", "-v", relative], cwd=REPO_ROOT, text=True)
    assert ".project-private" in output


def test_simulation_time_advances_four_steps_per_control_update() -> None:
    pytest.importorskip("pybullet")
    from robot_control_platform_simulator.control.actions import MotionController
    from robot_control_platform_simulator.physics.client import PhysicsClient
    from robot_control_platform_simulator.physics.scene import ROBOT_BODY_NAME, WorkcellScene

    with PhysicsClient(gui=False) as client:
        scene = WorkcellScene(client)
        scene.reset()
        assert client.simulation_time_seconds() == 0.0
        layout = discover_and_validate_robot_layout(client, scene.body_id(ROBOT_BODY_NAME))
        controller = MotionController(client, scene.body_id(ROBOT_BODY_NAME), layout)
        controller.advance_control()
        assert client.simulation_time_seconds() == pytest.approx(control_period_seconds())
        assert layout.end_effector_link_name == END_EFFECTOR_LINK_NAME
        assert len(layout.arm_joints) == 7


def _valid_layout_records() -> tuple[JointRecord, ...]:
    records: list[JointRecord] = []
    for index, name in enumerate(ARM_LINK_NAMES):
        records.append(_record(index, name, lower=-2.9, upper=2.9, force=300.0))
    next_index = len(ARM_LINK_NAMES)
    records.append(_record(next_index, "base_link", lower=-1.0, upper=1.0, force=50.0))
    next_index += 1
    for name in GRIPPER_FINGER_LINK_NAMES:
        records.append(_record(next_index, name, lower=-1.0, upper=1.0, force=100.0))
        next_index += 1
        records.append(_record(next_index, f"{name}_base", fixed=True, force=0.0))
        next_index += 1
    for name in GRIPPER_TIP_LINK_NAMES:
        records.append(_record(next_index, name, lower=-1.0, upper=1.0, force=0.0))
        next_index += 1
    return tuple(records)
