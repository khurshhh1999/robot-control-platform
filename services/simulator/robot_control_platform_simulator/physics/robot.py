"""Discover, validate, and command the configured arm and gripper joints."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from enum import StrEnum
from typing import Final

from robot_control_platform_simulator.domain.enums import CanonicalUnit
from robot_control_platform_simulator.domain.models import JointState, Pose, Vector3, require_finite
from robot_control_platform_simulator.physics.client import (
    FALLBACK_JOINT_FORCE_NEWTONS,
    JointRecord,
    PhysicsClient,
    SimulationError,
)

ARM_LINK_NAMES: Final[tuple[str, ...]] = (
    "lbr_iiwa_link_1",
    "lbr_iiwa_link_2",
    "lbr_iiwa_link_3",
    "lbr_iiwa_link_4",
    "lbr_iiwa_link_5",
    "lbr_iiwa_link_6",
    "lbr_iiwa_link_7",
)
END_EFFECTOR_LINK_NAME: Final[str] = "lbr_iiwa_link_7"
GRIPPER_ROTATE_LINK_NAME: Final[str] = "base_link"
GRIPPER_FINGER_LINK_NAMES: Final[tuple[str, ...]] = ("left_finger", "right_finger")
GRIPPER_TIP_LINK_NAMES: Final[tuple[str, ...]] = ("left_finger_tip", "right_finger_tip")
KNOWN_GRIPPER_LINK_NAMES: Final[frozenset[str]] = frozenset(
    (GRIPPER_ROTATE_LINK_NAME, *GRIPPER_FINGER_LINK_NAMES, *GRIPPER_TIP_LINK_NAMES)
)
IK_DAMPING: Final[float] = 0.1
IK_MAX_ITERATIONS: Final[int] = 100
IK_RESIDUAL_THRESHOLD: Final[float] = 1e-4
IK_LIMIT_TOLERANCE: Final[float] = 1e-6
IK_REQUEST_BIAS_METERS: Final[Vector3] = Vector3(x=0.10, y=0.0, z=0.0)
ARM_WRIST_YAW_RADIANS: Final[float] = 0.5 * math.pi
ARM_IK_REST_POSITIONS: Final[tuple[float, ...]] = (
    0.0,
    0.0,
    0.0,
    -0.8,
    0.0,
    0.8,
    0.0,
)
GRIPPER_OPEN_RADIANS: Final[float] = 0.30
GRIPPER_CLOSED_RADIANS: Final[float] = 0.0
GRIPPER_FINGER_FORCE_NEWTONS: Final[float] = 40.0
GRIPPER_TIP_FORCE_NEWTONS: Final[float] = 5.0
GRIPPER_ROTATE_FORCE_NEWTONS: Final[float] = 50.0
GRIPPER_MAX_VELOCITY: Final[float] = 2.00


class JointRole(StrEnum):
    ARM = "arm"
    GRIPPER = "gripper"


@dataclass(frozen=True)
class JointSpec:
    """Controlled joint identity, limits, and rest command used during reset and IK."""

    index: int
    name: str
    link_name: str
    rest_position: float
    force_newtons: float
    position_unit: CanonicalUnit
    lower_limit: float
    upper_limit: float
    role: JointRole


@dataclass(frozen=True)
class RobotLayout:
    """Validated arm/gripper layout. Body ids are ephemeral and are not stored."""

    arm_joints: tuple[JointSpec, ...]
    gripper_joints: tuple[JointSpec, ...]
    end_effector_link_index: int
    end_effector_link_name: str
    ik_damping: tuple[float, ...]
    ik_rest_poses: tuple[float, ...]
    ik_max_iterations: int
    ik_residual_threshold: float

    @property
    def controlled_joints(self) -> tuple[JointSpec, ...]:
        return tuple(sorted((*self.arm_joints, *self.gripper_joints), key=lambda spec: spec.index))

    @property
    def finger_joints(self) -> tuple[JointSpec, ...]:
        return tuple(
            spec for spec in self.gripper_joints if spec.link_name in GRIPPER_FINGER_LINK_NAMES
        )

    @property
    def tip_joints(self) -> tuple[JointSpec, ...]:
        return tuple(
            spec for spec in self.gripper_joints if spec.link_name in GRIPPER_TIP_LINK_NAMES
        )


def discover_controlled_joints(client: PhysicsClient, body_id: int) -> tuple[JointSpec, ...]:
    """Return non-fixed joints. Body ids are ephemeral and must not be stored."""

    specs: list[JointSpec] = []
    for record in client.joint_records(body_id):
        if client.is_fixed_joint(record.joint_type):
            continue
        specs.append(_spec_from_record(record, _role_for_link(record.link_name)))
    if not specs:
        raise SimulationError("robot has no controlled joints")
    return tuple(specs)


def discover_and_validate_robot_layout(client: PhysicsClient, body_id: int) -> RobotLayout:
    """Discover joints and require the configured arm, gripper, limits, and end effector."""

    records = client.joint_records(body_id)
    by_link: dict[str, JointRecord] = {}
    for record in records:
        if record.link_name in by_link:
            raise SimulationError("robot joint link names are not unique")
        by_link[record.link_name] = record
    arm = tuple(
        _validated_arm_spec(_required_record(by_link, name), client) for name in ARM_LINK_NAMES
    )
    gripper = _validated_gripper_specs(records, client)
    _reject_unknown_controlled_joints(records, client)
    ee = _required_record(by_link, END_EFFECTOR_LINK_NAME)
    controlled = tuple(sorted((*arm, *gripper), key=lambda spec: spec.index))
    rest = tuple(_ik_rest_position(spec) for spec in controlled)
    damping = tuple(IK_DAMPING for _ in controlled)
    return RobotLayout(
        arm_joints=arm,
        gripper_joints=gripper,
        end_effector_link_index=ee.index,
        end_effector_link_name=END_EFFECTOR_LINK_NAME,
        ik_damping=damping,
        ik_rest_poses=rest,
        ik_max_iterations=IK_MAX_ITERATIONS,
        ik_residual_threshold=IK_RESIDUAL_THRESHOLD,
    )


def reset_controlled_joints(
    client: PhysicsClient, body_id: int, specs: tuple[JointSpec, ...]
) -> None:
    """Set every controlled joint pose and POSITION_CONTROL motor explicitly."""

    for spec in specs:
        client.reset_joint_state(body_id, spec.index, spec.rest_position)
        client.set_position_control(
            body_id, spec.index, spec.rest_position, force_newtons=spec.force_newtons
        )


def apply_layout_rest(client: PhysicsClient, body_id: int, layout: RobotLayout) -> None:
    """Teleport to the validated rest poses used as IK rest configuration."""

    reset_controlled_joints(client, body_id, layout.controlled_joints)


def joint_states_from_specs(
    client: PhysicsClient, body_id: int, specs: tuple[JointSpec, ...]
) -> tuple[JointState, ...]:
    states: list[JointState] = []
    for spec in specs:
        position, velocity = client.get_joint_state(body_id, spec.index)
        states.append(
            JointState(
                name=spec.name,
                position=position,
                velocity=velocity,
                position_unit=spec.position_unit,
            )
        )
    return tuple(states)


def joint_range_radians(spec: JointSpec) -> float:
    span = spec.upper_limit - spec.lower_limit
    if not math.isfinite(span) or span <= 0.0:
        return 2.0 * math.pi
    return span


def _solution_for_specs(raw: Sequence[float], specs: Sequence[JointSpec]) -> tuple[float, ...]:
    if len(raw) == len(specs):
        return tuple(raw)
    try:
        return tuple(raw[spec.index] for spec in specs)
    except IndexError as exc:
        raise SimulationError("inverse kinematics solution length is invalid") from exc


def validate_ik_solution(
    positions: Sequence[float], specs: Sequence[JointSpec]
) -> tuple[float, ...]:
    """Reject nonfinite or out-of-limit IK solutions. Does not call the physics engine."""

    if len(positions) < len(specs):
        raise SimulationError("inverse kinematics solution length is invalid")
    validated: list[float] = []
    for index, spec in enumerate(specs):
        try:
            value = require_finite(f"ik_solution[{index}]", positions[index])
        except ValueError as exc:
            raise SimulationError("inverse kinematics solution is nonfinite") from exc
        if spec.upper_limit > spec.lower_limit:
            if value < spec.lower_limit - IK_LIMIT_TOLERANCE:
                raise SimulationError("inverse kinematics solution is out of joint limits")
            if value > spec.upper_limit + IK_LIMIT_TOLERANCE:
                raise SimulationError("inverse kinematics solution is out of joint limits")
            value = min(spec.upper_limit, max(spec.lower_limit, value))
        validated.append(value)
    return tuple(validated)


def saturate_ik_solution(
    positions: Sequence[float], specs: Sequence[JointSpec]
) -> tuple[float, ...]:
    """Return a finite, in-limit command. Nonfinite values are rejected."""

    mapped = _solution_for_specs(positions, specs)
    saturated: list[float] = []
    for index, spec in enumerate(specs):
        try:
            value = require_finite(f"ik_solution[{index}]", mapped[index])
        except ValueError as exc:
            raise SimulationError("inverse kinematics solution is nonfinite") from exc
        if spec.upper_limit > spec.lower_limit:
            value = min(spec.upper_limit, max(spec.lower_limit, value))
        else:
            value = spec.rest_position
        saturated.append(value)
    return tuple(saturated)


def solve_inverse_kinematics(
    client: PhysicsClient,
    body_id: int,
    layout: RobotLayout,
    target_pose: Pose,
) -> tuple[float, ...]:
    """Solve IK with explicit limits, ranges, rest poses, damping, and iteration bound."""

    specs = layout.controlled_joints
    current: list[float] = []
    lowers: list[float] = []
    uppers: list[float] = []
    ranges: list[float] = []
    for spec in specs:
        position, _velocity = client.get_joint_state(body_id, spec.index)
        current.append(position)
        lower, upper, span = _ik_limit_triplet(spec)
        lowers.append(lower)
        uppers.append(upper)
        ranges.append(span)
    biased = Pose(
        position_meters=Vector3(
            x=target_pose.position_meters.x + IK_REQUEST_BIAS_METERS.x,
            y=target_pose.position_meters.y + IK_REQUEST_BIAS_METERS.y,
            z=target_pose.position_meters.z + IK_REQUEST_BIAS_METERS.z,
        ),
        orientation_xyzw=target_pose.orientation_xyzw,
        frame=target_pose.frame,
    )
    raw = client.calculate_inverse_kinematics(
        body_id,
        layout.end_effector_link_index,
        biased,
        lower_limits=lowers,
        upper_limits=uppers,
        joint_ranges=ranges,
        rest_poses=layout.ik_rest_poses,
        damping=layout.ik_damping,
        current_positions=current,
        max_iterations=layout.ik_max_iterations,
        residual_threshold=layout.ik_residual_threshold,
        include_orientation=False,
    )
    return saturate_ik_solution(raw, specs)


def _ik_limit_triplet(spec: JointSpec) -> tuple[float, float, float]:
    if spec.upper_limit > spec.lower_limit:
        return spec.lower_limit, spec.upper_limit, joint_range_radians(spec)
    rest = spec.rest_position
    return rest - 0.01, rest + 0.01, 0.02


def apply_wrist_yaw(positions: Mapping[int, float], layout: RobotLayout) -> dict[int, float]:
    """Rotate J6 so the parallel-jaw fingers open along world Y."""

    commanded = dict(positions)
    commanded[layout.arm_joints[-1].index] = ARM_WRIST_YAW_RADIANS
    return commanded


def command_arm_positions(
    client: PhysicsClient,
    body_id: int,
    layout: RobotLayout,
    positions: Mapping[int, float],
) -> tuple[JointState, ...]:
    commanded: list[JointState] = []
    for spec in layout.arm_joints:
        target = require_finite(f"arm_target[{spec.name}]", positions[spec.index])
        client.set_position_control(
            body_id,
            spec.index,
            target,
            force_newtons=spec.force_newtons,
        )
        commanded.append(
            JointState(
                name=spec.name,
                position=target,
                velocity=0.0,
                position_unit=spec.position_unit,
            )
        )
    return tuple(commanded)


def command_gripper(
    client: PhysicsClient,
    body_id: int,
    layout: RobotLayout,
    *,
    opening_radians: float,
) -> tuple[JointState, ...]:
    opening = require_finite("opening_radians", opening_radians)
    commanded: list[JointState] = []
    for spec in layout.gripper_joints:
        target = _gripper_target(spec, opening)
        client.set_position_control(
            body_id,
            spec.index,
            target,
            force_newtons=spec.force_newtons,
            max_velocity=GRIPPER_MAX_VELOCITY,
        )
        commanded.append(
            JointState(
                name=spec.name,
                position=target,
                velocity=0.0,
                position_unit=spec.position_unit,
            )
        )
    return tuple(commanded)


def _gripper_target(spec: JointSpec, opening_radians: float) -> float:
    if spec.link_name == "left_finger":
        return -opening_radians
    if spec.link_name == "right_finger":
        return opening_radians
    return 0.0


def _role_for_link(link_name: str) -> JointRole:
    if link_name in ARM_LINK_NAMES:
        return JointRole.ARM
    return JointRole.GRIPPER


def _required_record(by_link: Mapping[str, JointRecord], link_name: str) -> JointRecord:
    try:
        return by_link[link_name]
    except KeyError as exc:
        raise SimulationError(f"configured robot link is missing: {link_name}") from exc


def _validated_arm_spec(record: JointRecord, client: PhysicsClient) -> JointSpec:
    if client.is_fixed_joint(record.joint_type):
        raise SimulationError(f"configured arm joint is fixed: {record.link_name}")
    spec = _spec_from_record(record, JointRole.ARM)
    if spec.upper_limit <= spec.lower_limit:
        raise SimulationError(f"configured arm joint limits are invalid: {spec.name}")
    rest = ARM_IK_REST_POSITIONS[ARM_LINK_NAMES.index(record.link_name)]
    if rest < spec.lower_limit or rest > spec.upper_limit:
        raise SimulationError(f"configured arm rest pose is out of limits: {spec.name}")
    return JointSpec(
        index=spec.index,
        name=spec.name,
        link_name=spec.link_name,
        rest_position=rest,
        force_newtons=spec.force_newtons,
        position_unit=spec.position_unit,
        lower_limit=spec.lower_limit,
        upper_limit=spec.upper_limit,
        role=JointRole.ARM,
    )


def _validated_gripper_specs(
    records: Sequence[JointRecord], client: PhysicsClient
) -> tuple[JointSpec, ...]:
    by_link = {record.link_name: record for record in records}
    required = (*GRIPPER_FINGER_LINK_NAMES, *GRIPPER_TIP_LINK_NAMES)
    specs: list[JointSpec] = []
    for name in required:
        record = _required_record(by_link, name)
        if client.is_fixed_joint(record.joint_type):
            raise SimulationError(f"configured gripper joint is fixed: {name}")
        specs.append(_gripper_spec(record))
        if specs[-1].upper_limit <= specs[-1].lower_limit:
            raise SimulationError(f"configured gripper joint limits are invalid: {name}")
    rotate = by_link.get(GRIPPER_ROTATE_LINK_NAME)
    if rotate is not None and not client.is_fixed_joint(rotate.joint_type):
        rotate_spec = _gripper_spec(rotate)
        _ = joint_range_radians(rotate_spec)
        specs.append(rotate_spec)
    return tuple(sorted(specs, key=lambda spec: spec.index))


def _reject_unknown_controlled_joints(
    records: Sequence[JointRecord], client: PhysicsClient
) -> None:
    known = set(ARM_LINK_NAMES) | KNOWN_GRIPPER_LINK_NAMES
    for record in records:
        if client.is_fixed_joint(record.joint_type):
            continue
        if record.link_name not in known:
            raise SimulationError(f"unexpected controlled joint: {record.link_name}")


def _gripper_spec(record: JointRecord) -> JointSpec:
    spec = _spec_from_record(record, JointRole.GRIPPER)
    force = spec.force_newtons
    if record.link_name in GRIPPER_FINGER_LINK_NAMES:
        force = GRIPPER_FINGER_FORCE_NEWTONS
    elif record.link_name in GRIPPER_TIP_LINK_NAMES:
        force = GRIPPER_TIP_FORCE_NEWTONS
    elif record.link_name == GRIPPER_ROTATE_LINK_NAME:
        force = GRIPPER_ROTATE_FORCE_NEWTONS
    return JointSpec(
        index=spec.index,
        name=spec.name,
        link_name=spec.link_name,
        rest_position=spec.rest_position,
        force_newtons=force,
        position_unit=spec.position_unit,
        lower_limit=spec.lower_limit,
        upper_limit=spec.upper_limit,
        role=JointRole.GRIPPER,
    )


def _ik_rest_position(spec: JointSpec) -> float:
    if spec.role is JointRole.ARM:
        return spec.rest_position
    return spec.rest_position


def _spec_from_record(record: JointRecord, role: JointRole) -> JointSpec:
    force = (
        record.max_force_newtons if record.max_force_newtons > 0.0 else FALLBACK_JOINT_FORCE_NEWTONS
    )
    unit = CanonicalUnit.METERS if record.is_prismatic else CanonicalUnit.RADIANS
    return JointSpec(
        index=record.index,
        name=record.name,
        link_name=record.link_name,
        rest_position=record.rest_position,
        force_newtons=force,
        position_unit=unit,
        lower_limit=record.lower_limit,
        upper_limit=record.upper_limit,
        role=role,
    )
