"""Typed robot motion primitives with timeouts, tolerances, and recorded results."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum
from typing import Final

from robot_control_platform_simulator.domain.models import (
    Action,
    JointState,
    Pose,
    QuaternionXYZW,
    Vector3,
    require_finite,
    require_nonnegative,
    require_positive,
)
from robot_control_platform_simulator.physics.client import (
    PHYSICS_TIMESTEP_SECONDS,
    WORLD_FRAME,
    PhysicsClient,
    SimulationError,
)
from robot_control_platform_simulator.physics.robot import (
    GRIPPER_CLOSED_RADIANS,
    GRIPPER_OPEN_RADIANS,
    JointRole,
    RobotLayout,
    apply_wrist_yaw,
    command_arm_positions,
    command_gripper,
    joint_states_from_specs,
    solve_inverse_kinematics,
)

PHYSICS_CONTROL_HZ: Final[int] = 60
PHYSICS_STEPS_PER_CONTROL: Final[int] = 4
DOWNWARD_EE_ORIENTATION: Final[QuaternionXYZW] = QuaternionXYZW(x=0.0, y=-1.0, z=0.0, w=0.0)


class MotionPrimitive(StrEnum):
    MOVE_END_EFFECTOR = "move_end_effector"
    OPEN = "open"
    CLOSE = "close"
    HOLD = "hold"
    RETRACT = "retract"
    SETTLE = "settle"


class ActionStatus(StrEnum):
    SUCCEEDED = "succeeded"
    TIMEOUT = "timeout"
    IK_REJECTED = "ik_rejected"


@dataclass(frozen=True)
class MotionCommand:
    """One motion primitive with a timeout (seconds) and a tolerance."""

    primitive: MotionPrimitive
    timeout_seconds: float
    tolerance: float
    target_pose: Pose | None = None
    opening_radians: float | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.primitive, MotionPrimitive):
            raise ValueError("primitive must be a MotionPrimitive")
        object.__setattr__(
            self,
            "timeout_seconds",
            require_positive(
                "timeout_seconds", require_finite("timeout_seconds", self.timeout_seconds)
            ),
        )
        object.__setattr__(
            self,
            "tolerance",
            require_positive("tolerance", require_finite("tolerance", self.tolerance)),
        )
        if self.target_pose is not None and not isinstance(self.target_pose, Pose):
            raise ValueError("target_pose must be a Pose or None")
        if self.opening_radians is not None:
            object.__setattr__(
                self,
                "opening_radians",
                require_nonnegative(
                    "opening_radians", require_finite("opening_radians", self.opening_radians)
                ),
            )


@dataclass(frozen=True)
class ActionResult:
    """Typed outcome of one motion primitive, with commanded and observed samples."""

    primitive: MotionPrimitive
    status: ActionStatus
    commanded: Action
    observed_joints: tuple[JointState, ...]
    observed_ee_pose: Pose
    observed_simulation_time_seconds: float
    control_updates: int

    @property
    def succeeded(self) -> bool:
        return self.status is ActionStatus.SUCCEEDED


class MotionController:
    """Execute typed arm/gripper actions at 60 Hz with four physics steps per update."""

    def __init__(self, client: PhysicsClient, body_id: int, layout: RobotLayout) -> None:
        self._client = client
        self._body_id = body_id
        self._layout = layout
        self._gripper_opening = GRIPPER_CLOSED_RADIANS
        self._arm_targets = {spec.index: spec.rest_position for spec in layout.arm_joints}

    def execute(self, command: MotionCommand) -> ActionResult:
        if command.primitive is MotionPrimitive.MOVE_END_EFFECTOR:
            return self._move_end_effector(command)
        if command.primitive is MotionPrimitive.RETRACT:
            return self._retract(command)
        if command.primitive is MotionPrimitive.OPEN:
            return self._set_gripper(command, GRIPPER_OPEN_RADIANS)
        if command.primitive is MotionPrimitive.CLOSE:
            opening = (
                command.opening_radians
                if command.opening_radians is not None
                else GRIPPER_CLOSED_RADIANS
            )
            return self._set_gripper(command, opening)
        if command.primitive is MotionPrimitive.HOLD:
            return self._hold(command, settle=False)
        if command.primitive is MotionPrimitive.SETTLE:
            return self._hold(command, settle=True)
        raise SimulationError(f"unsupported motion primitive: {command.primitive}")

    def _move_end_effector(self, command: MotionCommand) -> ActionResult:
        if command.target_pose is None:
            raise ValueError("move_end_effector requires target_pose")
        return self._track_pose(command, command.target_pose)

    def _retract(self, command: MotionCommand) -> ActionResult:
        if command.target_pose is None:
            raise ValueError("retract requires target_pose")
        return self._track_pose(command, command.target_pose)

    def _track_pose(self, command: MotionCommand, target: Pose) -> ActionResult:
        start_time = self._client.simulation_time_seconds()
        commanded_joints: tuple[JointState, ...] = ()
        updates = 0
        status = ActionStatus.TIMEOUT
        while self._client.simulation_time_seconds() - start_time < command.timeout_seconds:
            observed_pose = self._client.get_link_pose(
                self._body_id, self._layout.end_effector_link_index
            )
            ik_target = _compensated_pose_target(target, observed_pose)
            try:
                solution = solve_inverse_kinematics(
                    self._client, self._body_id, self._layout, ik_target
                )
            except (SimulationError, ValueError):
                status = ActionStatus.IK_REJECTED
                break
            arm_targets = apply_wrist_yaw(
                {
                    spec.index: solution[index]
                    for index, spec in enumerate(self._layout.controlled_joints)
                    if spec.role is JointRole.ARM
                },
                self._layout,
            )
            commanded_joints = command_arm_positions(
                self._client, self._body_id, self._layout, arm_targets
            )
            self._arm_targets = arm_targets
            command_gripper(
                self._client,
                self._body_id,
                self._layout,
                opening_radians=self._gripper_opening,
            )
            self.advance_control()
            updates += 1
            observed_pose = self._client.get_link_pose(
                self._body_id, self._layout.end_effector_link_index
            )
            if _pose_within_tolerance(target, observed_pose, command.tolerance):
                status = ActionStatus.SUCCEEDED
                break
        return self._result(command, status, commanded_joints, target, start_time, updates)

    def _set_gripper(self, command: MotionCommand, opening_radians: float) -> ActionResult:
        start_time = self._client.simulation_time_seconds()
        self._gripper_opening = opening_radians
        commanded_joints = command_gripper(
            self._client, self._body_id, self._layout, opening_radians=opening_radians
        )
        self._hold_arm()
        updates = 0
        status = ActionStatus.TIMEOUT
        while self._client.simulation_time_seconds() - start_time < command.timeout_seconds:
            command_gripper(
                self._client, self._body_id, self._layout, opening_radians=opening_radians
            )
            self._hold_arm()
            self.advance_control()
            updates += 1
            observed = joint_states_from_specs(
                self._client, self._body_id, self._layout.finger_joints
            )
            if _joints_within_tolerance(commanded_joints, observed, command.tolerance):
                status = ActionStatus.SUCCEEDED
                self._gripper_opening = opening_radians
                break
        if status is ActionStatus.SUCCEEDED:
            self._gripper_opening = opening_radians
        return self._result(
            command, status, commanded_joints, command.target_pose, start_time, updates
        )

    def _hold(self, command: MotionCommand, *, settle: bool) -> ActionResult:
        start_time = self._client.simulation_time_seconds()
        commanded_joints = self._hold_arm_and_gripper()
        updates = 0
        status = ActionStatus.TIMEOUT
        while self._client.simulation_time_seconds() - start_time < command.timeout_seconds:
            self._hold_arm_and_gripper()
            self.advance_control()
            updates += 1
            if not settle:
                if self._client.simulation_time_seconds() - start_time >= command.timeout_seconds:
                    break
                continue
            observed = joint_states_from_specs(
                self._client, self._body_id, self._layout.controlled_joints
            )
            if _velocities_within_tolerance(observed, command.tolerance):
                status = ActionStatus.SUCCEEDED
                break
        if not settle:
            status = ActionStatus.SUCCEEDED
        return self._result(
            command, status, commanded_joints, command.target_pose, start_time, updates
        )

    def _hold_arm(self) -> tuple[JointState, ...]:
        return command_arm_positions(self._client, self._body_id, self._layout, self._arm_targets)

    def _hold_arm_and_gripper(self) -> tuple[JointState, ...]:
        arm = self._hold_arm()
        gripper = command_gripper(
            self._client, self._body_id, self._layout, opening_radians=self._gripper_opening
        )
        return arm + gripper

    def advance_control(self) -> None:
        self._client.step_simulation(PHYSICS_STEPS_PER_CONTROL)

    def _result(
        self,
        command: MotionCommand,
        status: ActionStatus,
        commanded_joints: tuple[JointState, ...],
        target_pose: Pose | None,
        command_time: float,
        updates: int,
    ) -> ActionResult:
        observed_joints = joint_states_from_specs(
            self._client, self._body_id, self._layout.controlled_joints
        )
        observed_pose = self._client.get_link_pose(
            self._body_id, self._layout.end_effector_link_index
        )
        return ActionResult(
            primitive=command.primitive,
            status=status,
            commanded=Action(
                name=command.primitive.value,
                simulation_time_seconds=command_time,
                target_pose=target_pose,
                joint_targets=commanded_joints,
            ),
            observed_joints=observed_joints,
            observed_ee_pose=observed_pose,
            observed_simulation_time_seconds=self._client.simulation_time_seconds(),
            control_updates=updates,
        )


def downward_pose(position: Vector3) -> Pose:
    return Pose(
        position_meters=position,
        orientation_xyzw=DOWNWARD_EE_ORIENTATION,
        frame=WORLD_FRAME,
    )


def control_period_seconds() -> float:
    return PHYSICS_TIMESTEP_SECONDS * PHYSICS_STEPS_PER_CONTROL


def _pose_error_meters(target: Pose, observed: Pose) -> float:
    dx = target.position_meters.x - observed.position_meters.x
    dy = target.position_meters.y - observed.position_meters.y
    dz = target.position_meters.z - observed.position_meters.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _compensated_pose_target(target: Pose, observed: Pose) -> Pose:
    """Request the target plus residual error so position-only IK can cancel bias."""

    return Pose(
        position_meters=Vector3(
            x=target.position_meters.x + (target.position_meters.x - observed.position_meters.x),
            y=target.position_meters.y + (target.position_meters.y - observed.position_meters.y),
            z=target.position_meters.z + (target.position_meters.z - observed.position_meters.z),
        ),
        orientation_xyzw=target.orientation_xyzw,
        frame=target.frame,
    )


def _pose_within_tolerance(target: Pose, observed: Pose, tolerance_meters: float) -> bool:
    return _pose_error_meters(target, observed) <= tolerance_meters


def _joints_within_tolerance(
    commanded: tuple[JointState, ...],
    observed: tuple[JointState, ...],
    tolerance: float,
) -> bool:
    by_name = {state.name: state for state in commanded}
    for state in observed:
        target = by_name.get(state.name)
        if target is None:
            continue
        if abs(state.position - target.position) > tolerance:
            return False
    return True


def _velocities_within_tolerance(states: tuple[JointState, ...], tolerance: float) -> bool:
    return all(abs(state.velocity) <= tolerance for state in states)
