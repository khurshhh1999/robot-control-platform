"""Policy protocol, versioned config, static allowlist, and shared state dispatch."""

from __future__ import annotations

import re
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Final, Protocol, runtime_checkable
from uuid import UUID

from robot_control_platform_common.ids import new_id

from robot_control_platform_simulator.control.actions import (
    CONSERVATIVE_SETTLE_TIMEOUT_SECONDS,
    DEFAULT_GRIPPER_TIMEOUT_SECONDS,
    DEFAULT_GRIPPER_TOLERANCE_RADIANS,
    DEFAULT_HOLD_TIMEOUT_SECONDS,
    DEFAULT_MOVE_TIMEOUT_SECONDS,
    DEFAULT_MOVE_TOLERANCE_METERS,
    DEFAULT_SETTLE_TIMEOUT_SECONDS,
    DEFAULT_SETTLE_VELOCITY_TOLERANCE,
    EE_TOOL_OFFSET_METERS,
    EE_XY_OFFSET_METERS,
    FIXED_APPROACH_CLEARANCE_METERS,
    FIXED_LIFT_CLEARANCE_METERS,
    NOMINAL_OBJECT_HALF_HEIGHT_METERS,
    SAFER_LIFT_CLEARANCE_METERS,
    VIA_CLEARANCE_METERS,
    ActionStatus,
    MotionCommand,
    actions_from_commands,
    close_command,
    end_effector_pose,
    fixed_object_center_z,
    hold_command,
    interpolate_xy,
    move_end_effector_command,
    open_command,
    retract_command,
    settle_command,
)
from robot_control_platform_simulator.control.state_machine import is_allowed_transition
from robot_control_platform_simulator.domain.enums import ControllerState, ExperimentStatus
from robot_control_platform_simulator.domain.events import ContactEvent
from robot_control_platform_simulator.domain.models import (
    DOMAIN_SCHEMA_VERSION,
    Action,
    JSONValue,
    ObjectState,
    Pose,
    Vector3,
    canonical_dumps,
    require_finite,
    require_name,
    require_nonnegative,
    require_positive,
    sha256_hex,
)
from robot_control_platform_simulator.physics.scene import TABLE_TOP_Z_METERS
from robot_control_platform_simulator.scenarios.generator import Scenario

POLICIES_SCHEMA_VERSION: Final[str] = DOMAIN_SCHEMA_VERSION
POLICY_FIXED: Final[str] = "fixed"
POLICY_POSE_AWARE: Final[str] = "pose_aware"
POLICY_COLLISION_AWARE: Final[str] = "collision_aware"
POLICY_ALLOWLIST: Final[tuple[str, ...]] = (
    POLICY_FIXED,
    POLICY_POSE_AWARE,
    POLICY_COLLISION_AWARE,
)
POLICY_VERSION_NAMES: Final[dict[str, str]] = {
    POLICY_FIXED: "v1_fixed",
    POLICY_POSE_AWARE: "v2_pose_aware",
    POLICY_COLLISION_AWARE: "v3_collision_aware",
}
POLICY_SEMANTIC_VERSION: Final[str] = "1.0.0"
POLICY_DESCRIPTIONS: Final[dict[str, str]] = {
    POLICY_FIXED: (
        "Fixed approach height, direct top-down grasp, fixed lift, and direct transfer "
        "without obstacle-aware waypoints or retry."
    ),
    POLICY_POSE_AWARE: (
        "Uses the observed object pose, adaptive grasp height, safer lift, and "
        "placement centered over the target bin."
    ),
    POLICY_COLLISION_AWARE: (
        "Adds preflight reachability, staged waypoints, contact-aware abort and retract, "
        "one bounded re-grasp, and conservative release verification."
    ),
}
POLICY_LOCKING_EXPERIMENT_STATUSES: Final[frozenset[ExperimentStatus]] = frozenset(
    {
        ExperimentStatus.QUEUED,
        ExperimentStatus.RUNNING,
        ExperimentStatus.COMPLETED,
        ExperimentStatus.COMPLETED_WITH_ERRORS,
    }
)
_SEMANTIC_VERSION_PATTERN: Final[re.Pattern[str]] = re.compile(r"^\d+\.\d+\.\d+$")


class PolicyImmutableError(ValueError):
    """Raised when a referenced policy version would be mutated."""


def _require_nonnegative_int(field: str, value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        msg = f"{field} must be a nonnegative integer"
        raise ValueError(msg)
    return value


def _require_semantic_version(value: object) -> str:
    text = require_name("semantic_version", value)
    if _SEMANTIC_VERSION_PATTERN.fullmatch(text) is None:
        msg = "semantic_version must be dotted numeric major.minor.patch"
        raise ValueError(msg)
    return text


def _require_allowlisted_implementation(value: object) -> str:
    name = require_name("implementation", value)
    if name not in POLICY_ALLOWLIST:
        msg = f"policy implementation is not allowlisted: {name}"
        raise ValueError(msg)
    return name


@dataclass(frozen=True)
class PolicyConfig:
    """Versioned motion parameters. Policies do not read an uncontrolled RNG."""

    approach_clearance_meters: float
    lift_clearance_meters: float
    via_clearance_meters: float
    grasp_tool_offset_meters: float
    nominal_object_half_height_meters: float
    table_top_z_meters: float
    ee_xy_offset_meters: Vector3
    move_timeout_seconds: float
    gripper_timeout_seconds: float
    hold_timeout_seconds: float
    settle_timeout_seconds: float
    move_tolerance_meters: float
    gripper_tolerance_radians: float
    settle_velocity_tolerance: float
    regrasp_limit: int
    approach_stage_count: int
    transfer_stage_count: int
    release_settle_count: int

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "approach_clearance_meters",
            require_positive(
                "approach_clearance_meters",
                require_finite("approach_clearance_meters", self.approach_clearance_meters),
            ),
        )
        object.__setattr__(
            self,
            "lift_clearance_meters",
            require_positive(
                "lift_clearance_meters",
                require_finite("lift_clearance_meters", self.lift_clearance_meters),
            ),
        )
        object.__setattr__(
            self,
            "via_clearance_meters",
            require_nonnegative(
                "via_clearance_meters",
                require_finite("via_clearance_meters", self.via_clearance_meters),
            ),
        )
        object.__setattr__(
            self,
            "grasp_tool_offset_meters",
            require_positive(
                "grasp_tool_offset_meters",
                require_finite("grasp_tool_offset_meters", self.grasp_tool_offset_meters),
            ),
        )
        object.__setattr__(
            self,
            "nominal_object_half_height_meters",
            require_positive(
                "nominal_object_half_height_meters",
                require_finite(
                    "nominal_object_half_height_meters", self.nominal_object_half_height_meters
                ),
            ),
        )
        object.__setattr__(
            self,
            "table_top_z_meters",
            require_finite("table_top_z_meters", self.table_top_z_meters),
        )
        if not isinstance(self.ee_xy_offset_meters, Vector3):
            msg = "ee_xy_offset_meters must be a Vector3"
            raise ValueError(msg)
        object.__setattr__(
            self,
            "move_timeout_seconds",
            require_positive(
                "move_timeout_seconds",
                require_finite("move_timeout_seconds", self.move_timeout_seconds),
            ),
        )
        object.__setattr__(
            self,
            "gripper_timeout_seconds",
            require_positive(
                "gripper_timeout_seconds",
                require_finite("gripper_timeout_seconds", self.gripper_timeout_seconds),
            ),
        )
        object.__setattr__(
            self,
            "hold_timeout_seconds",
            require_positive(
                "hold_timeout_seconds",
                require_finite("hold_timeout_seconds", self.hold_timeout_seconds),
            ),
        )
        object.__setattr__(
            self,
            "settle_timeout_seconds",
            require_positive(
                "settle_timeout_seconds",
                require_finite("settle_timeout_seconds", self.settle_timeout_seconds),
            ),
        )
        object.__setattr__(
            self,
            "move_tolerance_meters",
            require_positive(
                "move_tolerance_meters",
                require_finite("move_tolerance_meters", self.move_tolerance_meters),
            ),
        )
        object.__setattr__(
            self,
            "gripper_tolerance_radians",
            require_positive(
                "gripper_tolerance_radians",
                require_finite("gripper_tolerance_radians", self.gripper_tolerance_radians),
            ),
        )
        object.__setattr__(
            self,
            "settle_velocity_tolerance",
            require_positive(
                "settle_velocity_tolerance",
                require_finite("settle_velocity_tolerance", self.settle_velocity_tolerance),
            ),
        )
        object.__setattr__(
            self, "regrasp_limit", _require_nonnegative_int("regrasp_limit", self.regrasp_limit)
        )
        object.__setattr__(
            self,
            "approach_stage_count",
            _require_nonnegative_int("approach_stage_count", self.approach_stage_count),
        )
        object.__setattr__(
            self,
            "transfer_stage_count",
            _require_nonnegative_int("transfer_stage_count", self.transfer_stage_count),
        )
        object.__setattr__(
            self,
            "release_settle_count",
            _require_nonnegative_int("release_settle_count", self.release_settle_count),
        )
        if self.release_settle_count < 1:
            msg = "release_settle_count must be at least 1"
            raise ValueError(msg)

    def to_checksum_payload(self) -> dict[str, JSONValue]:
        return {
            "schema_version": POLICIES_SCHEMA_VERSION,
            "approach_clearance_meters": self.approach_clearance_meters,
            "approach_stage_count": self.approach_stage_count,
            "ee_xy_offset_meters": self.ee_xy_offset_meters.to_checksum_payload(),
            "grasp_tool_offset_meters": self.grasp_tool_offset_meters,
            "gripper_timeout_seconds": self.gripper_timeout_seconds,
            "gripper_tolerance_radians": self.gripper_tolerance_radians,
            "hold_timeout_seconds": self.hold_timeout_seconds,
            "lift_clearance_meters": self.lift_clearance_meters,
            "move_timeout_seconds": self.move_timeout_seconds,
            "move_tolerance_meters": self.move_tolerance_meters,
            "nominal_object_half_height_meters": self.nominal_object_half_height_meters,
            "regrasp_limit": self.regrasp_limit,
            "release_settle_count": self.release_settle_count,
            "settle_timeout_seconds": self.settle_timeout_seconds,
            "settle_velocity_tolerance": self.settle_velocity_tolerance,
            "table_top_z_meters": self.table_top_z_meters,
            "transfer_stage_count": self.transfer_stage_count,
            "via_clearance_meters": self.via_clearance_meters,
        }

    def canonical_json(self) -> str:
        return canonical_dumps(self.to_checksum_payload())

    def sha256_hex(self) -> str:
        return sha256_hex(self.to_checksum_payload())


def _shared_policy_config(
    *,
    lift_clearance_meters: float,
    via_clearance_meters: float,
    settle_timeout_seconds: float,
    regrasp_limit: int,
    approach_stage_count: int,
    transfer_stage_count: int,
    release_settle_count: int,
) -> PolicyConfig:
    return PolicyConfig(
        approach_clearance_meters=FIXED_APPROACH_CLEARANCE_METERS,
        lift_clearance_meters=lift_clearance_meters,
        via_clearance_meters=via_clearance_meters,
        grasp_tool_offset_meters=EE_TOOL_OFFSET_METERS,
        nominal_object_half_height_meters=NOMINAL_OBJECT_HALF_HEIGHT_METERS,
        table_top_z_meters=TABLE_TOP_Z_METERS,
        ee_xy_offset_meters=EE_XY_OFFSET_METERS,
        move_timeout_seconds=DEFAULT_MOVE_TIMEOUT_SECONDS,
        gripper_timeout_seconds=DEFAULT_GRIPPER_TIMEOUT_SECONDS,
        hold_timeout_seconds=DEFAULT_HOLD_TIMEOUT_SECONDS,
        settle_timeout_seconds=settle_timeout_seconds,
        move_tolerance_meters=DEFAULT_MOVE_TOLERANCE_METERS,
        gripper_tolerance_radians=DEFAULT_GRIPPER_TOLERANCE_RADIANS,
        settle_velocity_tolerance=DEFAULT_SETTLE_VELOCITY_TOLERANCE,
        regrasp_limit=regrasp_limit,
        approach_stage_count=approach_stage_count,
        transfer_stage_count=transfer_stage_count,
        release_settle_count=release_settle_count,
    )


def default_fixed_policy_config() -> PolicyConfig:
    return _shared_policy_config(
        lift_clearance_meters=FIXED_LIFT_CLEARANCE_METERS,
        via_clearance_meters=0.0,
        settle_timeout_seconds=DEFAULT_SETTLE_TIMEOUT_SECONDS,
        regrasp_limit=0,
        approach_stage_count=0,
        transfer_stage_count=0,
        release_settle_count=1,
    )


def default_pose_aware_policy_config() -> PolicyConfig:
    return _shared_policy_config(
        lift_clearance_meters=SAFER_LIFT_CLEARANCE_METERS,
        via_clearance_meters=0.0,
        settle_timeout_seconds=DEFAULT_SETTLE_TIMEOUT_SECONDS,
        regrasp_limit=0,
        approach_stage_count=0,
        transfer_stage_count=0,
        release_settle_count=1,
    )


def default_collision_aware_policy_config() -> PolicyConfig:
    return _shared_policy_config(
        lift_clearance_meters=SAFER_LIFT_CLEARANCE_METERS,
        via_clearance_meters=VIA_CLEARANCE_METERS,
        settle_timeout_seconds=CONSERVATIVE_SETTLE_TIMEOUT_SECONDS,
        regrasp_limit=1,
        approach_stage_count=1,
        transfer_stage_count=1,
        release_settle_count=2,
    )


def default_config_for(implementation: str) -> PolicyConfig:
    resolved = _require_allowlisted_implementation(implementation)
    if resolved == POLICY_FIXED:
        return default_fixed_policy_config()
    if resolved == POLICY_POSE_AWARE:
        return default_pose_aware_policy_config()
    return default_collision_aware_policy_config()


@dataclass(frozen=True)
class ReachabilityAssessment:
    """Preflight reachability supplied in the observation. Policies do not call physics."""

    approach: bool
    grasp: bool
    lift: bool
    place: bool
    staged_waypoints: tuple[bool, ...] = ()

    def __post_init__(self) -> None:
        for field in ("approach", "grasp", "lift", "place"):
            if not isinstance(getattr(self, field), bool):
                msg = f"{field} must be a boolean"
                raise ValueError(msg)
        if not isinstance(self.staged_waypoints, tuple) or any(
            not isinstance(flag, bool) for flag in self.staged_waypoints
        ):
            msg = "staged_waypoints must be a tuple of booleans"
            raise ValueError(msg)

    def required_reachable(self) -> bool:
        return (
            self.approach and self.grasp and self.lift and self.place and all(self.staged_waypoints)
        )


def default_reachability() -> ReachabilityAssessment:
    return ReachabilityAssessment(
        approach=True, grasp=True, lift=True, place=True, staged_waypoints=()
    )


@dataclass(frozen=True)
class PolicyObservation:
    """Immutable controller observation. Body ids are not accepted."""

    controller_state: ControllerState
    simulation_time_seconds: float
    object_state: ObjectState
    end_effector_pose: Pose
    gripper_opening_radians: float
    gripper_closed: bool
    contacts: tuple[ContactEvent, ...]
    collision_detected: bool
    grasp_verified: bool
    regrasp_count: int
    last_action_status: ActionStatus | None
    reachability: ReachabilityAssessment

    def __post_init__(self) -> None:
        if not isinstance(self.controller_state, ControllerState):
            msg = "controller_state must be a ControllerState"
            raise ValueError(msg)
        object.__setattr__(
            self,
            "simulation_time_seconds",
            require_nonnegative(
                "simulation_time_seconds",
                require_finite("simulation_time_seconds", self.simulation_time_seconds),
            ),
        )
        if not isinstance(self.object_state, ObjectState):
            msg = "object_state must be an ObjectState"
            raise ValueError(msg)
        if not isinstance(self.end_effector_pose, Pose):
            msg = "end_effector_pose must be a Pose"
            raise ValueError(msg)
        object.__setattr__(
            self,
            "gripper_opening_radians",
            require_nonnegative(
                "gripper_opening_radians",
                require_finite("gripper_opening_radians", self.gripper_opening_radians),
            ),
        )
        for field in ("gripper_closed", "collision_detected", "grasp_verified"):
            if not isinstance(getattr(self, field), bool):
                msg = f"{field} must be a boolean"
                raise ValueError(msg)
        if not isinstance(self.contacts, tuple) or any(
            not isinstance(contact, ContactEvent) for contact in self.contacts
        ):
            msg = "contacts must be a tuple of ContactEvent values"
            raise ValueError(msg)
        object.__setattr__(
            self, "regrasp_count", _require_nonnegative_int("regrasp_count", self.regrasp_count)
        )
        if self.last_action_status is not None and not isinstance(
            self.last_action_status, ActionStatus
        ):
            msg = "last_action_status must be an ActionStatus or None"
            raise ValueError(msg)
        if not isinstance(self.reachability, ReachabilityAssessment):
            msg = "reachability must be a ReachabilityAssessment"
            raise ValueError(msg)


@dataclass(frozen=True)
class PolicyDecision:
    """Typed actions plus the next allowed controller state."""

    commands: tuple[MotionCommand, ...]
    actions: tuple[Action, ...]
    next_state: ControllerState
    reason: str
    regrasp: bool
    abort: bool

    def __post_init__(self) -> None:
        if not isinstance(self.commands, tuple) or any(
            not isinstance(command, MotionCommand) for command in self.commands
        ):
            msg = "commands must be a tuple of MotionCommand values"
            raise ValueError(msg)
        if not isinstance(self.actions, tuple) or any(
            not isinstance(action, Action) for action in self.actions
        ):
            msg = "actions must be a tuple of Action values"
            raise ValueError(msg)
        if len(self.actions) != len(self.commands):
            msg = "actions must correspond one-to-one with commands"
            raise ValueError(msg)
        if not isinstance(self.next_state, ControllerState):
            msg = "next_state must be a ControllerState"
            raise ValueError(msg)
        object.__setattr__(self, "reason", require_name("reason", self.reason))
        if not isinstance(self.regrasp, bool):
            msg = "regrasp must be a boolean"
            raise ValueError(msg)
        if not isinstance(self.abort, bool):
            msg = "abort must be a boolean"
            raise ValueError(msg)
        if self.regrasp and self.abort:
            msg = "regrasp and abort are mutually exclusive"
            raise ValueError(msg)

    def to_checksum_payload(self) -> dict[str, JSONValue]:
        return {
            "abort": self.abort,
            "actions": [action.to_checksum_payload() for action in self.actions],
            "next_state": self.next_state.value,
            "reason": self.reason,
            "regrasp": self.regrasp,
            "schema_version": POLICIES_SCHEMA_VERSION,
        }


@dataclass(frozen=True)
class PolicyVersion:
    """Immutable allowlisted policy identity plus canonical config checksum."""

    id: UUID
    name: str
    implementation: str
    semantic_version: str
    description: str
    config: PolicyConfig
    config_checksum: str

    def __post_init__(self) -> None:
        if not isinstance(self.id, UUID):
            msg = "id must be a UUID"
            raise ValueError(msg)
        object.__setattr__(
            self, "implementation", _require_allowlisted_implementation(self.implementation)
        )
        object.__setattr__(self, "name", require_name("name", self.name))
        expected_name = POLICY_VERSION_NAMES[self.implementation]
        if self.name != expected_name:
            msg = "policy version name does not match the allowlist"
            raise ValueError(msg)
        object.__setattr__(
            self, "semantic_version", _require_semantic_version(self.semantic_version)
        )
        object.__setattr__(self, "description", require_name("description", self.description))
        if not isinstance(self.config, PolicyConfig):
            msg = "config must be a PolicyConfig"
            raise ValueError(msg)
        checksum = require_name("config_checksum", self.config_checksum)
        if checksum != checksum.lower() or len(checksum) != 64:
            msg = "config_checksum must be a lowercase SHA-256 hex digest"
            raise ValueError(msg)
        if any(char not in "0123456789abcdef" for char in checksum):
            msg = "config_checksum must be a lowercase SHA-256 hex digest"
            raise ValueError(msg)
        if checksum != self.config.sha256_hex():
            msg = "policy config checksum does not match config"
            raise ValueError(msg)
        object.__setattr__(self, "config_checksum", checksum)

    def to_checksum_payload(self) -> dict[str, JSONValue]:
        return {
            "config": self.config.to_checksum_payload(),
            "config_checksum": self.config_checksum,
            "description": self.description,
            "implementation": self.implementation,
            "name": self.name,
            "schema_version": POLICIES_SCHEMA_VERSION,
            "semantic_version": self.semantic_version,
        }


def make_policy_version(implementation: str, config: PolicyConfig | None = None) -> PolicyVersion:
    resolved = _require_allowlisted_implementation(implementation)
    resolved_config = config if config is not None else default_config_for(resolved)
    if not isinstance(resolved_config, PolicyConfig):
        msg = "config must be a PolicyConfig"
        raise ValueError(msg)
    return PolicyVersion(
        id=new_id(),
        name=POLICY_VERSION_NAMES[resolved],
        implementation=resolved,
        semantic_version=POLICY_SEMANTIC_VERSION,
        description=POLICY_DESCRIPTIONS[resolved],
        config=resolved_config,
        config_checksum=resolved_config.sha256_hex(),
    )


@runtime_checkable
class Policy(Protocol):
    """Plan typed actions from an immutable observation, scenario, and config."""

    @property
    def implementation(self) -> str: ...

    @property
    def config(self) -> PolicyConfig: ...

    def plan(
        self,
        observation: PolicyObservation,
        scenario: Scenario,
        config: PolicyConfig,
    ) -> PolicyDecision: ...


class BasePolicy:
    """Shared observe/plan/approach/grasp/verify/lift/transfer/release/verify dispatch."""

    def __init__(self, config: PolicyConfig | None = None) -> None:
        resolved = config if config is not None else self.default_config()
        self._validate_config(resolved)
        self._config = resolved

    @classmethod
    def default_config(cls) -> PolicyConfig:
        raise NotImplementedError

    @property
    def implementation(self) -> str:
        raise NotImplementedError

    @property
    def config(self) -> PolicyConfig:
        return self._config

    def uses_live_object_pose(self) -> bool:
        return False

    def abort_reason(
        self,
        observation: PolicyObservation,
        scenario: Scenario,
        config: PolicyConfig,
    ) -> str | None:
        _ = (observation, scenario, config)
        return None

    def plan(
        self,
        observation: PolicyObservation,
        scenario: Scenario,
        config: PolicyConfig,
    ) -> PolicyDecision:
        if not isinstance(observation, PolicyObservation):
            msg = "observation must be a PolicyObservation"
            raise ValueError(msg)
        if not isinstance(scenario, Scenario):
            msg = "scenario must be a Scenario"
            raise ValueError(msg)
        if not isinstance(config, PolicyConfig):
            msg = "config must be a PolicyConfig"
            raise ValueError(msg)
        self._validate_config(config)
        abort_reason = self.abort_reason(observation, scenario, config)
        if abort_reason is not None:
            return self._abort(observation, config, abort_reason)
        state = observation.controller_state
        if state is ControllerState.RESET:
            return self._advance(observation, config, ControllerState.OBSERVE, "reset_complete")
        if state is ControllerState.OBSERVE:
            return self._advance(observation, config, ControllerState.PLAN, "observe_complete")
        if state is ControllerState.PLAN:
            return self._advance(observation, config, ControllerState.APPROACH, "plan_complete")
        if state is ControllerState.APPROACH:
            return self._advance(
                observation,
                config,
                ControllerState.GRASP,
                "approach_complete",
                commands=self._approach_commands(observation, scenario, config),
            )
        if state is ControllerState.GRASP:
            return self._advance(
                observation,
                config,
                ControllerState.VERIFY_GRASP,
                "grasp_complete",
                commands=self._grasp_commands(observation, scenario, config),
            )
        if state is ControllerState.VERIFY_GRASP:
            return self._verify_grasp(observation, scenario, config)
        if state is ControllerState.LIFT:
            return self._advance(
                observation,
                config,
                ControllerState.TRANSFER,
                "lift_complete",
                commands=(self._move(self._lift_pose(observation, scenario, config), config),),
            )
        if state is ControllerState.TRANSFER:
            return self._advance(
                observation,
                config,
                ControllerState.RELEASE,
                "transfer_complete",
                commands=self._transfer_commands(observation, scenario, config),
            )
        if state is ControllerState.RELEASE:
            return self._advance(
                observation,
                config,
                ControllerState.VERIFY_PLACE,
                "release_complete",
                commands=self._release_commands(config),
            )
        if state is ControllerState.VERIFY_PLACE:
            return self._advance(
                observation,
                config,
                ControllerState.RETRACT,
                "place_verified",
                commands=(self._settle(config),),
            )
        if state is ControllerState.RETRACT:
            return self._advance(
                observation,
                config,
                ControllerState.TERMINAL,
                "retract_complete",
                commands=(
                    self._retract(self._retract_pose(observation, scenario, config), config),
                ),
            )
        return self._decision(
            observation,
            config,
            next_state=ControllerState.TERMINAL,
            commands=(),
            reason="already_terminal",
        )

    def planned_object_position(
        self, observation: PolicyObservation, scenario: Scenario, config: PolicyConfig
    ) -> Vector3:
        initial = scenario.initial_pose.position_meters
        if self.uses_live_object_pose():
            return observation.object_state.pose.position_meters
        return Vector3(
            x=initial.x,
            y=initial.y,
            z=fixed_object_center_z(
                config.table_top_z_meters, config.nominal_object_half_height_meters
            ),
        )

    def planned_place_position(
        self, observation: PolicyObservation, scenario: Scenario, config: PolicyConfig
    ) -> Vector3:
        bin_position = scenario.target_bin_pose.position_meters
        object_z = self.planned_object_position(observation, scenario, config).z
        return Vector3(x=bin_position.x, y=bin_position.y, z=object_z)

    def _validate_config(self, config: PolicyConfig) -> None:
        if config.regrasp_limit != 0:
            msg = f"{self.implementation} regrasp_limit must be 0"
            raise ValueError(msg)
        if config.approach_stage_count != 0:
            msg = f"{self.implementation} approach_stage_count must be 0"
            raise ValueError(msg)
        if config.transfer_stage_count != 0:
            msg = f"{self.implementation} transfer_stage_count must be 0"
            raise ValueError(msg)

    def _verify_grasp(
        self,
        observation: PolicyObservation,
        scenario: Scenario,
        config: PolicyConfig,
    ) -> PolicyDecision:
        _ = scenario
        if observation.grasp_verified:
            return self._advance(observation, config, ControllerState.LIFT, "grasp_verified")
        if config.regrasp_limit > 0 and observation.regrasp_count < config.regrasp_limit:
            return self._decision(
                observation,
                config,
                next_state=ControllerState.APPROACH,
                commands=(),
                reason="regrasp:grasp_unverified",
                regrasp=True,
            )
        if config.regrasp_limit > 0:
            return self._abort(observation, config, "regrasp_budget_exhausted")
        return self._abort(observation, config, "grasp_unverified")

    def _approach_commands(
        self, observation: PolicyObservation, scenario: Scenario, config: PolicyConfig
    ) -> tuple[MotionCommand, ...]:
        object_position = self.planned_object_position(observation, scenario, config)
        approach_z = (
            object_position.z + config.grasp_tool_offset_meters + config.approach_clearance_meters
        )
        commands: list[MotionCommand] = [self._open(config)]
        for stage in range(config.approach_stage_count, 0, -1):
            staged_z = approach_z + float(stage) * config.via_clearance_meters
            commands.append(
                self._move(
                    end_effector_pose(object_position, staged_z, config.ee_xy_offset_meters),
                    config,
                )
            )
        commands.append(
            self._move(
                end_effector_pose(object_position, approach_z, config.ee_xy_offset_meters),
                config,
            )
        )
        return tuple(commands)

    def _grasp_commands(
        self, observation: PolicyObservation, scenario: Scenario, config: PolicyConfig
    ) -> tuple[MotionCommand, ...]:
        object_position = self.planned_object_position(observation, scenario, config)
        grasp_z = object_position.z + config.grasp_tool_offset_meters
        return (
            self._move(
                end_effector_pose(object_position, grasp_z, config.ee_xy_offset_meters),
                config,
            ),
            self._close(config),
            self._hold(config),
        )

    def _transfer_commands(
        self, observation: PolicyObservation, scenario: Scenario, config: PolicyConfig
    ) -> tuple[MotionCommand, ...]:
        object_position = self.planned_object_position(observation, scenario, config)
        place_position = self.planned_place_position(observation, scenario, config)
        lift_z = object_position.z + config.grasp_tool_offset_meters + config.lift_clearance_meters
        place_z = place_position.z + config.grasp_tool_offset_meters
        commands: list[MotionCommand] = []
        stages = config.transfer_stage_count
        for index in range(stages):
            fraction = float(index + 1) / float(stages + 1)
            via_xy = interpolate_xy(object_position, place_position, fraction)
            via_z = lift_z + config.via_clearance_meters
            commands.append(
                self._move(end_effector_pose(via_xy, via_z, config.ee_xy_offset_meters), config)
            )
        commands.append(
            self._move(
                end_effector_pose(place_position, lift_z, config.ee_xy_offset_meters), config
            )
        )
        commands.append(
            self._move(
                end_effector_pose(place_position, place_z, config.ee_xy_offset_meters), config
            )
        )
        return tuple(commands)

    def _release_commands(self, config: PolicyConfig) -> tuple[MotionCommand, ...]:
        if config.release_settle_count <= 1:
            return (self._open(config),)
        commands: list[MotionCommand] = [self._settle(config), self._open(config)]
        extra = config.release_settle_count - 1
        commands.extend(self._settle(config) for _ in range(extra))
        return tuple(commands)

    def _lift_pose(
        self, observation: PolicyObservation, scenario: Scenario, config: PolicyConfig
    ) -> Pose:
        object_position = self.planned_object_position(observation, scenario, config)
        lift_z = object_position.z + config.grasp_tool_offset_meters + config.lift_clearance_meters
        return end_effector_pose(object_position, lift_z, config.ee_xy_offset_meters)

    def _retract_pose(
        self, observation: PolicyObservation, scenario: Scenario, config: PolicyConfig
    ) -> Pose:
        place_position = self.planned_place_position(observation, scenario, config)
        object_position = self.planned_object_position(observation, scenario, config)
        lift_z = object_position.z + config.grasp_tool_offset_meters + config.lift_clearance_meters
        return end_effector_pose(place_position, lift_z, config.ee_xy_offset_meters)

    def _abort(
        self, observation: PolicyObservation, config: PolicyConfig, reason: str
    ) -> PolicyDecision:
        target = _abort_target(observation.controller_state)
        return self._decision(
            observation,
            config,
            next_state=target,
            commands=(),
            reason=reason,
            abort=True,
        )

    def _advance(
        self,
        observation: PolicyObservation,
        config: PolicyConfig,
        next_state: ControllerState,
        reason: str,
        *,
        commands: tuple[MotionCommand, ...] = (),
    ) -> PolicyDecision:
        return self._decision(
            observation, config, next_state=next_state, commands=commands, reason=reason
        )

    def _decision(
        self,
        observation: PolicyObservation,
        config: PolicyConfig,
        *,
        next_state: ControllerState,
        commands: tuple[MotionCommand, ...],
        reason: str,
        regrasp: bool = False,
        abort: bool = False,
    ) -> PolicyDecision:
        _ = config
        current = observation.controller_state
        if next_state is not current and not is_allowed_transition(current, next_state):
            msg = f"policy transition is not allowed: {current.value} -> {next_state.value}"
            raise ValueError(msg)
        return PolicyDecision(
            commands=commands,
            actions=actions_from_commands(
                commands, simulation_time_seconds=observation.simulation_time_seconds
            ),
            next_state=next_state,
            reason=reason,
            regrasp=regrasp,
            abort=abort,
        )

    def _move(self, pose: Pose, config: PolicyConfig) -> MotionCommand:
        return move_end_effector_command(
            pose,
            timeout_seconds=config.move_timeout_seconds,
            tolerance_meters=config.move_tolerance_meters,
        )

    def _open(self, config: PolicyConfig) -> MotionCommand:
        return open_command(
            timeout_seconds=config.gripper_timeout_seconds,
            tolerance_radians=config.gripper_tolerance_radians,
        )

    def _close(self, config: PolicyConfig) -> MotionCommand:
        return close_command(
            timeout_seconds=config.gripper_timeout_seconds,
            tolerance_radians=config.gripper_tolerance_radians,
        )

    def _hold(self, config: PolicyConfig) -> MotionCommand:
        return hold_command(
            timeout_seconds=config.hold_timeout_seconds,
            tolerance=config.settle_velocity_tolerance,
        )

    def _settle(self, config: PolicyConfig) -> MotionCommand:
        return settle_command(
            timeout_seconds=config.settle_timeout_seconds,
            tolerance=config.settle_velocity_tolerance,
        )

    def _retract(self, pose: Pose, config: PolicyConfig) -> MotionCommand:
        return retract_command(
            pose,
            timeout_seconds=config.move_timeout_seconds,
            tolerance_meters=config.move_tolerance_meters,
        )


def _abort_target(state: ControllerState) -> ControllerState:
    if state is ControllerState.TERMINAL:
        return ControllerState.TERMINAL
    if is_allowed_transition(state, ControllerState.RETRACT):
        return ControllerState.RETRACT
    if is_allowed_transition(state, ControllerState.TERMINAL):
        return ControllerState.TERMINAL
    return ControllerState.TERMINAL


def create_policy(implementation: str, config: PolicyConfig | None = None) -> Policy:
    """Return an allowlisted policy. Unknown implementations are rejected."""

    resolved = _require_allowlisted_implementation(implementation)
    from robot_control_platform_simulator.policies.collision_aware import CollisionAwarePolicy
    from robot_control_platform_simulator.policies.fixed import FixedPolicy
    from robot_control_platform_simulator.policies.pose_aware import PoseAwarePolicy

    factories: dict[str, type[BasePolicy]] = {
        POLICY_FIXED: FixedPolicy,
        POLICY_POSE_AWARE: PoseAwarePolicy,
        POLICY_COLLISION_AWARE: CollisionAwarePolicy,
    }
    return factories[resolved](config)


class PolicyRegistry:
    """Static allowlist of policy versions with experiment-reference immutability."""

    def __init__(self) -> None:
        self._versions: dict[str, PolicyVersion] = {}
        self._experiments: dict[UUID, tuple[frozenset[str], ExperimentStatus]] = {}

    def register(self, version: PolicyVersion) -> None:
        if not isinstance(version, PolicyVersion):
            msg = "version must be a PolicyVersion"
            raise ValueError(msg)
        if self.is_locked(version.implementation):
            raise PolicyImmutableError(
                "policy version is immutable after an active or completed experiment reference"
            )
        self._versions[version.implementation] = version

    def register_allowlist(self) -> None:
        for implementation in POLICY_ALLOWLIST:
            self.register(make_policy_version(implementation))

    def get(self, implementation: str) -> PolicyVersion:
        resolved = _require_allowlisted_implementation(implementation)
        try:
            return self._versions[resolved]
        except KeyError as exc:
            msg = f"policy implementation is not registered: {resolved}"
            raise ValueError(msg) from exc

    def get_by_name(self, name: str) -> PolicyVersion:
        trimmed = require_name("name", name)
        for version in self._versions.values():
            if version.name == trimmed:
                return version
        msg = f"policy version is not registered: {trimmed}"
        raise ValueError(msg)

    def versions(self) -> tuple[PolicyVersion, ...]:
        return tuple(self._versions[name] for name in POLICY_ALLOWLIST if name in self._versions)

    def create(self, implementation: str) -> Policy:
        version = self.get(implementation)
        return create_policy(version.implementation, version.config)

    def replace_config(self, implementation: str, config: PolicyConfig) -> PolicyVersion:
        if not isinstance(config, PolicyConfig):
            msg = "config must be a PolicyConfig"
            raise ValueError(msg)
        current = self.get(implementation)
        if self.is_locked(implementation):
            raise PolicyImmutableError(
                "policy version is immutable after an active or completed experiment reference"
            )
        updated = PolicyVersion(
            id=current.id,
            name=current.name,
            implementation=current.implementation,
            semantic_version=current.semantic_version,
            description=current.description,
            config=config,
            config_checksum=config.sha256_hex(),
        )
        self._versions[current.implementation] = updated
        return updated

    def reference_experiment(
        self,
        experiment_id: UUID,
        implementations: Sequence[str],
        status: ExperimentStatus,
    ) -> None:
        if not isinstance(experiment_id, UUID):
            msg = "experiment_id must be a UUID"
            raise ValueError(msg)
        if not isinstance(status, ExperimentStatus):
            msg = "status must be an ExperimentStatus"
            raise ValueError(msg)
        if isinstance(implementations, (str, bytes, bytearray)) or not isinstance(
            implementations, Sequence
        ):
            msg = "implementations must be a sequence of allowlisted names"
            raise ValueError(msg)
        resolved = tuple(_require_allowlisted_implementation(name) for name in implementations)
        if not resolved:
            msg = "implementations must not be empty"
            raise ValueError(msg)
        for name in resolved:
            self.get(name)
        self._experiments[experiment_id] = (frozenset(resolved), status)

    def set_experiment_status(self, experiment_id: UUID, status: ExperimentStatus) -> None:
        if not isinstance(experiment_id, UUID):
            msg = "experiment_id must be a UUID"
            raise ValueError(msg)
        if not isinstance(status, ExperimentStatus):
            msg = "status must be an ExperimentStatus"
            raise ValueError(msg)
        try:
            implementations, _previous = self._experiments[experiment_id]
        except KeyError as exc:
            msg = "experiment is not referenced"
            raise ValueError(msg) from exc
        self._experiments[experiment_id] = (implementations, status)

    def is_locked(self, implementation: str) -> bool:
        resolved = _require_allowlisted_implementation(implementation)
        for referenced, status in self._experiments.values():
            if resolved in referenced and status in POLICY_LOCKING_EXPERIMENT_STATUSES:
                return True
        return False


def default_policy_registry() -> PolicyRegistry:
    registry = PolicyRegistry()
    registry.register_allowlist()
    return registry


def policy_config_checksum(config: PolicyConfig) -> str:
    if not isinstance(config, PolicyConfig):
        msg = "config must be a PolicyConfig"
        raise ValueError(msg)
    return config.sha256_hex()
