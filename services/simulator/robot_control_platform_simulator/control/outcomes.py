"""Pure terminal-outcome classifier with explicit frozen precedence.

A completed trial receives exactly one of ``system_error``, ``collision``,
``missed_grasp``, ``dropped_object``, ``wrong_bin``, or ``success``. Timeout is
an event, not an outcome: the classifier maps it to the highest-precedence
evaluable task outcome, or to ``system_error`` when no task outcome can be
evaluated. Infrastructure failures always map to a sanitized ``system_error``.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Final

from robot_control_platform_simulator.control.state_machine import TrialEventLog
from robot_control_platform_simulator.domain.enums import (
    ControllerState,
    EventType,
    TerminalOutcome,
)
from robot_control_platform_simulator.domain.events import ContactEvent, TrialEvent
from robot_control_platform_simulator.domain.models import (
    DOMAIN_SCHEMA_VERSION,
    JSONValue,
    Pose,
    Vector3,
    canonical_dumps,
    require_finite,
    require_name,
    require_nonnegative,
    require_positive,
    sha256_hex,
)
from robot_control_platform_simulator.physics.client import WORLD_FRAME, SimulationError
from robot_control_platform_simulator.physics.robot import (
    GRIPPER_FINGER_LINK_NAMES,
    GRIPPER_TIP_LINK_NAMES,
)
from robot_control_platform_simulator.physics.scene import (
    BIN_HEIGHT_METERS,
    BIN_INNER_XY_METERS,
    ROBOT_BODY_NAME,
    SCENE_BODY_NAMES,
    TABLE_TOP_Z_METERS,
    default_scene_config,
)

OUTCOMES_SCHEMA_VERSION: Final[str] = DOMAIN_SCHEMA_VERSION
OUTCOME_PRECEDENCE: Final[tuple[TerminalOutcome, ...]] = (
    TerminalOutcome.SYSTEM_ERROR,
    TerminalOutcome.COLLISION,
    TerminalOutcome.MISSED_GRASP,
    TerminalOutcome.DROPPED_OBJECT,
    TerminalOutcome.WRONG_BIN,
    TerminalOutcome.SUCCESS,
)
WORKCELL_BIN_NAMES: Final[tuple[str, ...]] = tuple(
    name for name in SCENE_BODY_NAMES if name.startswith("bin_")
)
GRIPPER_GRASP_LINK_NAMES: Final[frozenset[str]] = frozenset(
    (*GRIPPER_FINGER_LINK_NAMES, *GRIPPER_TIP_LINK_NAMES)
)
SYSTEM_ERROR_SIMULATION: Final[str] = "simulation_error"
SYSTEM_ERROR_TIMEOUT_UNEVALUABLE: Final[str] = "timeout_unevaluable"
SYSTEM_ERROR_EVALUATION_UNAVAILABLE: Final[str] = "evaluation_unavailable"
ALLOWED_SYSTEM_ERROR_CODES: Final[frozenset[str]] = frozenset(
    {
        SYSTEM_ERROR_SIMULATION,
        SYSTEM_ERROR_TIMEOUT_UNEVALUABLE,
        SYSTEM_ERROR_EVALUATION_UNAVAILABLE,
    }
)
_SYSTEM_ERROR_MESSAGES: Final[dict[str, str]] = {
    SYSTEM_ERROR_SIMULATION: "simulator failure prevented evaluation",
    SYSTEM_ERROR_TIMEOUT_UNEVALUABLE: "timeout occurred before a task outcome was evaluable",
    SYSTEM_ERROR_EVALUATION_UNAVAILABLE: "trial completed without an evaluable task outcome",
}
DEFAULT_GRASP_ELEVATION_METERS: Final[float] = 0.02
DEFAULT_GRASP_CONTACT_DURATION_SECONDS: Final[float] = 0.05
DEFAULT_GRASP_CONTACT_FORCE_NEWTONS: Final[float] = 1.0
DEFAULT_BIN_CONTAINMENT_MARGIN_METERS: Final[float] = 0.02
DEFAULT_SETTLE_WINDOW_SECONDS: Final[float] = 0.10
DEFAULT_SETTLE_LINEAR_SPEED_METERS_PER_SECOND: Final[float] = 0.05
DEFAULT_SETTLE_ANGULAR_SPEED_RADIANS_PER_SECOND: Final[float] = 0.5
TIME_COMPARISON_EPSILON_SECONDS: Final[float] = 1e-9
_SETTLED_OUTSIDE: Final[str] = "none"
_UNSAFE_DETAIL_MARKERS: Final[tuple[str, ...]] = (
    "/",
    "\\",
    "file:",
    ".urdf",
    ".sdf",
    ".py",
    "traceback",
    "pybullet",
)


@dataclass(frozen=True)
class OutcomeClassificationConfig:
    """Versioned thresholds for grasp, drop, containment, and settling."""

    grasp_elevation_meters: float
    grasp_contact_duration_seconds: float
    grasp_contact_force_newtons: float
    bin_containment_margin_meters: float
    settle_window_seconds: float
    settle_linear_speed_meters_per_second: float
    settle_angular_speed_radians_per_second: float
    bin_inner_xy_meters: tuple[float, float]
    bin_height_meters: float
    table_top_z_meters: float

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "grasp_elevation_meters",
            require_positive(
                "grasp_elevation_meters",
                require_finite("grasp_elevation_meters", self.grasp_elevation_meters),
            ),
        )
        object.__setattr__(
            self,
            "grasp_contact_duration_seconds",
            require_nonnegative(
                "grasp_contact_duration_seconds",
                require_finite(
                    "grasp_contact_duration_seconds", self.grasp_contact_duration_seconds
                ),
            ),
        )
        object.__setattr__(
            self,
            "grasp_contact_force_newtons",
            require_nonnegative(
                "grasp_contact_force_newtons",
                require_finite("grasp_contact_force_newtons", self.grasp_contact_force_newtons),
            ),
        )
        object.__setattr__(
            self,
            "bin_containment_margin_meters",
            require_nonnegative(
                "bin_containment_margin_meters",
                require_finite("bin_containment_margin_meters", self.bin_containment_margin_meters),
            ),
        )
        object.__setattr__(
            self,
            "settle_window_seconds",
            require_nonnegative(
                "settle_window_seconds",
                require_finite("settle_window_seconds", self.settle_window_seconds),
            ),
        )
        object.__setattr__(
            self,
            "settle_linear_speed_meters_per_second",
            require_nonnegative(
                "settle_linear_speed_meters_per_second",
                require_finite(
                    "settle_linear_speed_meters_per_second",
                    self.settle_linear_speed_meters_per_second,
                ),
            ),
        )
        object.__setattr__(
            self,
            "settle_angular_speed_radians_per_second",
            require_nonnegative(
                "settle_angular_speed_radians_per_second",
                require_finite(
                    "settle_angular_speed_radians_per_second",
                    self.settle_angular_speed_radians_per_second,
                ),
            ),
        )
        object.__setattr__(self, "bin_inner_xy_meters", _require_inner_xy(self.bin_inner_xy_meters))
        object.__setattr__(
            self,
            "bin_height_meters",
            require_positive(
                "bin_height_meters", require_finite("bin_height_meters", self.bin_height_meters)
            ),
        )
        object.__setattr__(
            self,
            "table_top_z_meters",
            require_finite("table_top_z_meters", self.table_top_z_meters),
        )
        half_x = self.bin_inner_xy_meters[0] / 2.0
        half_y = self.bin_inner_xy_meters[1] / 2.0
        if (
            self.bin_containment_margin_meters >= half_x
            or self.bin_containment_margin_meters >= half_y
        ):
            msg = "bin_containment_margin_meters must be smaller than bin inner half-extents"
            raise ValueError(msg)

    def to_checksum_payload(self) -> dict[str, JSONValue]:
        return {
            "schema_version": OUTCOMES_SCHEMA_VERSION,
            "bin_containment_margin_meters": self.bin_containment_margin_meters,
            "bin_height_meters": self.bin_height_meters,
            "bin_inner_xy_meters": [
                self.bin_inner_xy_meters[0],
                self.bin_inner_xy_meters[1],
            ],
            "grasp_contact_duration_seconds": self.grasp_contact_duration_seconds,
            "grasp_contact_force_newtons": self.grasp_contact_force_newtons,
            "grasp_elevation_meters": self.grasp_elevation_meters,
            "settle_angular_speed_radians_per_second": (
                self.settle_angular_speed_radians_per_second
            ),
            "settle_linear_speed_meters_per_second": self.settle_linear_speed_meters_per_second,
            "settle_window_seconds": self.settle_window_seconds,
            "table_top_z_meters": self.table_top_z_meters,
        }

    def canonical_json(self) -> str:
        return canonical_dumps(self.to_checksum_payload())

    def sha256_hex(self) -> str:
        return sha256_hex(self.to_checksum_payload())


@dataclass(frozen=True)
class GraspSample:
    """Object pose and gripper/object contact at one simulation time."""

    simulation_time_seconds: float
    object_position_meters: Vector3
    gripper_object_contact: bool
    contact_force_newtons: float

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "simulation_time_seconds",
            require_nonnegative(
                "simulation_time_seconds",
                require_finite("simulation_time_seconds", self.simulation_time_seconds),
            ),
        )
        if not isinstance(self.object_position_meters, Vector3):
            msg = "object_position_meters must be a Vector3"
            raise ValueError(msg)
        if not isinstance(self.gripper_object_contact, bool):
            msg = "gripper_object_contact must be a boolean"
            raise ValueError(msg)
        object.__setattr__(
            self,
            "contact_force_newtons",
            require_nonnegative(
                "contact_force_newtons",
                require_finite("contact_force_newtons", self.contact_force_newtons),
            ),
        )


@dataclass(frozen=True)
class PlacementSample:
    """Object pose and speed during the terminal settle window."""

    simulation_time_seconds: float
    object_position_meters: Vector3
    linear_speed_meters_per_second: float
    angular_speed_radians_per_second: float

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "simulation_time_seconds",
            require_nonnegative(
                "simulation_time_seconds",
                require_finite("simulation_time_seconds", self.simulation_time_seconds),
            ),
        )
        if not isinstance(self.object_position_meters, Vector3):
            msg = "object_position_meters must be a Vector3"
            raise ValueError(msg)
        object.__setattr__(
            self,
            "linear_speed_meters_per_second",
            require_nonnegative(
                "linear_speed_meters_per_second",
                require_finite(
                    "linear_speed_meters_per_second", self.linear_speed_meters_per_second
                ),
            ),
        )
        object.__setattr__(
            self,
            "angular_speed_radians_per_second",
            require_nonnegative(
                "angular_speed_radians_per_second",
                require_finite(
                    "angular_speed_radians_per_second", self.angular_speed_radians_per_second
                ),
            ),
        )


@dataclass(frozen=True)
class OutcomeEvidence:
    """Immutable classifier inputs. Body ids are not accepted."""

    events: tuple[TrialEvent, ...]
    collision_detected: bool
    infrastructure_failure: str | None
    gripper_closed: bool
    released: bool
    object_id: str
    target_bin: str
    bin_poses: tuple[tuple[str, Pose], ...]
    initial_object_position_meters: Vector3
    grasp_samples: tuple[GraspSample, ...]
    placement_samples: tuple[PlacementSample, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.events, tuple) or any(
            not isinstance(event, TrialEvent) for event in self.events
        ):
            msg = "events must be a tuple of TrialEvent values"
            raise ValueError(msg)
        _require_monotonic_events(self.events)
        if not isinstance(self.collision_detected, bool):
            msg = "collision_detected must be a boolean"
            raise ValueError(msg)
        if self.infrastructure_failure is not None:
            require_name("infrastructure_failure", self.infrastructure_failure)
            object.__setattr__(
                self,
                "infrastructure_failure",
                sanitize_infrastructure_failure(self.infrastructure_failure),
            )
        if not isinstance(self.gripper_closed, bool):
            msg = "gripper_closed must be a boolean"
            raise ValueError(msg)
        if not isinstance(self.released, bool):
            msg = "released must be a boolean"
            raise ValueError(msg)
        object.__setattr__(self, "object_id", require_name("object_id", self.object_id))
        object.__setattr__(self, "target_bin", require_name("target_bin", self.target_bin))
        if self.target_bin not in WORKCELL_BIN_NAMES:
            msg = "target_bin must be a workcell bin name"
            raise ValueError(msg)
        object.__setattr__(self, "bin_poses", _require_bin_poses(self.bin_poses))
        if not isinstance(self.initial_object_position_meters, Vector3):
            msg = "initial_object_position_meters must be a Vector3"
            raise ValueError(msg)
        object.__setattr__(self, "grasp_samples", _require_grasp_samples(self.grasp_samples))
        object.__setattr__(
            self, "placement_samples", _require_placement_samples(self.placement_samples)
        )


@dataclass(frozen=True)
class OutcomeClassification:
    """Singular terminal outcome plus the facts used to select it."""

    outcome: TerminalOutcome
    reason: str
    target_bin: str
    settled_bin: str | None
    in_target_bin: bool
    grasp_verified: bool
    grasp_lost_outside_target: bool
    gripper_closed: bool
    released: bool
    collision_detected: bool
    timeout_observed: bool
    settled: bool
    system_error_code: str | None
    event_detail: str

    def __post_init__(self) -> None:
        if not isinstance(self.outcome, TerminalOutcome):
            msg = "outcome must be a TerminalOutcome"
            raise ValueError(msg)
        object.__setattr__(self, "reason", require_name("reason", self.reason))
        object.__setattr__(self, "target_bin", require_name("target_bin", self.target_bin))
        if self.settled_bin is not None:
            object.__setattr__(self, "settled_bin", require_name("settled_bin", self.settled_bin))
        for field in (
            "in_target_bin",
            "grasp_verified",
            "grasp_lost_outside_target",
            "gripper_closed",
            "released",
            "collision_detected",
            "timeout_observed",
            "settled",
        ):
            if not isinstance(getattr(self, field), bool):
                msg = f"{field} must be a boolean"
                raise ValueError(msg)
        if self.system_error_code is not None:
            if self.system_error_code not in ALLOWED_SYSTEM_ERROR_CODES:
                msg = "system_error_code is not allowlisted"
                raise ValueError(msg)
        object.__setattr__(self, "event_detail", require_name("event_detail", self.event_detail))

    def to_checksum_payload(self) -> dict[str, JSONValue]:
        return {
            "schema_version": OUTCOMES_SCHEMA_VERSION,
            "collision_detected": self.collision_detected,
            "event_detail": self.event_detail,
            "grasp_lost_outside_target": self.grasp_lost_outside_target,
            "grasp_verified": self.grasp_verified,
            "gripper_closed": self.gripper_closed,
            "in_target_bin": self.in_target_bin,
            "outcome": self.outcome.value,
            "reason": self.reason,
            "released": self.released,
            "settled": self.settled,
            "settled_bin": self.settled_bin,
            "system_error_code": self.system_error_code,
            "target_bin": self.target_bin,
            "timeout_observed": self.timeout_observed,
        }


def default_outcome_classification_config() -> OutcomeClassificationConfig:
    return OutcomeClassificationConfig(
        grasp_elevation_meters=DEFAULT_GRASP_ELEVATION_METERS,
        grasp_contact_duration_seconds=DEFAULT_GRASP_CONTACT_DURATION_SECONDS,
        grasp_contact_force_newtons=DEFAULT_GRASP_CONTACT_FORCE_NEWTONS,
        bin_containment_margin_meters=DEFAULT_BIN_CONTAINMENT_MARGIN_METERS,
        settle_window_seconds=DEFAULT_SETTLE_WINDOW_SECONDS,
        settle_linear_speed_meters_per_second=DEFAULT_SETTLE_LINEAR_SPEED_METERS_PER_SECOND,
        settle_angular_speed_radians_per_second=DEFAULT_SETTLE_ANGULAR_SPEED_RADIANS_PER_SECOND,
        bin_inner_xy_meters=BIN_INNER_XY_METERS,
        bin_height_meters=BIN_HEIGHT_METERS,
        table_top_z_meters=TABLE_TOP_Z_METERS,
    )


def default_bin_poses() -> tuple[tuple[str, Pose], ...]:
    return default_scene_config().bin_poses


def sanitize_infrastructure_failure(source: object) -> str:
    """Map any infrastructure failure to an allowlisted system-error code.

    Raw exception text, local paths, and engine details are discarded.
    """

    if isinstance(source, str) and source in ALLOWED_SYSTEM_ERROR_CODES:
        return source
    if isinstance(source, SimulationError):
        return SYSTEM_ERROR_SIMULATION
    if isinstance(source, str) and not _looks_unsafe(source):
        if source in ALLOWED_SYSTEM_ERROR_CODES:
            return source
        return SYSTEM_ERROR_SIMULATION
    return SYSTEM_ERROR_SIMULATION


def is_gripper_object_contact(contact: ContactEvent, object_body_name: str) -> bool:
    """Return whether a normalized contact is between gripper pads and the object."""

    if not isinstance(contact, ContactEvent):
        msg = "contact must be a ContactEvent"
        raise ValueError(msg)
    object_name = require_name("object_body_name", object_body_name)
    if object_name == ROBOT_BODY_NAME:
        msg = "object_body_name must not be the robot body"
        raise ValueError(msg)
    if contact.body_a == object_name and contact.body_b == ROBOT_BODY_NAME:
        return contact.link_b in GRIPPER_GRASP_LINK_NAMES
    if contact.body_b == object_name and contact.body_a == ROBOT_BODY_NAME:
        return contact.link_a in GRIPPER_GRASP_LINK_NAMES
    return False


def object_center_in_bin(
    position_meters: Vector3,
    bin_pose: Pose,
    config: OutcomeClassificationConfig,
) -> bool:
    """Return whether the object center is inside the bin opening with margin."""

    if not isinstance(position_meters, Vector3):
        msg = "position_meters must be a Vector3"
        raise ValueError(msg)
    if not isinstance(bin_pose, Pose):
        msg = "bin_pose must be a Pose"
        raise ValueError(msg)
    if not isinstance(config, OutcomeClassificationConfig):
        msg = "config must be an OutcomeClassificationConfig"
        raise ValueError(msg)
    if bin_pose.frame != WORLD_FRAME:
        msg = "bin_pose must use the world frame"
        raise ValueError(msg)
    half_x = config.bin_inner_xy_meters[0] / 2.0 - config.bin_containment_margin_meters
    half_y = config.bin_inner_xy_meters[1] / 2.0 - config.bin_containment_margin_meters
    dx = abs(position_meters.x - bin_pose.position_meters.x)
    dy = abs(position_meters.y - bin_pose.position_meters.y)
    z_min = config.table_top_z_meters - config.bin_containment_margin_meters
    z_max = (
        config.table_top_z_meters + config.bin_height_meters + config.bin_containment_margin_meters
    )
    return dx <= half_x and dy <= half_y and z_min <= position_meters.z <= z_max


def classify_outcome(
    evidence: OutcomeEvidence,
    config: OutcomeClassificationConfig | None = None,
) -> OutcomeClassification:
    """Return exactly one terminal outcome using frozen precedence."""

    if not isinstance(evidence, OutcomeEvidence):
        msg = "evidence must be an OutcomeEvidence"
        raise ValueError(msg)
    resolved = config if config is not None else default_outcome_classification_config()
    if not isinstance(resolved, OutcomeClassificationConfig):
        msg = "config must be an OutcomeClassificationConfig"
        raise ValueError(msg)

    timeout_observed = _timeout_observed(evidence)
    gripper_closed = evidence.gripper_closed or _gripper_closed_from_events(evidence.events)
    released = evidence.released or _released_from_events(evidence.events)
    grasp_verified = _stable_grasp_verified(
        evidence.grasp_samples, evidence.initial_object_position_meters, resolved
    )
    settled, settled_bin, in_target_bin = _placement_settlement(
        evidence.placement_samples, evidence.target_bin, evidence.bin_poses, resolved
    )
    grasp_lost_outside_target = _grasp_lost_outside_target(
        evidence.grasp_samples,
        evidence.initial_object_position_meters,
        evidence.target_bin,
        evidence.bin_poses,
        resolved,
        released=released,
    )

    if evidence.infrastructure_failure is not None:
        code = sanitize_infrastructure_failure(evidence.infrastructure_failure)
        return _classification(
            TerminalOutcome.SYSTEM_ERROR,
            _SYSTEM_ERROR_MESSAGES[code],
            evidence,
            settled_bin=settled_bin,
            in_target_bin=in_target_bin,
            grasp_verified=grasp_verified,
            grasp_lost_outside_target=grasp_lost_outside_target,
            gripper_closed=gripper_closed,
            released=released,
            timeout_observed=timeout_observed,
            settled=settled,
            system_error_code=code,
        )

    task_outcome = _select_task_outcome(
        collision_detected=evidence.collision_detected,
        missed_grasp=gripper_closed and not grasp_verified,
        dropped_object=grasp_lost_outside_target,
        wrong_bin=settled and not in_target_bin,
        success=settled and in_target_bin,
    )
    if task_outcome is TerminalOutcome.COLLISION:
        return _classification(
            task_outcome,
            "prohibited collision exceeded configured thresholds",
            evidence,
            settled_bin=settled_bin,
            in_target_bin=in_target_bin,
            grasp_verified=grasp_verified,
            grasp_lost_outside_target=grasp_lost_outside_target,
            gripper_closed=gripper_closed,
            released=released,
            timeout_observed=timeout_observed,
            settled=settled,
        )
    if task_outcome is TerminalOutcome.MISSED_GRASP:
        return _classification(
            task_outcome,
            "gripper closed without a stable grasp",
            evidence,
            settled_bin=settled_bin,
            in_target_bin=in_target_bin,
            grasp_verified=grasp_verified,
            grasp_lost_outside_target=grasp_lost_outside_target,
            gripper_closed=gripper_closed,
            released=released,
            timeout_observed=timeout_observed,
            settled=settled,
        )
    if task_outcome is TerminalOutcome.DROPPED_OBJECT:
        return _classification(
            task_outcome,
            "verified grasp was lost outside the destination bin",
            evidence,
            settled_bin=settled_bin,
            in_target_bin=in_target_bin,
            grasp_verified=grasp_verified,
            grasp_lost_outside_target=grasp_lost_outside_target,
            gripper_closed=gripper_closed,
            released=released,
            timeout_observed=timeout_observed,
            settled=settled,
        )
    if task_outcome is TerminalOutcome.WRONG_BIN:
        return _classification(
            task_outcome,
            "object settled outside the target bin",
            evidence,
            settled_bin=settled_bin,
            in_target_bin=in_target_bin,
            grasp_verified=grasp_verified,
            grasp_lost_outside_target=grasp_lost_outside_target,
            gripper_closed=gripper_closed,
            released=released,
            timeout_observed=timeout_observed,
            settled=settled,
        )
    if task_outcome is TerminalOutcome.SUCCESS:
        return _classification(
            task_outcome,
            "object settled in the target bin after the stability window",
            evidence,
            settled_bin=settled_bin,
            in_target_bin=in_target_bin,
            grasp_verified=grasp_verified,
            grasp_lost_outside_target=grasp_lost_outside_target,
            gripper_closed=gripper_closed,
            released=released,
            timeout_observed=timeout_observed,
            settled=settled,
        )

    code = (
        SYSTEM_ERROR_TIMEOUT_UNEVALUABLE
        if timeout_observed
        else SYSTEM_ERROR_EVALUATION_UNAVAILABLE
    )
    return _classification(
        TerminalOutcome.SYSTEM_ERROR,
        _SYSTEM_ERROR_MESSAGES[code],
        evidence,
        settled_bin=settled_bin,
        in_target_bin=in_target_bin,
        grasp_verified=grasp_verified,
        grasp_lost_outside_target=grasp_lost_outside_target,
        gripper_closed=gripper_closed,
        released=released,
        timeout_observed=timeout_observed,
        settled=settled,
        system_error_code=code,
    )


def emit_outcome_event(
    log: TrialEventLog,
    classification: OutcomeClassification,
    *,
    simulation_time_seconds: float,
) -> TrialEvent:
    """Record the singular classified outcome as a terminal observation event."""

    if not isinstance(log, TrialEventLog):
        msg = "log must be a TrialEventLog"
        raise ValueError(msg)
    if not isinstance(classification, OutcomeClassification):
        msg = "classification must be an OutcomeClassification"
        raise ValueError(msg)
    return log.record(
        EventType.OBSERVATION,
        ControllerState.TERMINAL,
        simulation_time_seconds=simulation_time_seconds,
        detail=classification.event_detail,
    )


def _select_task_outcome(
    *,
    collision_detected: bool,
    missed_grasp: bool,
    dropped_object: bool,
    wrong_bin: bool,
    success: bool,
) -> TerminalOutcome | None:
    flags = {
        TerminalOutcome.COLLISION: collision_detected,
        TerminalOutcome.MISSED_GRASP: missed_grasp,
        TerminalOutcome.DROPPED_OBJECT: dropped_object,
        TerminalOutcome.WRONG_BIN: wrong_bin,
        TerminalOutcome.SUCCESS: success,
    }
    for outcome in OUTCOME_PRECEDENCE:
        if flags.get(outcome, False):
            return outcome
    return None


def _classification(
    outcome: TerminalOutcome,
    reason: str,
    evidence: OutcomeEvidence,
    *,
    settled_bin: str | None,
    in_target_bin: bool,
    grasp_verified: bool,
    grasp_lost_outside_target: bool,
    gripper_closed: bool,
    released: bool,
    timeout_observed: bool,
    settled: bool,
    system_error_code: str | None = None,
) -> OutcomeClassification:
    return OutcomeClassification(
        outcome=outcome,
        reason=reason,
        target_bin=evidence.target_bin,
        settled_bin=settled_bin,
        in_target_bin=in_target_bin,
        grasp_verified=grasp_verified,
        grasp_lost_outside_target=grasp_lost_outside_target,
        gripper_closed=gripper_closed,
        released=released,
        collision_detected=evidence.collision_detected,
        timeout_observed=timeout_observed,
        settled=settled,
        system_error_code=system_error_code,
        event_detail=_event_detail(
            outcome,
            target_bin=evidence.target_bin,
            settled_bin=settled_bin,
            system_error_code=system_error_code,
        ),
    )


def _event_detail(
    outcome: TerminalOutcome,
    *,
    target_bin: str,
    settled_bin: str | None,
    system_error_code: str | None,
) -> str:
    parts = [f"outcome={outcome.value}"]
    if outcome is TerminalOutcome.SYSTEM_ERROR and system_error_code is not None:
        parts.append(f"code={system_error_code}")
    if outcome in {TerminalOutcome.WRONG_BIN, TerminalOutcome.SUCCESS}:
        settled = settled_bin if settled_bin is not None else _SETTLED_OUTSIDE
        parts.append(f"target={target_bin}")
        parts.append(f"settled_in={settled}")
    elif outcome is TerminalOutcome.DROPPED_OBJECT:
        parts.append(f"target={target_bin}")
    return ";".join(parts)


def _stable_grasp_verified(
    samples: tuple[GraspSample, ...],
    initial_position: Vector3,
    config: OutcomeClassificationConfig,
) -> bool:
    return _first_verified_grasp_time(samples, initial_position, config) is not None


def _first_verified_grasp_time(
    samples: tuple[GraspSample, ...],
    initial_position: Vector3,
    config: OutcomeClassificationConfig,
) -> float | None:
    run_start: float | None = None
    for sample in samples:
        if _is_stable_grasp_sample(sample, initial_position, config):
            if run_start is None:
                run_start = sample.simulation_time_seconds
            if _duration_met(
                sample.simulation_time_seconds - run_start,
                config.grasp_contact_duration_seconds,
            ):
                return sample.simulation_time_seconds
        else:
            run_start = None
    return None


def _is_stable_grasp_sample(
    sample: GraspSample,
    initial_position: Vector3,
    config: OutcomeClassificationConfig,
) -> bool:
    elevated = sample.object_position_meters.z >= initial_position.z + config.grasp_elevation_meters
    contacting = (
        sample.gripper_object_contact
        and sample.contact_force_newtons >= config.grasp_contact_force_newtons
    )
    return elevated and contacting


def _grasp_lost_outside_target(
    samples: tuple[GraspSample, ...],
    initial_position: Vector3,
    target_bin: str,
    bin_poses: tuple[tuple[str, Pose], ...],
    config: OutcomeClassificationConfig,
    *,
    released: bool,
) -> bool:
    if released:
        return False
    verified_at = _first_verified_grasp_time(samples, initial_position, config)
    if verified_at is None:
        return False
    poses = dict(bin_poses)
    target_pose = poses[target_bin]
    for sample in samples:
        if sample.simulation_time_seconds < verified_at:
            continue
        if _is_stable_grasp_sample(sample, initial_position, config):
            continue
        if object_center_in_bin(sample.object_position_meters, target_pose, config):
            continue
        return True
    return False


def _placement_settlement(
    samples: tuple[PlacementSample, ...],
    target_bin: str,
    bin_poses: tuple[tuple[str, Pose], ...],
    config: OutcomeClassificationConfig,
) -> tuple[bool, str | None, bool]:
    if not samples:
        return False, None, False
    last_time = samples[-1].simulation_time_seconds
    window = tuple(
        sample
        for sample in samples
        if _within_window(last_time - sample.simulation_time_seconds, config.settle_window_seconds)
    )
    if not window:
        return False, None, False
    span = window[-1].simulation_time_seconds - window[0].simulation_time_seconds
    if not _duration_met(span, config.settle_window_seconds):
        return False, None, False
    if any(not _at_rest(sample, config) for sample in window):
        return False, None, False
    regions = {
        _containing_bin(sample.object_position_meters, bin_poses, config) for sample in window
    }
    if len(regions) != 1:
        return False, None, False
    settled_bin = next(iter(regions))
    in_target = settled_bin == target_bin
    return True, settled_bin, in_target


def _containing_bin(
    position: Vector3,
    bin_poses: tuple[tuple[str, Pose], ...],
    config: OutcomeClassificationConfig,
) -> str | None:
    for name, pose in bin_poses:
        if object_center_in_bin(position, pose, config):
            return name
    return None


def _duration_met(elapsed_seconds: float, required_seconds: float) -> bool:
    return elapsed_seconds + TIME_COMPARISON_EPSILON_SECONDS >= required_seconds


def _within_window(elapsed_seconds: float, window_seconds: float) -> bool:
    return elapsed_seconds <= window_seconds + TIME_COMPARISON_EPSILON_SECONDS


def _at_rest(sample: PlacementSample, config: OutcomeClassificationConfig) -> bool:
    return (
        sample.linear_speed_meters_per_second <= config.settle_linear_speed_meters_per_second
        and sample.angular_speed_radians_per_second
        <= config.settle_angular_speed_radians_per_second
    )


def _timeout_observed(evidence: OutcomeEvidence) -> bool:
    return any(event.event_type is EventType.TIMEOUT for event in evidence.events)


def _gripper_closed_from_events(events: tuple[TrialEvent, ...]) -> bool:
    return any(
        event.event_type is EventType.STATE_END and event.controller_state is ControllerState.GRASP
        for event in events
    )


def _released_from_events(events: tuple[TrialEvent, ...]) -> bool:
    return any(
        event.event_type is EventType.STATE_START
        and event.controller_state is ControllerState.RELEASE
        for event in events
    )


def _looks_unsafe(value: str) -> bool:
    lowered = value.lower()
    return any(marker in lowered for marker in _UNSAFE_DETAIL_MARKERS)


def _require_inner_xy(value: object) -> tuple[float, float]:
    if not isinstance(value, tuple) or len(value) != 2:
        msg = "bin_inner_xy_meters must be a pair of positive lengths"
        raise ValueError(msg)
    return (
        require_positive(
            "bin_inner_xy_meters.x", require_finite("bin_inner_xy_meters.x", value[0])
        ),
        require_positive(
            "bin_inner_xy_meters.y", require_finite("bin_inner_xy_meters.y", value[1])
        ),
    )


def _require_bin_poses(value: object) -> tuple[tuple[str, Pose], ...]:
    if not isinstance(value, tuple) or not value:
        msg = "bin_poses must be a tuple of name and Pose pairs"
        raise ValueError(msg)
    names: list[str] = []
    poses: list[tuple[str, Pose]] = []
    for item in value:
        if not isinstance(item, tuple) or len(item) != 2:
            msg = "bin_poses must be a tuple of name and Pose pairs"
            raise ValueError(msg)
        name, pose = item
        trimmed = require_name("bin_name", name)
        if trimmed not in WORKCELL_BIN_NAMES:
            msg = "bin_poses must use workcell bin names"
            raise ValueError(msg)
        if not isinstance(pose, Pose):
            msg = "bin_poses must be a tuple of name and Pose pairs"
            raise ValueError(msg)
        if pose.frame != WORLD_FRAME:
            msg = "bin poses must use the world frame"
            raise ValueError(msg)
        names.append(trimmed)
        poses.append((trimmed, pose))
    if tuple(names) != WORKCELL_BIN_NAMES:
        msg = "bin_poses must include every workcell bin in order"
        raise ValueError(msg)
    return tuple(poses)


def _require_grasp_samples(value: object) -> tuple[GraspSample, ...]:
    samples = _require_sample_tuple(value, GraspSample, "grasp_samples")
    _require_nondecreasing_times(
        tuple(sample.simulation_time_seconds for sample in samples), "grasp_samples"
    )
    return samples


def _require_placement_samples(value: object) -> tuple[PlacementSample, ...]:
    samples = _require_sample_tuple(value, PlacementSample, "placement_samples")
    _require_nondecreasing_times(
        tuple(sample.simulation_time_seconds for sample in samples), "placement_samples"
    )
    return samples


def _require_sample_tuple[SampleT](
    value: object, sample_type: type[SampleT], field: str
) -> tuple[SampleT, ...]:
    if not isinstance(value, tuple):
        msg = f"{field} must be a tuple of {sample_type.__name__} values"
        raise ValueError(msg)
    if any(not isinstance(item, sample_type) for item in value):
        msg = f"{field} must be a tuple of {sample_type.__name__} values"
        raise ValueError(msg)
    return value


def _require_nondecreasing_times(times: Sequence[float], field: str) -> None:
    previous: float | None = None
    for time_seconds in times:
        if previous is not None and time_seconds < previous:
            msg = f"{field} simulation_time_seconds must be nondecreasing"
            raise ValueError(msg)
        previous = time_seconds


def _require_monotonic_events(events: tuple[TrialEvent, ...]) -> None:
    previous_ordinal = -1
    previous_time: float | None = None
    for event in events:
        if event.ordinal != previous_ordinal + 1:
            msg = "event ordinals must be contiguous and increasing"
            raise ValueError(msg)
        if previous_time is not None and event.simulation_time_seconds < previous_time:
            msg = "event simulation_time_seconds must be nondecreasing"
            raise ValueError(msg)
        previous_ordinal = event.ordinal
        previous_time = event.simulation_time_seconds
