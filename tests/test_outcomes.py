from __future__ import annotations

from dataclasses import replace

import pytest
from robot_control_platform_simulator.control.outcomes import (
    ALLOWED_SYSTEM_ERROR_CODES,
    OUTCOME_PRECEDENCE,
    GraspSample,
    OutcomeClassificationConfig,
    OutcomeEvidence,
    PlacementSample,
    classify_outcome,
    default_bin_poses,
    default_outcome_classification_config,
    emit_outcome_event,
    is_gripper_object_contact,
    object_center_in_bin,
    sanitize_infrastructure_failure,
)
from robot_control_platform_simulator.control.state_machine import TrialEventLog
from robot_control_platform_simulator.domain.enums import (
    ControllerState,
    EventType,
    TerminalOutcome,
)
from robot_control_platform_simulator.domain.events import ContactEvent, TrialEvent
from robot_control_platform_simulator.domain.models import Vector3
from robot_control_platform_simulator.physics.client import SimulationError
from robot_control_platform_simulator.physics.scene import ROBOT_BODY_NAME, TABLE_TOP_Z_METERS

_OBJECT_ID = "parcel_0"
_TARGET = "bin_red"
_INITIAL = Vector3(x=0.50, y=0.0, z=TABLE_TOP_Z_METERS + 0.025)
_LIFTED = Vector3(x=0.50, y=0.0, z=_INITIAL.z + 0.08)
_TABLE_DROP = Vector3(x=0.50, y=0.12, z=_INITIAL.z)


def _bin_center(name: str, *, z: float | None = None) -> Vector3:
    pose = dict(default_bin_poses())[name]
    height = _INITIAL.z if z is None else z
    return Vector3(x=pose.position_meters.x, y=pose.position_meters.y, z=height)


def _grasp(
    *,
    time_seconds: float,
    position: Vector3,
    contact: bool,
    force_newtons: float = 8.0,
) -> GraspSample:
    return GraspSample(
        simulation_time_seconds=time_seconds,
        object_position_meters=position,
        gripper_object_contact=contact,
        contact_force_newtons=force_newtons if contact else 0.0,
    )


def _place(
    *,
    time_seconds: float,
    position: Vector3,
    linear_speed: float = 0.0,
    angular_speed: float = 0.0,
) -> PlacementSample:
    return PlacementSample(
        simulation_time_seconds=time_seconds,
        object_position_meters=position,
        linear_speed_meters_per_second=linear_speed,
        angular_speed_radians_per_second=angular_speed,
    )


def _timeout_event(time_seconds: float = 4.0) -> TrialEvent:
    return TrialEvent(
        ordinal=0,
        event_type=EventType.TIMEOUT,
        controller_state=ControllerState.TRANSFER,
        simulation_time_seconds=time_seconds,
        detail="timeout",
    )


def _grasp_close_events() -> tuple[TrialEvent, ...]:
    return (
        TrialEvent(
            ordinal=0,
            event_type=EventType.STATE_START,
            controller_state=ControllerState.GRASP,
            simulation_time_seconds=0.4,
        ),
        TrialEvent(
            ordinal=1,
            event_type=EventType.STATE_END,
            controller_state=ControllerState.GRASP,
            simulation_time_seconds=0.6,
        ),
    )


def _release_event() -> TrialEvent:
    return TrialEvent(
        ordinal=0,
        event_type=EventType.STATE_START,
        controller_state=ControllerState.RELEASE,
        simulation_time_seconds=2.0,
        detail="release",
    )


def _stable_grasp_samples() -> tuple[GraspSample, ...]:
    return (
        _grasp(time_seconds=0.50, position=_INITIAL, contact=True),
        _grasp(time_seconds=0.56, position=_LIFTED, contact=True),
        _grasp(time_seconds=0.70, position=_LIFTED, contact=True),
    )


def _success_placement() -> tuple[PlacementSample, ...]:
    center = _bin_center(_TARGET)
    return (
        _place(time_seconds=2.00, position=center, linear_speed=0.04),
        _place(time_seconds=2.10, position=center, linear_speed=0.01),
    )


def _evidence(
    *,
    events: tuple[TrialEvent, ...] = (),
    collision_detected: bool = False,
    infrastructure_failure: str | None = None,
    gripper_closed: bool = True,
    released: bool = True,
    target_bin: str = _TARGET,
    grasp_samples: tuple[GraspSample, ...] | None = None,
    placement_samples: tuple[PlacementSample, ...] | None = None,
) -> OutcomeEvidence:
    return OutcomeEvidence(
        events=events,
        collision_detected=collision_detected,
        infrastructure_failure=infrastructure_failure,
        gripper_closed=gripper_closed,
        released=released,
        object_id=_OBJECT_ID,
        target_bin=target_bin,
        bin_poses=default_bin_poses(),
        initial_object_position_meters=_INITIAL,
        grasp_samples=_stable_grasp_samples() if grasp_samples is None else grasp_samples,
        placement_samples=_success_placement() if placement_samples is None else placement_samples,
    )


def _success_fixture() -> OutcomeEvidence:
    return _evidence()


def _system_error_fixture() -> OutcomeEvidence:
    return _evidence(infrastructure_failure="simulation_error")


def _collision_fixture() -> OutcomeEvidence:
    return _evidence(collision_detected=True)


def _missed_grasp_fixture() -> OutcomeEvidence:
    return _evidence(
        gripper_closed=True,
        released=False,
        grasp_samples=(
            _grasp(time_seconds=0.50, position=_INITIAL, contact=True, force_newtons=0.2),
            _grasp(time_seconds=0.60, position=_INITIAL, contact=False),
        ),
        placement_samples=(),
    )


def _dropped_object_fixture() -> OutcomeEvidence:
    return _evidence(
        released=False,
        grasp_samples=(
            *_stable_grasp_samples(),
            _grasp(time_seconds=1.20, position=_TABLE_DROP, contact=False),
        ),
        placement_samples=(),
    )


def _wrong_bin_fixture() -> OutcomeEvidence:
    other = _bin_center("bin_blue")
    return _evidence(
        events=(_release_event(),),
        released=True,
        placement_samples=(
            _place(time_seconds=2.00, position=other),
            _place(time_seconds=2.10, position=other),
        ),
    )


def _collision_and_success_fixture() -> OutcomeEvidence:
    return _evidence(collision_detected=True)


def _system_error_and_collision_fixture() -> OutcomeEvidence:
    return _evidence(collision_detected=True, infrastructure_failure="simulation_error")


def _missed_grasp_and_wrong_bin_fixture() -> OutcomeEvidence:
    other = _bin_center("bin_green")
    return _evidence(
        gripper_closed=True,
        released=True,
        grasp_samples=(
            _grasp(time_seconds=0.50, position=_INITIAL, contact=False),
            _grasp(time_seconds=0.60, position=_INITIAL, contact=False),
        ),
        placement_samples=(
            _place(time_seconds=2.00, position=other),
            _place(time_seconds=2.10, position=other),
        ),
    )


def _drop_into_wrong_bin_fixture() -> OutcomeEvidence:
    other = _bin_center("bin_yellow")
    return _evidence(
        released=False,
        grasp_samples=(
            *_stable_grasp_samples(),
            _grasp(time_seconds=1.40, position=other, contact=False),
        ),
        placement_samples=(
            _place(time_seconds=2.00, position=other),
            _place(time_seconds=2.10, position=other),
        ),
    )


def _timeout_unevaluable_fixture() -> OutcomeEvidence:
    return _evidence(
        events=(_timeout_event(),),
        gripper_closed=False,
        released=False,
        grasp_samples=(_grasp(time_seconds=0.20, position=_INITIAL, contact=False),),
        placement_samples=(),
    )


def _timeout_with_collision_fixture() -> OutcomeEvidence:
    return _evidence(events=(_timeout_event(),), collision_detected=True)


_COMPLETED_FIXTURES: tuple[tuple[str, OutcomeEvidence, TerminalOutcome], ...] = (
    ("system_error", _system_error_fixture(), TerminalOutcome.SYSTEM_ERROR),
    ("collision", _collision_fixture(), TerminalOutcome.COLLISION),
    ("missed_grasp", _missed_grasp_fixture(), TerminalOutcome.MISSED_GRASP),
    ("dropped_object", _dropped_object_fixture(), TerminalOutcome.DROPPED_OBJECT),
    ("wrong_bin", _wrong_bin_fixture(), TerminalOutcome.WRONG_BIN),
    ("success", _success_fixture(), TerminalOutcome.SUCCESS),
    (
        "ambiguous_system_error_over_collision",
        _system_error_and_collision_fixture(),
        TerminalOutcome.SYSTEM_ERROR,
    ),
    (
        "ambiguous_collision_over_success",
        _collision_and_success_fixture(),
        TerminalOutcome.COLLISION,
    ),
    (
        "ambiguous_missed_grasp_over_wrong_bin",
        _missed_grasp_and_wrong_bin_fixture(),
        TerminalOutcome.MISSED_GRASP,
    ),
    (
        "ambiguous_drop_over_wrong_bin",
        _drop_into_wrong_bin_fixture(),
        TerminalOutcome.DROPPED_OBJECT,
    ),
    (
        "timeout_unevaluable_system_error",
        _timeout_unevaluable_fixture(),
        TerminalOutcome.SYSTEM_ERROR,
    ),
    (
        "timeout_maps_to_collision",
        _timeout_with_collision_fixture(),
        TerminalOutcome.COLLISION,
    ),
)


@pytest.mark.parametrize(
    ("name", "evidence", "expected"),
    _COMPLETED_FIXTURES,
    ids=[name for name, _evidence, _expected in _COMPLETED_FIXTURES],
)
def test_every_completed_fixture_yields_exactly_one_outcome(
    name: str, evidence: OutcomeEvidence, expected: TerminalOutcome
) -> None:
    result = classify_outcome(evidence)
    assert isinstance(result.outcome, TerminalOutcome)
    assert result.outcome is expected
    assert list(TerminalOutcome).count(result.outcome) == 1
    assert name
    restored = classify_outcome(evidence, default_outcome_classification_config())
    assert restored.outcome is result.outcome
    assert restored.event_detail == result.event_detail


def test_ambiguous_precedence_keeps_losing_conditions_visible() -> None:
    collision_over_success = classify_outcome(_collision_and_success_fixture())
    assert collision_over_success.outcome is TerminalOutcome.COLLISION
    assert collision_over_success.collision_detected is True
    assert collision_over_success.grasp_verified is True
    assert collision_over_success.settled is True
    assert collision_over_success.in_target_bin is True

    system_over_collision = classify_outcome(_system_error_and_collision_fixture())
    assert system_over_collision.outcome is TerminalOutcome.SYSTEM_ERROR
    assert system_over_collision.collision_detected is True

    missed_over_wrong = classify_outcome(_missed_grasp_and_wrong_bin_fixture())
    assert missed_over_wrong.outcome is TerminalOutcome.MISSED_GRASP
    assert missed_over_wrong.settled is True
    assert missed_over_wrong.in_target_bin is False

    drop_over_wrong = classify_outcome(_drop_into_wrong_bin_fixture())
    assert drop_over_wrong.outcome is TerminalOutcome.DROPPED_OBJECT
    assert drop_over_wrong.grasp_lost_outside_target is True
    assert drop_over_wrong.settled is True
    assert drop_over_wrong.settled_bin == "bin_yellow"


def test_frozen_precedence_order() -> None:
    assert OUTCOME_PRECEDENCE == (
        TerminalOutcome.SYSTEM_ERROR,
        TerminalOutcome.COLLISION,
        TerminalOutcome.MISSED_GRASP,
        TerminalOutcome.DROPPED_OBJECT,
        TerminalOutcome.WRONG_BIN,
        TerminalOutcome.SUCCESS,
    )
    assert tuple(TerminalOutcome) == OUTCOME_PRECEDENCE


def test_stable_grasp_requires_elevation_and_sustained_contact() -> None:
    contact_only = _evidence(
        gripper_closed=True,
        released=False,
        grasp_samples=(
            _grasp(time_seconds=0.50, position=_INITIAL, contact=True),
            _grasp(time_seconds=0.62, position=_INITIAL, contact=True),
        ),
        placement_samples=(),
    )
    elevation_only = _evidence(
        gripper_closed=True,
        released=False,
        grasp_samples=(
            _grasp(time_seconds=0.50, position=_LIFTED, contact=False),
            _grasp(time_seconds=0.62, position=_LIFTED, contact=False),
        ),
        placement_samples=(),
    )
    brief_contact = _evidence(
        gripper_closed=True,
        released=False,
        grasp_samples=(_grasp(time_seconds=0.50, position=_LIFTED, contact=True),),
        placement_samples=(),
    )
    assert classify_outcome(contact_only).grasp_verified is False
    assert classify_outcome(contact_only).outcome is TerminalOutcome.MISSED_GRASP
    assert classify_outcome(elevation_only).grasp_verified is False
    assert classify_outcome(brief_contact).grasp_verified is False
    verified = classify_outcome(_dropped_object_fixture())
    assert verified.grasp_verified is True


def test_drop_requires_a_previously_verified_stable_grasp() -> None:
    never_grasped = _evidence(
        gripper_closed=True,
        released=False,
        grasp_samples=(
            _grasp(time_seconds=0.50, position=_INITIAL, contact=False),
            _grasp(time_seconds=1.20, position=_TABLE_DROP, contact=False),
        ),
        placement_samples=(),
    )
    result = classify_outcome(never_grasped)
    assert result.grasp_verified is False
    assert result.grasp_lost_outside_target is False
    assert result.outcome is TerminalOutcome.MISSED_GRASP
    dropped = classify_outcome(_dropped_object_fixture())
    assert dropped.grasp_verified is True
    assert dropped.grasp_lost_outside_target is True
    assert dropped.outcome is TerminalOutcome.DROPPED_OBJECT


def test_success_requires_center_containment_with_margin_after_settling() -> None:
    config = default_outcome_classification_config()
    center = _bin_center(_TARGET)
    pose = dict(default_bin_poses())[_TARGET]
    assert object_center_in_bin(center, pose, config)
    outside_margin = Vector3(x=center.x + 0.08, y=center.y, z=center.z)
    assert not object_center_in_bin(outside_margin, pose, config)
    hovering = Vector3(x=center.x, y=center.y, z=center.z + 0.20)
    assert not object_center_in_bin(hovering, pose, config)
    moving = _evidence(
        placement_samples=(
            _place(time_seconds=2.00, position=center, linear_speed=0.40),
            _place(time_seconds=2.10, position=center, linear_speed=0.40),
        )
    )
    short_window = _evidence(placement_samples=(_place(time_seconds=2.00, position=center),))
    margin_miss = _evidence(
        placement_samples=(
            _place(time_seconds=2.00, position=outside_margin),
            _place(time_seconds=2.10, position=outside_margin),
        )
    )
    assert classify_outcome(moving).outcome is not TerminalOutcome.SUCCESS
    assert classify_outcome(short_window).settled is False
    assert classify_outcome(margin_miss).outcome is TerminalOutcome.WRONG_BIN
    assert classify_outcome(_success_fixture()).outcome is TerminalOutcome.SUCCESS
    assert classify_outcome(_success_fixture()).in_target_bin is True


def test_wrong_bin_includes_a_detailed_settled_region_event() -> None:
    result = classify_outcome(_wrong_bin_fixture())
    assert result.outcome is TerminalOutcome.WRONG_BIN
    assert result.settled_bin == "bin_blue"
    assert result.event_detail == "outcome=wrong_bin;target=bin_red;settled_in=bin_blue"
    log = TrialEventLog()
    recorded = emit_outcome_event(log, result, simulation_time_seconds=2.10)
    assert recorded.event_type is EventType.OBSERVATION
    assert recorded.controller_state is ControllerState.TERMINAL
    assert recorded.detail == result.event_detail
    assert recorded.ordinal == 0
    outside = _evidence(
        released=True,
        placement_samples=(
            _place(time_seconds=2.00, position=_TABLE_DROP),
            _place(time_seconds=2.10, position=_TABLE_DROP),
        ),
    )
    table_result = classify_outcome(outside)
    assert table_result.outcome is TerminalOutcome.WRONG_BIN
    assert table_result.settled_bin is None
    assert "settled_in=none" in table_result.event_detail


def test_infrastructure_failure_maps_to_sanitized_system_error() -> None:
    raw_path = "/untrusted/workcell/kuka.urdf"
    assert sanitize_infrastructure_failure(raw_path) == "simulation_error"
    assert sanitize_infrastructure_failure(SimulationError("reset failed")) == "simulation_error"
    assert sanitize_infrastructure_failure("timeout_unevaluable") == "timeout_unevaluable"
    result = classify_outcome(_evidence(infrastructure_failure=raw_path, collision_detected=True))
    assert result.outcome is TerminalOutcome.SYSTEM_ERROR
    assert result.system_error_code == "simulation_error"
    assert result.event_detail == "outcome=system_error;code=simulation_error"
    assert raw_path not in result.reason
    assert raw_path not in result.event_detail
    assert "urdf" not in result.reason.lower()
    assert result.system_error_code in ALLOWED_SYSTEM_ERROR_CODES


def test_timeout_maps_to_evaluable_task_outcome_or_system_error() -> None:
    unevaluable = classify_outcome(_timeout_unevaluable_fixture())
    assert unevaluable.timeout_observed is True
    assert unevaluable.outcome is TerminalOutcome.SYSTEM_ERROR
    assert unevaluable.system_error_code == "timeout_unevaluable"
    collided = classify_outcome(_timeout_with_collision_fixture())
    assert collided.timeout_observed is True
    assert collided.outcome is TerminalOutcome.COLLISION
    missed = classify_outcome(replace(_missed_grasp_fixture(), events=(_timeout_event(),)))
    assert missed.timeout_observed is True
    assert missed.outcome is TerminalOutcome.MISSED_GRASP


def test_controlled_release_into_wrong_bin_is_not_a_drop() -> None:
    result = classify_outcome(_wrong_bin_fixture())
    assert result.released is True
    assert result.grasp_verified is True
    assert result.grasp_lost_outside_target is False
    assert result.outcome is TerminalOutcome.WRONG_BIN


def test_gripper_object_contact_uses_finger_links_only() -> None:
    finger = ContactEvent(
        body_a=ROBOT_BODY_NAME,
        body_b=_OBJECT_ID,
        link_a="left_finger",
        link_b="base",
        position_meters=Vector3(x=0.5, y=0.0, z=0.7),
        normal=Vector3(x=0.0, y=0.0, z=1.0),
        force_newtons=6.0,
        simulation_time_seconds=0.5,
    )
    arm = replace(finger, link_a="lbr_iiwa_link_4")
    table = replace(finger, body_a="table", link_a="base")
    assert is_gripper_object_contact(finger, _OBJECT_ID) is True
    assert is_gripper_object_contact(arm, _OBJECT_ID) is False
    assert is_gripper_object_contact(table, _OBJECT_ID) is False
    with pytest.raises(ValueError, match="must not be the robot body"):
        is_gripper_object_contact(finger, ROBOT_BODY_NAME)


def test_outcome_config_checksum_is_stable_and_rejects_oversize_margin() -> None:
    first = default_outcome_classification_config()
    second = default_outcome_classification_config()
    assert first.canonical_json() == second.canonical_json()
    assert first.sha256_hex() == second.sha256_hex()
    with pytest.raises(ValueError, match="smaller than bin inner half-extents"):
        OutcomeClassificationConfig(
            grasp_elevation_meters=0.02,
            grasp_contact_duration_seconds=0.05,
            grasp_contact_force_newtons=1.0,
            bin_containment_margin_meters=0.10,
            settle_window_seconds=0.10,
            settle_linear_speed_meters_per_second=0.05,
            settle_angular_speed_radians_per_second=0.5,
            bin_inner_xy_meters=(0.18, 0.18),
            bin_height_meters=0.10,
            table_top_z_meters=TABLE_TOP_Z_METERS,
        )


def test_events_must_have_contiguous_ordinals() -> None:
    with pytest.raises(ValueError, match="contiguous"):
        _evidence(
            events=(
                TrialEvent(
                    ordinal=2,
                    event_type=EventType.TIMEOUT,
                    controller_state=ControllerState.APPROACH,
                    simulation_time_seconds=1.0,
                    detail="timeout",
                ),
            )
        )


def test_grasp_close_event_counts_as_gripper_closed() -> None:
    evidence = _evidence(
        events=_grasp_close_events(),
        gripper_closed=False,
        released=False,
        grasp_samples=(
            _grasp(time_seconds=0.50, position=_INITIAL, contact=False),
            _grasp(time_seconds=0.70, position=_INITIAL, contact=False),
        ),
        placement_samples=(),
    )
    result = classify_outcome(evidence)
    assert result.gripper_closed is True
    assert result.outcome is TerminalOutcome.MISSED_GRASP
