from __future__ import annotations

import json

import pytest
from robot_control_platform_simulator.control.state_machine import (
    ALLOWED_TRANSITIONS,
    HAPPY_PATH_TRANSITIONS,
    ControllerStateMachine,
    InvalidControllerTransition,
    TrialEventLog,
    allowed_targets,
    is_allowed_transition,
)
from robot_control_platform_simulator.domain.enums import ControllerState, EventType
from robot_control_platform_simulator.domain.events import ContactEvent, TrialEvent
from robot_control_platform_simulator.domain.models import Vector3

_FROZEN_STATES = (
    ControllerState.RESET,
    ControllerState.OBSERVE,
    ControllerState.PLAN,
    ControllerState.APPROACH,
    ControllerState.GRASP,
    ControllerState.VERIFY_GRASP,
    ControllerState.LIFT,
    ControllerState.TRANSFER,
    ControllerState.RELEASE,
    ControllerState.VERIFY_PLACE,
    ControllerState.RETRACT,
    ControllerState.TERMINAL,
)
_ALLOWED_CASES = sorted(ALLOWED_TRANSITIONS, key=lambda pair: (pair[0].value, pair[1].value))
_REJECTED_CASES = (
    (ControllerState.RESET, ControllerState.GRASP),
    (ControllerState.OBSERVE, ControllerState.RESET),
    (ControllerState.APPROACH, ControllerState.APPROACH),
    (ControllerState.LIFT, ControllerState.OBSERVE),
    (ControllerState.VERIFY_PLACE, ControllerState.GRASP),
    (ControllerState.TERMINAL, ControllerState.RETRACT),
    (ControllerState.TERMINAL, ControllerState.TERMINAL),
)


def _contact(*, force_newtons: float = 1.5, time_seconds: float = 0.2) -> ContactEvent:
    return ContactEvent(
        body_a="kuka_iiwa",
        body_b="bin_red",
        link_a="lbr_iiwa_link_4",
        link_b="base",
        position_meters=Vector3(x=0.4, y=0.0, z=0.7),
        normal=Vector3(x=0.0, y=0.0, z=1.0),
        force_newtons=force_newtons,
        simulation_time_seconds=time_seconds,
    )


def test_frozen_controller_states_match_the_allowed_table() -> None:
    assert tuple(ControllerState) == _FROZEN_STATES
    sources = {source for source, _target in ALLOWED_TRANSITIONS}
    targets = {target for _source, target in ALLOWED_TRANSITIONS}
    assert ControllerState.TERMINAL not in sources
    assert allowed_targets(ControllerState.TERMINAL) == frozenset()
    assert ControllerState.RESET not in targets
    assert sources | targets == set(ControllerState)
    for source, target in HAPPY_PATH_TRANSITIONS:
        assert is_allowed_transition(source, target)


@pytest.mark.parametrize(
    ("source", "target"),
    _ALLOWED_CASES,
    ids=[f"{source.value}->{target.value}" for source, target in _ALLOWED_CASES],
)
def test_every_allowed_transition_is_accepted(
    source: ControllerState, target: ControllerState
) -> None:
    machine = ControllerStateMachine(initial_state=source, simulation_time_seconds=0.0)
    ended, started = machine.transition(target, simulation_time_seconds=0.1)
    assert machine.state is target
    assert ended.event_type is EventType.STATE_END
    assert ended.controller_state is source
    assert started.event_type is EventType.STATE_START
    assert started.controller_state is target
    assert ended.ordinal + 1 == started.ordinal


@pytest.mark.parametrize(
    ("source", "target"),
    _REJECTED_CASES,
    ids=[f"{source.value}->{target.value}" for source, target in _REJECTED_CASES],
)
def test_representative_illegal_transitions_are_rejected(
    source: ControllerState, target: ControllerState
) -> None:
    machine = ControllerStateMachine(initial_state=source, simulation_time_seconds=0.0)
    before = machine.events
    with pytest.raises(InvalidControllerTransition, match=f"{source.value} -> {target.value}"):
        machine.transition(target, simulation_time_seconds=0.1)
    assert machine.state is source
    assert machine.events == before
    assert not is_allowed_transition(source, target)


def test_happy_path_emits_monotonic_start_and_end_ordinals() -> None:
    machine = ControllerStateMachine()
    time_seconds = 0.0
    for source, target in HAPPY_PATH_TRANSITIONS:
        assert machine.state is source
        time_seconds += 0.1
        machine.transition(target, simulation_time_seconds=time_seconds)
    assert machine.state is ControllerState.TERMINAL
    ordinals = [event.ordinal for event in machine.events]
    assert ordinals == list(range(len(machine.events)))
    types = [event.event_type for event in machine.events]
    assert types[0] is EventType.STATE_START
    assert types[0:3] == [EventType.STATE_START, EventType.STATE_END, EventType.STATE_START]
    assert types[-2:] == [EventType.STATE_END, EventType.STATE_START]
    assert types.count(EventType.STATE_START) == len(_FROZEN_STATES)
    assert types.count(EventType.STATE_END) == len(HAPPY_PATH_TRANSITIONS)
    times = [event.simulation_time_seconds for event in machine.events]
    assert times == sorted(times)


def test_failure_transition_emits_state_failure_and_keeps_ordinals_monotonic() -> None:
    machine = ControllerStateMachine()
    machine.transition(ControllerState.OBSERVE, simulation_time_seconds=0.1)
    machine.transition(
        ControllerState.RETRACT,
        simulation_time_seconds=0.2,
        failed=True,
        detail="collision",
    )
    machine.transition(ControllerState.TERMINAL, simulation_time_seconds=0.3)
    types = [event.event_type for event in machine.events]
    assert types == [
        EventType.STATE_START,
        EventType.STATE_END,
        EventType.STATE_START,
        EventType.STATE_FAILURE,
        EventType.STATE_START,
        EventType.STATE_END,
        EventType.STATE_START,
    ]
    failure = machine.events[3]
    assert failure.controller_state is ControllerState.OBSERVE
    assert failure.detail == "collision"
    assert [event.ordinal for event in machine.events] == list(range(7))
    assert machine.state is ControllerState.TERMINAL


def test_contact_events_share_the_monotonic_ordinal_stream() -> None:
    machine = ControllerStateMachine()
    machine.transition(ControllerState.OBSERVE, simulation_time_seconds=0.1)
    recorded = machine.record_contact(_contact(time_seconds=0.2))
    assert recorded.event_type is EventType.CONTACT
    assert recorded.ordinal == 3
    assert recorded.controller_state is ControllerState.OBSERVE
    assert recorded.contact == _contact(time_seconds=0.2)
    assert machine.state is ControllerState.OBSERVE
    machine.transition(ControllerState.PLAN, simulation_time_seconds=0.3)
    assert [event.ordinal for event in machine.events] == list(range(6))
    assert [event.event_type for event in machine.events] == [
        EventType.STATE_START,
        EventType.STATE_END,
        EventType.STATE_START,
        EventType.CONTACT,
        EventType.STATE_END,
        EventType.STATE_START,
    ]


def test_event_log_rejects_decreasing_simulation_time() -> None:
    machine = ControllerStateMachine()
    machine.transition(ControllerState.OBSERVE, simulation_time_seconds=0.4)
    with pytest.raises(ValueError, match="nondecreasing"):
        machine.transition(ControllerState.PLAN, simulation_time_seconds=0.3)
    assert machine.state is ControllerState.OBSERVE
    with pytest.raises(ValueError, match="nondecreasing"):
        machine.record_contact(_contact(time_seconds=0.2))


def test_trial_event_round_trip_and_rejects_invalid_contact_pairing() -> None:
    event = TrialEvent(
        ordinal=4,
        event_type=EventType.CONTACT,
        controller_state=ControllerState.APPROACH,
        simulation_time_seconds=1.25,
        contact=_contact(time_seconds=1.25),
    )
    restored = TrialEvent.from_checksum_payload(json.loads(event.canonical_json()))
    assert restored == event
    assert restored.sha256_hex() == event.sha256_hex()
    with pytest.raises(ValueError, match="nonnegative integer"):
        TrialEvent(
            ordinal=-1,
            event_type=EventType.STATE_START,
            controller_state=ControllerState.RESET,
            simulation_time_seconds=0.0,
        )
    with pytest.raises(ValueError, match="require a ContactEvent"):
        TrialEvent(
            ordinal=0,
            event_type=EventType.CONTACT,
            controller_state=ControllerState.APPROACH,
            simulation_time_seconds=0.0,
        )
    with pytest.raises(ValueError, match="only allowed on contact events"):
        TrialEvent(
            ordinal=0,
            event_type=EventType.STATE_START,
            controller_state=ControllerState.RESET,
            simulation_time_seconds=0.0,
            contact=_contact(),
        )


def test_standalone_event_log_assigns_zero_based_ordinals() -> None:
    log = TrialEventLog()
    first = log.record(EventType.STATE_START, ControllerState.RESET, simulation_time_seconds=0.0)
    second = log.record(EventType.TIMEOUT, ControllerState.APPROACH, simulation_time_seconds=0.0)
    assert first.ordinal == 0
    assert second.ordinal == 1
    assert second.event_type is EventType.TIMEOUT
