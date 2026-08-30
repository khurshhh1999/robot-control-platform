"""Controller state machine with data-defined transitions and monotonic trial events."""

from __future__ import annotations

from typing import Final

from robot_control_platform_simulator.domain.enums import ControllerState, EventType
from robot_control_platform_simulator.domain.events import ContactEvent, TrialEvent
from robot_control_platform_simulator.domain.models import require_finite, require_nonnegative

HAPPY_PATH_TRANSITIONS: Final[tuple[tuple[ControllerState, ControllerState], ...]] = (
    (ControllerState.RESET, ControllerState.OBSERVE),
    (ControllerState.OBSERVE, ControllerState.PLAN),
    (ControllerState.PLAN, ControllerState.APPROACH),
    (ControllerState.APPROACH, ControllerState.GRASP),
    (ControllerState.GRASP, ControllerState.VERIFY_GRASP),
    (ControllerState.VERIFY_GRASP, ControllerState.LIFT),
    (ControllerState.LIFT, ControllerState.TRANSFER),
    (ControllerState.TRANSFER, ControllerState.RELEASE),
    (ControllerState.RELEASE, ControllerState.VERIFY_PLACE),
    (ControllerState.VERIFY_PLACE, ControllerState.RETRACT),
    (ControllerState.RETRACT, ControllerState.TERMINAL),
)
_REGRASP_TRANSITIONS: Final[tuple[tuple[ControllerState, ControllerState], ...]] = (
    (ControllerState.VERIFY_GRASP, ControllerState.APPROACH),
    (ControllerState.RETRACT, ControllerState.APPROACH),
)
_ABORT_SOURCES: Final[tuple[ControllerState, ...]] = (
    ControllerState.OBSERVE,
    ControllerState.PLAN,
    ControllerState.APPROACH,
    ControllerState.GRASP,
    ControllerState.VERIFY_GRASP,
    ControllerState.LIFT,
    ControllerState.TRANSFER,
    ControllerState.RELEASE,
    ControllerState.VERIFY_PLACE,
)
_ERROR_SOURCES: Final[tuple[ControllerState, ...]] = (
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
)
ALLOWED_TRANSITIONS: Final[frozenset[tuple[ControllerState, ControllerState]]] = frozenset(
    (
        *HAPPY_PATH_TRANSITIONS,
        *_REGRASP_TRANSITIONS,
        *((source, ControllerState.RETRACT) for source in _ABORT_SOURCES),
        *((source, ControllerState.TERMINAL) for source in _ERROR_SOURCES),
    )
)


class InvalidControllerTransition(ValueError):
    """Raised when a requested controller transition is not in the allowed table."""

    def __init__(self, source: ControllerState, target: ControllerState) -> None:
        self.source = source
        self.target = target
        super().__init__(f"controller transition is not allowed: {source.value} -> {target.value}")


def is_allowed_transition(source: ControllerState, target: ControllerState) -> bool:
    return (source, target) in ALLOWED_TRANSITIONS


def allowed_targets(source: ControllerState) -> frozenset[ControllerState]:
    return frozenset(target for candidate, target in ALLOWED_TRANSITIONS if candidate is source)


class TrialEventLog:
    """Assigns strictly increasing ordinals and nondecreasing simulation times."""

    def __init__(self) -> None:
        self._events: list[TrialEvent] = []
        self._next_ordinal = 0
        self._last_time_seconds: float | None = None

    @property
    def events(self) -> tuple[TrialEvent, ...]:
        return tuple(self._events)

    def record(
        self,
        event_type: EventType,
        controller_state: ControllerState,
        *,
        simulation_time_seconds: float,
        contact: ContactEvent | None = None,
        detail: str | None = None,
    ) -> TrialEvent:
        time_seconds = require_nonnegative(
            "simulation_time_seconds",
            require_finite("simulation_time_seconds", simulation_time_seconds),
        )
        if self._last_time_seconds is not None and time_seconds < self._last_time_seconds:
            msg = "simulation_time_seconds must be nondecreasing"
            raise ValueError(msg)
        event = TrialEvent(
            ordinal=self._next_ordinal,
            event_type=event_type,
            controller_state=controller_state,
            simulation_time_seconds=time_seconds,
            contact=contact,
            detail=detail,
        )
        self._events.append(event)
        self._next_ordinal += 1
        self._last_time_seconds = time_seconds
        return event


class ControllerStateMachine:
    """Reject illegal transitions and emit state start, end, and failure events."""

    def __init__(
        self,
        *,
        initial_state: ControllerState = ControllerState.RESET,
        simulation_time_seconds: float = 0.0,
    ) -> None:
        if not isinstance(initial_state, ControllerState):
            msg = "initial_state must be a ControllerState"
            raise ValueError(msg)
        self._state = initial_state
        self._log = TrialEventLog()
        self._log.record(
            EventType.STATE_START,
            initial_state,
            simulation_time_seconds=simulation_time_seconds,
        )

    @property
    def state(self) -> ControllerState:
        return self._state

    @property
    def events(self) -> tuple[TrialEvent, ...]:
        return self._log.events

    def transition(
        self,
        target: ControllerState,
        *,
        simulation_time_seconds: float,
        failed: bool = False,
        detail: str | None = None,
    ) -> tuple[TrialEvent, TrialEvent]:
        if not isinstance(target, ControllerState):
            msg = "target must be a ControllerState"
            raise ValueError(msg)
        if not isinstance(failed, bool):
            msg = "failed must be a boolean"
            raise ValueError(msg)
        if not is_allowed_transition(self._state, target):
            raise InvalidControllerTransition(self._state, target)
        end_type = EventType.STATE_FAILURE if failed else EventType.STATE_END
        ended = self._log.record(
            end_type,
            self._state,
            simulation_time_seconds=simulation_time_seconds,
            detail=detail,
        )
        started = self._log.record(
            EventType.STATE_START,
            target,
            simulation_time_seconds=simulation_time_seconds,
        )
        self._state = target
        return ended, started

    def record_contact(self, contact: ContactEvent) -> TrialEvent:
        if not isinstance(contact, ContactEvent):
            msg = "contact must be a ContactEvent"
            raise ValueError(msg)
        return self._log.record(
            EventType.CONTACT,
            self._state,
            simulation_time_seconds=contact.simulation_time_seconds,
            contact=contact,
        )
