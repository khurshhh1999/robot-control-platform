"""Immutable contact and trial event values used in trial evidence."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import Final

from robot_control_platform_simulator.domain.enums import ControllerState, EventType
from robot_control_platform_simulator.domain.models import (
    DOMAIN_SCHEMA_VERSION,
    JSONValue,
    Vector3,
    canonical_dumps,
    require_finite,
    require_name,
    require_nonnegative,
    require_payload,
    require_schema_version,
    require_unit_vector,
    sha256_hex,
)

_CONTACT_PAYLOAD_KEYS: Final[frozenset[str]] = frozenset(
    {
        "schema_version",
        "body_a",
        "body_b",
        "link_a",
        "link_b",
        "position_meters",
        "normal",
        "force_newtons",
        "simulation_time_seconds",
    }
)
_TRIAL_EVENT_PAYLOAD_KEYS: Final[frozenset[str]] = frozenset(
    {
        "schema_version",
        "ordinal",
        "event_type",
        "controller_state",
        "simulation_time_seconds",
        "contact",
        "detail",
    }
)


def require_ordinal(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        msg = "ordinal must be a nonnegative integer"
        raise ValueError(msg)
    return value


def require_enum[EnumT: StrEnum](enum_type: type[EnumT], field: str, value: object) -> EnumT:
    if isinstance(value, enum_type):
        return value
    if isinstance(value, str):
        try:
            return enum_type(value)
        except ValueError as exc:
            msg = f"{field} is invalid"
            raise ValueError(msg) from exc
    msg = f"{field} is invalid"
    raise ValueError(msg)


@dataclass(frozen=True)
class ContactEvent:
    """Normalized contact sample: body/link names, point, unit normal, force, time."""

    body_a: str
    body_b: str
    link_a: str
    link_b: str
    position_meters: Vector3
    normal: Vector3
    force_newtons: float
    simulation_time_seconds: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "body_a", require_name("body_a", self.body_a))
        object.__setattr__(self, "body_b", require_name("body_b", self.body_b))
        object.__setattr__(self, "link_a", require_name("link_a", self.link_a))
        object.__setattr__(self, "link_b", require_name("link_b", self.link_b))
        if not isinstance(self.position_meters, Vector3):
            msg = "position_meters must be a Vector3"
            raise ValueError(msg)
        if not isinstance(self.normal, Vector3):
            msg = "normal must be a Vector3"
            raise ValueError(msg)
        require_unit_vector("normal", self.normal)
        object.__setattr__(
            self,
            "force_newtons",
            require_nonnegative(
                "force_newtons", require_finite("force_newtons", self.force_newtons)
            ),
        )
        object.__setattr__(
            self,
            "simulation_time_seconds",
            require_nonnegative(
                "simulation_time_seconds",
                require_finite("simulation_time_seconds", self.simulation_time_seconds),
            ),
        )

    def to_checksum_payload(self) -> dict[str, JSONValue]:
        return {
            "schema_version": DOMAIN_SCHEMA_VERSION,
            "body_a": self.body_a,
            "body_b": self.body_b,
            "link_a": self.link_a,
            "link_b": self.link_b,
            "position_meters": self.position_meters.to_checksum_payload(),
            "normal": self.normal.to_checksum_payload(),
            "force_newtons": self.force_newtons,
            "simulation_time_seconds": self.simulation_time_seconds,
        }

    @classmethod
    def from_checksum_payload(cls, payload: object) -> ContactEvent:
        data = require_payload(payload, _CONTACT_PAYLOAD_KEYS, "ContactEvent")
        require_schema_version(data["schema_version"])
        return cls(
            body_a=require_name("body_a", data["body_a"]),
            body_b=require_name("body_b", data["body_b"]),
            link_a=require_name("link_a", data["link_a"]),
            link_b=require_name("link_b", data["link_b"]),
            position_meters=Vector3.from_xyz(data["position_meters"]),
            normal=Vector3.from_xyz(data["normal"]),
            force_newtons=require_nonnegative(
                "force_newtons", require_finite("force_newtons", data["force_newtons"])
            ),
            simulation_time_seconds=require_nonnegative(
                "simulation_time_seconds",
                require_finite("simulation_time_seconds", data["simulation_time_seconds"]),
            ),
        )

    def canonical_json(self) -> str:
        return canonical_dumps(self.to_checksum_payload())

    def sha256_hex(self) -> str:
        return sha256_hex(self.to_checksum_payload())


@dataclass(frozen=True)
class TrialEvent:
    """One ordered trial evidence record: state, contact, or later action/observation."""

    ordinal: int
    event_type: EventType
    controller_state: ControllerState
    simulation_time_seconds: float
    contact: ContactEvent | None = None
    detail: str | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "ordinal", require_ordinal(self.ordinal))
        object.__setattr__(
            self, "event_type", require_enum(EventType, "event_type", self.event_type)
        )
        object.__setattr__(
            self,
            "controller_state",
            require_enum(ControllerState, "controller_state", self.controller_state),
        )
        object.__setattr__(
            self,
            "simulation_time_seconds",
            require_nonnegative(
                "simulation_time_seconds",
                require_finite("simulation_time_seconds", self.simulation_time_seconds),
            ),
        )
        if self.contact is not None and not isinstance(self.contact, ContactEvent):
            msg = "contact must be a ContactEvent or None"
            raise ValueError(msg)
        if self.event_type is EventType.CONTACT:
            if self.contact is None:
                msg = "contact events require a ContactEvent"
                raise ValueError(msg)
        elif self.contact is not None:
            msg = "contact is only allowed on contact events"
            raise ValueError(msg)
        if self.detail is not None:
            object.__setattr__(self, "detail", require_name("detail", self.detail))

    def to_checksum_payload(self) -> dict[str, JSONValue]:
        contact_payload: JSONValue
        if self.contact is None:
            contact_payload = None
        else:
            contact_payload = self.contact.to_checksum_payload()
        return {
            "schema_version": DOMAIN_SCHEMA_VERSION,
            "ordinal": self.ordinal,
            "event_type": self.event_type.value,
            "controller_state": self.controller_state.value,
            "simulation_time_seconds": self.simulation_time_seconds,
            "contact": contact_payload,
            "detail": self.detail,
        }

    @classmethod
    def from_checksum_payload(cls, payload: object) -> TrialEvent:
        data = require_payload(payload, _TRIAL_EVENT_PAYLOAD_KEYS, "TrialEvent")
        require_schema_version(data["schema_version"])
        raw_contact = data["contact"]
        contact = None if raw_contact is None else ContactEvent.from_checksum_payload(raw_contact)
        raw_detail = data["detail"]
        detail = None if raw_detail is None else require_name("detail", raw_detail)
        return cls(
            ordinal=require_ordinal(data["ordinal"]),
            event_type=require_enum(EventType, "event_type", data["event_type"]),
            controller_state=require_enum(
                ControllerState, "controller_state", data["controller_state"]
            ),
            simulation_time_seconds=require_nonnegative(
                "simulation_time_seconds",
                require_finite("simulation_time_seconds", data["simulation_time_seconds"]),
            ),
            contact=contact,
            detail=detail,
        )

    def canonical_json(self) -> str:
        return canonical_dumps(self.to_checksum_payload())

    def sha256_hex(self) -> str:
        return sha256_hex(self.to_checksum_payload())
