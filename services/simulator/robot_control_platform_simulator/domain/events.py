"""Immutable contact event values used in trial evidence."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final

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
