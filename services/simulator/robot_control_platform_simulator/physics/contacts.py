"""Normalize PyBullet contacts and classify prohibited collisions by force and duration."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Final, Protocol

from robot_control_platform_simulator.domain.events import ContactEvent
from robot_control_platform_simulator.domain.models import (
    DOMAIN_SCHEMA_VERSION,
    JSONValue,
    Vector3,
    canonical_dumps,
    require_finite,
    require_name,
    require_nonnegative,
    sha256_hex,
)
from robot_control_platform_simulator.physics.client import (
    JointRecord,
    RawContactPoint,
    SimulationError,
)
from robot_control_platform_simulator.physics.scene import ROBOT_BODY_NAME, SCENE_BODY_NAMES

ANY_LINK: Final[str] = "*"
BASE_LINK_NAME: Final[str] = "base"
CONTACTS_SCHEMA_VERSION: Final[str] = DOMAIN_SCHEMA_VERSION
DEFAULT_COLLISION_FORCE_THRESHOLD_NEWTONS: Final[float] = 5.0
DEFAULT_COLLISION_DURATION_THRESHOLD_SECONDS: Final[float] = 0.05
_DEFAULT_PROHIBITED_BODY_NAMES: Final[tuple[str, ...]] = (
    "plane",
    "table",
    "bin_red",
    "bin_green",
    "bin_blue",
    "bin_yellow",
)


class ContactQueryClient(Protocol):
    def get_contact_points(self) -> tuple[RawContactPoint, ...]: ...

    def joint_records(self, body_id: int) -> tuple[JointRecord, ...]: ...


@dataclass(frozen=True)
class ProhibitedPair:
    """Configured body/link pair. ``*`` matches any link. Matching is undirected."""

    body_a: str
    link_a: str
    body_b: str
    link_b: str

    def __post_init__(self) -> None:
        body_a = require_name("body_a", self.body_a)
        link_a = require_name("link_a", self.link_a)
        body_b = require_name("body_b", self.body_b)
        link_b = require_name("link_b", self.link_b)
        left = (body_a, link_a)
        right = (body_b, link_b)
        if right < left:
            body_a, link_a, body_b, link_b = body_b, link_b, body_a, link_a
        object.__setattr__(self, "body_a", body_a)
        object.__setattr__(self, "link_a", link_a)
        object.__setattr__(self, "body_b", body_b)
        object.__setattr__(self, "link_b", link_b)

    def matches(self, contact: ContactEvent) -> bool:
        return _directed_match(
            self, contact.body_a, contact.link_a, contact.body_b, contact.link_b
        ) or _directed_match(self, contact.body_b, contact.link_b, contact.body_a, contact.link_a)

    def to_checksum_payload(self) -> dict[str, JSONValue]:
        return {
            "body_a": self.body_a,
            "link_a": self.link_a,
            "body_b": self.body_b,
            "link_b": self.link_b,
        }


@dataclass(frozen=True)
class CollisionConfig:
    """Force and duration thresholds required before a prohibited contact is a collision."""

    prohibited_pairs: tuple[ProhibitedPair, ...]
    force_threshold_newtons: float
    duration_threshold_seconds: float

    def __post_init__(self) -> None:
        if not isinstance(self.prohibited_pairs, tuple) or any(
            not isinstance(pair, ProhibitedPair) for pair in self.prohibited_pairs
        ):
            msg = "prohibited_pairs must be a tuple of ProhibitedPair values"
            raise ValueError(msg)
        unique = tuple(sorted(set(self.prohibited_pairs), key=_pair_sort_key))
        object.__setattr__(self, "prohibited_pairs", unique)
        object.__setattr__(
            self,
            "force_threshold_newtons",
            require_nonnegative(
                "force_threshold_newtons",
                require_finite("force_threshold_newtons", self.force_threshold_newtons),
            ),
        )
        object.__setattr__(
            self,
            "duration_threshold_seconds",
            require_nonnegative(
                "duration_threshold_seconds",
                require_finite("duration_threshold_seconds", self.duration_threshold_seconds),
            ),
        )

    def to_checksum_payload(self) -> dict[str, JSONValue]:
        return {
            "schema_version": CONTACTS_SCHEMA_VERSION,
            "duration_threshold_seconds": self.duration_threshold_seconds,
            "force_threshold_newtons": self.force_threshold_newtons,
            "prohibited_pairs": [pair.to_checksum_payload() for pair in self.prohibited_pairs],
        }

    def canonical_json(self) -> str:
        return canonical_dumps(self.to_checksum_payload())

    def sha256_hex(self) -> str:
        return sha256_hex(self.to_checksum_payload())


@dataclass(frozen=True)
class CollisionAssessment:
    """All sampled contacts plus those that currently meet the collision rule."""

    contacts: tuple[ContactEvent, ...]
    collision_contacts: tuple[ContactEvent, ...]
    collision_detected: bool
    collision_duration_seconds: float


@dataclass
class _PairDuration:
    accumulated_seconds: float
    last_time_seconds: float


class CollisionMonitor:
    """Accumulate prohibited-contact duration across simulation-time samples."""

    def __init__(self, config: CollisionConfig) -> None:
        if not isinstance(config, CollisionConfig):
            msg = "config must be a CollisionConfig"
            raise ValueError(msg)
        self._config = config
        self._trackers: dict[ProhibitedPair, _PairDuration] = {}
        self._last_time_seconds: float | None = None

    @property
    def config(self) -> CollisionConfig:
        return self._config

    def observe(
        self, contacts: Sequence[ContactEvent], *, simulation_time_seconds: float
    ) -> CollisionAssessment:
        if not isinstance(contacts, Sequence) or isinstance(contacts, (str, bytes, bytearray)):
            msg = "contacts must be a sequence of ContactEvent values"
            raise ValueError(msg)
        samples = tuple(contacts)
        if any(not isinstance(contact, ContactEvent) for contact in samples):
            msg = "contacts must be a sequence of ContactEvent values"
            raise ValueError(msg)
        time_seconds = require_nonnegative(
            "simulation_time_seconds",
            require_finite("simulation_time_seconds", simulation_time_seconds),
        )
        if self._last_time_seconds is not None and time_seconds < self._last_time_seconds:
            msg = "simulation_time_seconds must be nondecreasing"
            raise ValueError(msg)

        collision_contacts: list[ContactEvent] = []
        max_duration = 0.0
        collision_detected = False
        active: dict[ProhibitedPair, _PairDuration] = {}
        for pair in self._config.prohibited_pairs:
            matching = tuple(contact for contact in samples if pair.matches(contact))
            if not matching:
                continue
            peak_force = max(contact.force_newtons for contact in matching)
            if peak_force < self._config.force_threshold_newtons:
                continue
            previous = self._trackers.get(pair)
            if previous is None:
                duration = 0.0
            else:
                duration = previous.accumulated_seconds + (
                    time_seconds - previous.last_time_seconds
                )
            active[pair] = _PairDuration(duration, time_seconds)
            if duration > max_duration:
                max_duration = duration
            if duration >= self._config.duration_threshold_seconds:
                collision_detected = True
                collision_contacts.extend(matching)

        self._trackers = active
        self._last_time_seconds = time_seconds
        unique_collision = tuple(dict.fromkeys(collision_contacts))
        return CollisionAssessment(
            contacts=samples,
            collision_contacts=unique_collision,
            collision_detected=collision_detected,
            collision_duration_seconds=max_duration,
        )


def default_prohibited_pairs() -> tuple[ProhibitedPair, ...]:
    pairs = [
        ProhibitedPair(
            body_a=ROBOT_BODY_NAME,
            link_a=ANY_LINK,
            body_b=name,
            link_b=ANY_LINK,
        )
        for name in _DEFAULT_PROHIBITED_BODY_NAMES
    ]
    expected = {
        name for name in SCENE_BODY_NAMES if name != ROBOT_BODY_NAME and name != "pickup_region"
    }
    if set(_DEFAULT_PROHIBITED_BODY_NAMES) != expected:
        msg = "default prohibited bodies do not match the workcell"
        raise ValueError(msg)
    return tuple(pairs)


def default_collision_config() -> CollisionConfig:
    return CollisionConfig(
        prohibited_pairs=default_prohibited_pairs(),
        force_threshold_newtons=DEFAULT_COLLISION_FORCE_THRESHOLD_NEWTONS,
        duration_threshold_seconds=DEFAULT_COLLISION_DURATION_THRESHOLD_SECONDS,
    )


def sample_contacts(
    client: ContactQueryClient,
    body_id_to_name: Mapping[int, str],
    *,
    simulation_time_seconds: float,
) -> tuple[ContactEvent, ...]:
    """Normalize the current manifold to named ContactEvent values."""

    names = _validated_body_names(body_id_to_name)
    time_seconds = require_nonnegative(
        "simulation_time_seconds",
        require_finite("simulation_time_seconds", simulation_time_seconds),
    )
    link_cache: dict[tuple[int, int], str] = {}
    events: list[ContactEvent] = []
    for raw in client.get_contact_points():
        events.append(
            normalize_raw_contact(
                raw,
                names,
                simulation_time_seconds=time_seconds,
                link_name=_link_name(client, raw.body_unique_id_a, raw.link_index_a, link_cache),
                link_name_b=_link_name(client, raw.body_unique_id_b, raw.link_index_b, link_cache),
            )
        )
    return tuple(events)


def normalize_raw_contact(
    raw: RawContactPoint,
    body_id_to_name: Mapping[int, str],
    *,
    simulation_time_seconds: float,
    link_name: str,
    link_name_b: str,
) -> ContactEvent:
    if not isinstance(raw, RawContactPoint):
        raise SimulationError("contact sample is invalid")
    names = _validated_body_names(body_id_to_name)
    try:
        body_a = names[raw.body_unique_id_a]
        body_b = names[raw.body_unique_id_b]
    except KeyError as exc:
        raise SimulationError("contact referenced an unknown body") from exc
    return ContactEvent(
        body_a=body_a,
        body_b=body_b,
        link_a=require_name("link_a", link_name),
        link_b=require_name("link_b", link_name_b),
        position_meters=raw.position_world_on_a_meters,
        normal=_unit_normal(raw.contact_normal_on_b),
        force_newtons=_nonnegative_force(raw.normal_force_newtons),
        simulation_time_seconds=simulation_time_seconds,
    )


def _link_name(
    client: ContactQueryClient,
    body_id: int,
    link_index: int,
    cache: dict[tuple[int, int], str],
) -> str:
    key = (body_id, link_index)
    cached = cache.get(key)
    if cached is not None:
        return cached
    if link_index < 0:
        cache[key] = BASE_LINK_NAME
        return BASE_LINK_NAME
    for record in client.joint_records(body_id):
        if record.index == link_index:
            cache[key] = record.link_name
            return record.link_name
    raise SimulationError("contact referenced an unknown link")


def _validated_body_names(body_id_to_name: Mapping[int, str]) -> dict[int, str]:
    if not isinstance(body_id_to_name, Mapping):
        raise SimulationError("contact body map is invalid")
    names: dict[int, str] = {}
    for body_id, name in body_id_to_name.items():
        if isinstance(body_id, bool) or not isinstance(body_id, int) or body_id < 0:
            raise SimulationError("contact body map is invalid")
        names[body_id] = require_name("body_name", name)
    return names


def _unit_normal(vector: Vector3) -> Vector3:
    magnitude = math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)
    if not math.isfinite(magnitude) or magnitude == 0.0:
        raise SimulationError("contact normal is invalid")
    return Vector3(x=vector.x / magnitude, y=vector.y / magnitude, z=vector.z / magnitude)


def _nonnegative_force(value: float) -> float:
    force = require_finite("force_newtons", value)
    if force < 0.0:
        if force > -1e-9:
            return 0.0
        raise SimulationError("contact force is invalid")
    return force


def _directed_match(
    pair: ProhibitedPair, body_a: str, link_a: str, body_b: str, link_b: str
) -> bool:
    return (
        pair.body_a == body_a
        and _link_matches(pair.link_a, link_a)
        and pair.body_b == body_b
        and _link_matches(pair.link_b, link_b)
    )


def _link_matches(configured: str, actual: str) -> bool:
    return configured == ANY_LINK or configured == actual


def _pair_sort_key(pair: ProhibitedPair) -> tuple[str, str, str, str]:
    return (pair.body_a, pair.link_a, pair.body_b, pair.link_b)
