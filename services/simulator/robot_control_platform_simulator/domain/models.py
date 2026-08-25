"""Immutable geometric and control values with canonical checksum serialization."""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Final

from robot_control_platform_simulator.domain.enums import CanonicalUnit

DOMAIN_SCHEMA_VERSION: Final[str] = "1"
UNIT_VECTOR_TOLERANCE: Final[float] = 1e-6
_JOINT_POSITION_UNITS: Final[frozenset[CanonicalUnit]] = frozenset(
    {CanonicalUnit.METERS, CanonicalUnit.RADIANS}
)
_POSE_PAYLOAD_KEYS: Final[frozenset[str]] = frozenset(
    {"schema_version", "frame", "position_meters", "orientation_xyzw"}
)
_JOINT_PAYLOAD_KEYS: Final[frozenset[str]] = frozenset(
    {"schema_version", "name", "position", "velocity", "position_unit"}
)
_OBJECT_PAYLOAD_KEYS: Final[frozenset[str]] = frozenset(
    {
        "schema_version",
        "object_id",
        "pose",
        "mass_kilograms",
        "linear_velocity_meters_per_second",
        "angular_velocity_radians_per_second",
    }
)
_ACTION_PAYLOAD_KEYS: Final[frozenset[str]] = frozenset(
    {
        "schema_version",
        "name",
        "simulation_time_seconds",
        "target_pose",
        "joint_targets",
    }
)

type JSONScalar = str | int | float | bool | None
type JSONValue = JSONScalar | list[JSONValue] | dict[str, JSONValue]


def canonical_dumps(payload: JSONValue) -> str:
    """Serialize checksum input with sorted keys and compact separators."""

    return json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )


def sha256_hex(payload: JSONValue) -> str:
    """Return the lowercase SHA-256 hex digest of canonical JSON bytes."""

    return hashlib.sha256(canonical_dumps(payload).encode("utf-8")).hexdigest()


def require_finite(field: str, value: object) -> float:
    """Return ``value`` as a finite float, collapsing ``-0.0`` to ``0.0``."""

    if isinstance(value, bool) or not isinstance(value, (int, float)):
        msg = f"{field} must be a finite number"
        raise ValueError(msg)
    number = float(value)
    if not math.isfinite(number):
        msg = f"{field} must be a finite number"
        raise ValueError(msg)
    if number == 0.0:
        return 0.0
    return number


def require_nonnegative(field: str, value: float) -> float:
    if value < 0.0:
        msg = f"{field} must be nonnegative"
        raise ValueError(msg)
    return value


def require_positive(field: str, value: float) -> float:
    if value <= 0.0:
        msg = f"{field} must be positive"
        raise ValueError(msg)
    return value


def require_name(field: str, value: object) -> str:
    if not isinstance(value, str) or value.strip() == "" or value != value.strip():
        msg = f"{field} must be a non-empty trimmed string"
        raise ValueError(msg)
    return value


def require_sequence(value: object, length: int, field: str) -> tuple[object, ...]:
    if isinstance(value, (str, bytes, bytearray, Mapping)) or not isinstance(value, Sequence):
        msg = f"{field} must have length {length}"
        raise ValueError(msg)
    if len(value) != length:
        msg = f"{field} must have length {length}"
        raise ValueError(msg)
    return tuple(value)


def require_unit_vector(field: str, vector: Vector3) -> None:
    norm = math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)
    if not math.isfinite(norm) or abs(norm - 1.0) > UNIT_VECTOR_TOLERANCE:
        msg = f"{field} must be a unit vector within tolerance {UNIT_VECTOR_TOLERANCE}"
        raise ValueError(msg)


def require_schema_version(value: object) -> None:
    if value != DOMAIN_SCHEMA_VERSION:
        msg = "unsupported domain schema version"
        raise ValueError(msg)


def require_payload(payload: object, keys: frozenset[str], name: str) -> Mapping[str, object]:
    if not isinstance(payload, Mapping):
        msg = f"{name} checksum payload must be an object"
        raise ValueError(msg)
    if any(not isinstance(key, str) for key in payload):
        msg = f"{name} checksum payload keys must be strings"
        raise ValueError(msg)
    if set(payload.keys()) != keys:
        msg = f"{name} checksum payload keys are invalid"
        raise ValueError(msg)
    return payload


def require_canonical_unit(
    field: str, value: object, allowed: frozenset[CanonicalUnit]
) -> CanonicalUnit:
    if isinstance(value, CanonicalUnit):
        unit = value
    elif isinstance(value, str):
        try:
            unit = CanonicalUnit(value)
        except ValueError as exc:
            msg = f"{field} must be a canonical unit"
            raise ValueError(msg) from exc
    else:
        msg = f"{field} must be a canonical unit"
        raise ValueError(msg)
    if unit not in allowed:
        allowed_names = ", ".join(sorted(item.value for item in allowed))
        msg = f"{field} must be one of: {allowed_names}"
        raise ValueError(msg)
    return unit


@dataclass(frozen=True)
class Vector3:
    """Three-dimensional vector. Units are declared by the enclosing field name."""

    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "x", require_finite("x", self.x))
        object.__setattr__(self, "y", require_finite("y", self.y))
        object.__setattr__(self, "z", require_finite("z", self.z))

    @classmethod
    def from_xyz(cls, values: object) -> Vector3:
        components = require_sequence(values, 3, "Vector3")
        return cls(
            x=require_finite("x", components[0]),
            y=require_finite("y", components[1]),
            z=require_finite("z", components[2]),
        )

    def to_checksum_payload(self) -> list[JSONValue]:
        return [self.x, self.y, self.z]

    def canonical_json(self) -> str:
        return canonical_dumps(self.to_checksum_payload())

    def sha256_hex(self) -> str:
        return sha256_hex(self.to_checksum_payload())


@dataclass(frozen=True)
class QuaternionXYZW:
    """Unit quaternion stored and serialized as ``[x, y, z, w]``."""

    x: float
    y: float
    z: float
    w: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "x", require_finite("x", self.x))
        object.__setattr__(self, "y", require_finite("y", self.y))
        object.__setattr__(self, "z", require_finite("z", self.z))
        object.__setattr__(self, "w", require_finite("w", self.w))
        norm = math.sqrt(self.x**2 + self.y**2 + self.z**2 + self.w**2)
        if not math.isfinite(norm) or abs(norm - 1.0) > UNIT_VECTOR_TOLERANCE:
            msg = (
                "quaternion must be normalized to length 1.0 within "
                f"tolerance {UNIT_VECTOR_TOLERANCE}"
            )
            raise ValueError(msg)

    @classmethod
    def from_xyzw(cls, values: object) -> QuaternionXYZW:
        components = require_sequence(values, 4, "QuaternionXYZW")
        return cls(
            x=require_finite("x", components[0]),
            y=require_finite("y", components[1]),
            z=require_finite("z", components[2]),
            w=require_finite("w", components[3]),
        )

    def to_checksum_payload(self) -> list[JSONValue]:
        return [self.x, self.y, self.z, self.w]

    def canonical_json(self) -> str:
        return canonical_dumps(self.to_checksum_payload())

    def sha256_hex(self) -> str:
        return sha256_hex(self.to_checksum_payload())


@dataclass(frozen=True)
class Pose:
    """Rigid pose: position in meters and ``[x, y, z, w]`` orientation in ``frame``."""

    position_meters: Vector3
    orientation_xyzw: QuaternionXYZW
    frame: str

    def __post_init__(self) -> None:
        if not isinstance(self.position_meters, Vector3):
            msg = "position_meters must be a Vector3"
            raise ValueError(msg)
        if not isinstance(self.orientation_xyzw, QuaternionXYZW):
            msg = "orientation_xyzw must be a QuaternionXYZW"
            raise ValueError(msg)
        object.__setattr__(self, "frame", require_name("frame", self.frame))

    def to_checksum_payload(self) -> dict[str, JSONValue]:
        return {
            "schema_version": DOMAIN_SCHEMA_VERSION,
            "frame": self.frame,
            "position_meters": self.position_meters.to_checksum_payload(),
            "orientation_xyzw": self.orientation_xyzw.to_checksum_payload(),
        }

    @classmethod
    def from_checksum_payload(cls, payload: object) -> Pose:
        data = require_payload(payload, _POSE_PAYLOAD_KEYS, "Pose")
        require_schema_version(data["schema_version"])
        return cls(
            position_meters=Vector3.from_xyz(data["position_meters"]),
            orientation_xyzw=QuaternionXYZW.from_xyzw(data["orientation_xyzw"]),
            frame=require_name("frame", data["frame"]),
        )

    def canonical_json(self) -> str:
        return canonical_dumps(self.to_checksum_payload())

    def sha256_hex(self) -> str:
        return sha256_hex(self.to_checksum_payload())


@dataclass(frozen=True)
class JointState:
    """Named joint sample. ``position_unit`` is meters or radians; velocity follows."""

    name: str
    position: float
    velocity: float
    position_unit: CanonicalUnit

    def __post_init__(self) -> None:
        object.__setattr__(self, "name", require_name("name", self.name))
        object.__setattr__(self, "position", require_finite("position", self.position))
        object.__setattr__(self, "velocity", require_finite("velocity", self.velocity))
        object.__setattr__(
            self,
            "position_unit",
            require_canonical_unit("position_unit", self.position_unit, _JOINT_POSITION_UNITS),
        )

    def to_checksum_payload(self) -> dict[str, JSONValue]:
        return {
            "schema_version": DOMAIN_SCHEMA_VERSION,
            "name": self.name,
            "position": self.position,
            "velocity": self.velocity,
            "position_unit": self.position_unit.value,
        }

    @classmethod
    def from_checksum_payload(cls, payload: object) -> JointState:
        data = require_payload(payload, _JOINT_PAYLOAD_KEYS, "JointState")
        require_schema_version(data["schema_version"])
        return cls(
            name=require_name("name", data["name"]),
            position=require_finite("position", data["position"]),
            velocity=require_finite("velocity", data["velocity"]),
            position_unit=require_canonical_unit(
                "position_unit", data["position_unit"], _JOINT_POSITION_UNITS
            ),
        )

    def canonical_json(self) -> str:
        return canonical_dumps(self.to_checksum_payload())

    def sha256_hex(self) -> str:
        return sha256_hex(self.to_checksum_payload())


@dataclass(frozen=True)
class ObjectState:
    """Object snapshot: pose, mass in kilograms, and translational/rotational velocity."""

    object_id: str
    pose: Pose
    mass_kilograms: float
    linear_velocity_meters_per_second: Vector3
    angular_velocity_radians_per_second: Vector3

    def __post_init__(self) -> None:
        object.__setattr__(self, "object_id", require_name("object_id", self.object_id))
        if not isinstance(self.pose, Pose):
            msg = "pose must be a Pose"
            raise ValueError(msg)
        object.__setattr__(
            self,
            "mass_kilograms",
            require_positive(
                "mass_kilograms", require_finite("mass_kilograms", self.mass_kilograms)
            ),
        )
        if not isinstance(self.linear_velocity_meters_per_second, Vector3):
            msg = "linear_velocity_meters_per_second must be a Vector3"
            raise ValueError(msg)
        if not isinstance(self.angular_velocity_radians_per_second, Vector3):
            msg = "angular_velocity_radians_per_second must be a Vector3"
            raise ValueError(msg)

    def to_checksum_payload(self) -> dict[str, JSONValue]:
        return {
            "schema_version": DOMAIN_SCHEMA_VERSION,
            "object_id": self.object_id,
            "pose": self.pose.to_checksum_payload(),
            "mass_kilograms": self.mass_kilograms,
            "linear_velocity_meters_per_second": (
                self.linear_velocity_meters_per_second.to_checksum_payload()
            ),
            "angular_velocity_radians_per_second": (
                self.angular_velocity_radians_per_second.to_checksum_payload()
            ),
        }

    @classmethod
    def from_checksum_payload(cls, payload: object) -> ObjectState:
        data = require_payload(payload, _OBJECT_PAYLOAD_KEYS, "ObjectState")
        require_schema_version(data["schema_version"])
        return cls(
            object_id=require_name("object_id", data["object_id"]),
            pose=Pose.from_checksum_payload(data["pose"]),
            mass_kilograms=require_positive(
                "mass_kilograms", require_finite("mass_kilograms", data["mass_kilograms"])
            ),
            linear_velocity_meters_per_second=Vector3.from_xyz(
                data["linear_velocity_meters_per_second"]
            ),
            angular_velocity_radians_per_second=Vector3.from_xyz(
                data["angular_velocity_radians_per_second"]
            ),
        )

    def canonical_json(self) -> str:
        return canonical_dumps(self.to_checksum_payload())

    def sha256_hex(self) -> str:
        return sha256_hex(self.to_checksum_payload())


@dataclass(frozen=True)
class Action:
    """Commanded action at a nonnegative simulation time in seconds."""

    name: str
    simulation_time_seconds: float
    target_pose: Pose | None = None
    joint_targets: tuple[JointState, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "name", require_name("name", self.name))
        object.__setattr__(
            self,
            "simulation_time_seconds",
            require_nonnegative(
                "simulation_time_seconds",
                require_finite("simulation_time_seconds", self.simulation_time_seconds),
            ),
        )
        if self.target_pose is not None and not isinstance(self.target_pose, Pose):
            msg = "target_pose must be a Pose or None"
            raise ValueError(msg)
        if not isinstance(self.joint_targets, tuple) or any(
            not isinstance(joint, JointState) for joint in self.joint_targets
        ):
            msg = "joint_targets must be a tuple of JointState values"
            raise ValueError(msg)

    def to_checksum_payload(self) -> dict[str, JSONValue]:
        target: JSONValue
        if self.target_pose is None:
            target = None
        else:
            target = self.target_pose.to_checksum_payload()
        return {
            "schema_version": DOMAIN_SCHEMA_VERSION,
            "name": self.name,
            "simulation_time_seconds": self.simulation_time_seconds,
            "target_pose": target,
            "joint_targets": [joint.to_checksum_payload() for joint in self.joint_targets],
        }

    @classmethod
    def from_checksum_payload(cls, payload: object) -> Action:
        data = require_payload(payload, _ACTION_PAYLOAD_KEYS, "Action")
        require_schema_version(data["schema_version"])
        raw_pose = data["target_pose"]
        target_pose = None if raw_pose is None else Pose.from_checksum_payload(raw_pose)
        raw_joints = data["joint_targets"]
        if not isinstance(raw_joints, list):
            msg = "joint_targets must be an array"
            raise ValueError(msg)
        return cls(
            name=require_name("name", data["name"]),
            simulation_time_seconds=require_nonnegative(
                "simulation_time_seconds",
                require_finite("simulation_time_seconds", data["simulation_time_seconds"]),
            ),
            target_pose=target_pose,
            joint_targets=tuple(JointState.from_checksum_payload(item) for item in raw_joints),
        )

    def canonical_json(self) -> str:
        return canonical_dumps(self.to_checksum_payload())

    def sha256_hex(self) -> str:
        return sha256_hex(self.to_checksum_payload())
