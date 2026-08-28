"""Seeded scenario generation from explicit bounds. Policy identity is not an input."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Final

import numpy as np

from robot_control_platform_simulator.domain.models import (
    DOMAIN_SCHEMA_VERSION,
    JSONValue,
    Pose,
    QuaternionXYZW,
    Vector3,
    canonical_dumps,
    require_finite,
    require_name,
    require_payload,
    require_positive,
    require_schema_version,
    sha256_hex,
)
from robot_control_platform_simulator.physics.client import WORLD_FRAME
from robot_control_platform_simulator.physics.scene import (
    BIN_INNER_XY_METERS,
    BIN_WALL_METERS,
    PICKUP_HALF_EXTENTS_METERS,
    SCENE_BODY_NAMES,
    TABLE_TOP_Z_METERS,
    SceneConfig,
    default_scene_config,
)

SCENARIO_GENERATOR_VERSION: Final[str] = "1"
# Canonical JSON rounds generated floats so checksums match across libm implementations.
CANONICAL_FLOAT_DIGITS: Final[int] = 12
OBJECT_SHAPE_BOX: Final[str] = "box"
OBJECT_SHAPE_CYLINDER: Final[str] = "cylinder"
ALLOWED_OBJECT_SHAPES: Final[frozenset[str]] = frozenset({OBJECT_SHAPE_BOX, OBJECT_SHAPE_CYLINDER})
DEFAULT_PLACEMENT_RETRY_LIMIT: Final[int] = 64
WORKCELL_BIN_NAMES: Final[tuple[str, ...]] = tuple(
    name for name in SCENE_BODY_NAMES if name.startswith("bin_")
)

_OBJECT_TYPE_PAYLOAD_KEYS: Final[frozenset[str]] = frozenset(
    {"schema_version", "name", "category", "shape", "half_extents_meters"}
)
_PERTURBATION_PAYLOAD_KEYS: Final[frozenset[str]] = frozenset(
    {
        "schema_version",
        "pose_translation_noise_meters",
        "yaw_noise_radians",
        "camera_intensity_noise",
        "lighting_scale_min",
        "lighting_scale_max",
    }
)
_SCENARIO_PAYLOAD_KEYS: Final[frozenset[str]] = frozenset(
    {
        "schema_version",
        "generator_version",
        "seed",
        "object_type",
        "initial_pose",
        "mass_kilograms",
        "lateral_friction",
        "target_bin",
        "target_bin_pose",
        "allowed_perturbations",
        "generator_config_checksum",
        "scene_checksum",
    }
)


class ScenarioGenerationError(RuntimeError):
    """Seeded scenario generation failed. Messages must not include local paths."""


def _require_seed(seed: object) -> int:
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise ValueError("seed must be a nonnegative integer")
    if seed < 0:
        raise ValueError("seed must be a nonnegative integer")
    return seed


def _require_positive_int(field: str, value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{field} must be a positive integer")
    return value


def _require_sha256_hex(field: str, value: object) -> str:
    if not isinstance(value, str) or len(value) != 64:
        raise ValueError(f"{field} must be a lowercase SHA-256 hex digest")
    if value != value.lower() or any(char not in "0123456789abcdef" for char in value):
        raise ValueError(f"{field} must be a lowercase SHA-256 hex digest")
    return value


def _require_closed_range(
    name: str, low: object, high: object, *, min_exclusive: float | None = None
) -> tuple[float, float]:
    low_value = require_finite(f"{name}_min", low)
    high_value = require_finite(f"{name}_max", high)
    if high_value < low_value:
        raise ValueError(f"{name} max must be greater than or equal to min")
    if min_exclusive is not None and (low_value <= min_exclusive or high_value <= min_exclusive):
        raise ValueError(f"{name} bounds must be greater than {min_exclusive}")
    return low_value, high_value


def canonical_float(field: str, value: object) -> float:
    """Round a finite float to ``CANONICAL_FLOAT_DIGITS`` decimal places."""

    number = require_finite(field, value)
    return require_finite(field, round(number, CANONICAL_FLOAT_DIGITS))


def canonical_vector3(vector: Vector3) -> Vector3:
    return Vector3(
        x=canonical_float("x", vector.x),
        y=canonical_float("y", vector.y),
        z=canonical_float("z", vector.z),
    )


def canonical_quaternion(quaternion: QuaternionXYZW) -> QuaternionXYZW:
    return QuaternionXYZW(
        x=canonical_float("qx", quaternion.x),
        y=canonical_float("qy", quaternion.y),
        z=canonical_float("qz", quaternion.z),
        w=canonical_float("qw", quaternion.w),
    )


def canonical_pose(pose: Pose) -> Pose:
    return Pose(
        position_meters=canonical_vector3(pose.position_meters),
        orientation_xyzw=canonical_quaternion(pose.orientation_xyzw),
        frame=pose.frame,
    )


def quaternion_from_yaw_radians(yaw_radians: float) -> QuaternionXYZW:
    """Return a world-frame yaw quaternion ordered ``[x, y, z, w]``."""

    half = require_finite("yaw_radians", yaw_radians) / 2.0
    return QuaternionXYZW(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def scenario_rng(seed: int) -> np.random.Generator:
    """Return a local PCG64 generator. Callers must not use a global NumPy RNG."""

    return np.random.Generator(np.random.PCG64(_require_seed(seed)))


@dataclass(frozen=True)
class ObjectTypeSpec:
    """Allowlisted parcel primitive. ``half_extents_meters`` is the axis-aligned size."""

    name: str
    category: str
    shape: str
    half_extents_meters: Vector3

    def __post_init__(self) -> None:
        object.__setattr__(self, "name", require_name("name", self.name))
        object.__setattr__(self, "category", require_name("category", self.category))
        if self.shape not in ALLOWED_OBJECT_SHAPES:
            allowed = ", ".join(sorted(ALLOWED_OBJECT_SHAPES))
            raise ValueError(f"shape must be one of: {allowed}")
        if not isinstance(self.half_extents_meters, Vector3):
            raise ValueError("half_extents_meters must be a Vector3")
        require_positive("half_extents_meters.x", self.half_extents_meters.x)
        require_positive("half_extents_meters.y", self.half_extents_meters.y)
        require_positive("half_extents_meters.z", self.half_extents_meters.z)
        if self.shape == OBJECT_SHAPE_CYLINDER and (
            self.half_extents_meters.x != self.half_extents_meters.y
        ):
            raise ValueError("cylinder radius extents must be equal")

    @property
    def footprint_radius_meters(self) -> float:
        if self.shape == OBJECT_SHAPE_CYLINDER:
            return self.half_extents_meters.x
        return math.hypot(self.half_extents_meters.x, self.half_extents_meters.y)

    def to_checksum_payload(self) -> dict[str, JSONValue]:
        return {
            "schema_version": DOMAIN_SCHEMA_VERSION,
            "name": self.name,
            "category": self.category,
            "shape": self.shape,
            "half_extents_meters": self.half_extents_meters.to_checksum_payload(),
        }

    @classmethod
    def from_checksum_payload(cls, payload: object) -> ObjectTypeSpec:
        data = require_payload(payload, _OBJECT_TYPE_PAYLOAD_KEYS, "ObjectTypeSpec")
        require_schema_version(data["schema_version"])
        return cls(
            name=require_name("name", data["name"]),
            category=require_name("category", data["category"]),
            shape=require_name("shape", data["shape"]),
            half_extents_meters=Vector3.from_xyz(data["half_extents_meters"]),
        )


@dataclass(frozen=True)
class AllowedPerturbations:
    """Recorded bounds for optional later noise. T07 does not sample these onto pose."""

    pose_translation_noise_meters: float
    yaw_noise_radians: float
    camera_intensity_noise: float
    lighting_scale_min: float
    lighting_scale_max: float

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "pose_translation_noise_meters",
            _require_nonnegative_finite(
                "pose_translation_noise_meters", self.pose_translation_noise_meters
            ),
        )
        object.__setattr__(
            self,
            "yaw_noise_radians",
            _require_nonnegative_finite("yaw_noise_radians", self.yaw_noise_radians),
        )
        object.__setattr__(
            self,
            "camera_intensity_noise",
            _require_nonnegative_finite("camera_intensity_noise", self.camera_intensity_noise),
        )
        low, high = _require_closed_range(
            "lighting_scale",
            self.lighting_scale_min,
            self.lighting_scale_max,
            min_exclusive=0.0,
        )
        object.__setattr__(self, "lighting_scale_min", low)
        object.__setattr__(self, "lighting_scale_max", high)

    def to_checksum_payload(self) -> dict[str, JSONValue]:
        return {
            "schema_version": DOMAIN_SCHEMA_VERSION,
            "pose_translation_noise_meters": self.pose_translation_noise_meters,
            "yaw_noise_radians": self.yaw_noise_radians,
            "camera_intensity_noise": self.camera_intensity_noise,
            "lighting_scale_min": self.lighting_scale_min,
            "lighting_scale_max": self.lighting_scale_max,
        }

    @classmethod
    def from_checksum_payload(cls, payload: object) -> AllowedPerturbations:
        data = require_payload(payload, _PERTURBATION_PAYLOAD_KEYS, "AllowedPerturbations")
        require_schema_version(data["schema_version"])
        return cls(
            pose_translation_noise_meters=require_finite(
                "pose_translation_noise_meters", data["pose_translation_noise_meters"]
            ),
            yaw_noise_radians=require_finite("yaw_noise_radians", data["yaw_noise_radians"]),
            camera_intensity_noise=require_finite(
                "camera_intensity_noise", data["camera_intensity_noise"]
            ),
            lighting_scale_min=require_finite("lighting_scale_min", data["lighting_scale_min"]),
            lighting_scale_max=require_finite("lighting_scale_max", data["lighting_scale_max"]),
        )


def _require_nonnegative_finite(field: str, value: object) -> float:
    number = require_finite(field, value)
    if number < 0.0:
        raise ValueError(f"{field} must be nonnegative")
    return number


def default_allowed_perturbations() -> AllowedPerturbations:
    return AllowedPerturbations(
        pose_translation_noise_meters=0.0,
        yaw_noise_radians=0.0,
        camera_intensity_noise=0.0,
        lighting_scale_min=1.0,
        lighting_scale_max=1.0,
    )


def default_object_types() -> tuple[ObjectTypeSpec, ...]:
    """Small public-neutral parcel primitives that fit the default pickup region."""

    return (
        ObjectTypeSpec(
            name="cube",
            category="cube",
            shape=OBJECT_SHAPE_BOX,
            half_extents_meters=Vector3(x=0.025, y=0.025, z=0.025),
        ),
        ObjectTypeSpec(
            name="box",
            category="box",
            shape=OBJECT_SHAPE_BOX,
            half_extents_meters=Vector3(x=0.040, y=0.025, z=0.020),
        ),
        ObjectTypeSpec(
            name="cylinder",
            category="cylinder",
            shape=OBJECT_SHAPE_CYLINDER,
            half_extents_meters=Vector3(x=0.022, y=0.022, z=0.025),
        ),
        ObjectTypeSpec(
            name="tall_box",
            category="tall_box",
            shape=OBJECT_SHAPE_BOX,
            half_extents_meters=Vector3(x=0.020, y=0.020, z=0.040),
        ),
    )


@dataclass(frozen=True)
class ScenarioGeneratorConfig:
    """Explicit generation bounds. Policy identity is not a field and must not be added."""

    object_types: tuple[ObjectTypeSpec, ...]
    target_bins: tuple[str, ...]
    mass_kilograms_min: float
    mass_kilograms_max: float
    lateral_friction_min: float
    lateral_friction_max: float
    yaw_radians_min: float
    yaw_radians_max: float
    pickup_center_meters: Vector3
    pickup_half_extents_meters: Vector3
    table_top_z_meters: float
    placement_retry_limit: int
    allowed_perturbations: AllowedPerturbations

    def __post_init__(self) -> None:
        if not isinstance(self.object_types, tuple) or not self.object_types:
            raise ValueError("object_types must be a non-empty tuple")
        names: list[str] = []
        for spec in self.object_types:
            if not isinstance(spec, ObjectTypeSpec):
                raise ValueError("object_types must contain ObjectTypeSpec values")
            names.append(spec.name)
        if len(set(names)) != len(names):
            raise ValueError("object_types names must be unique")
        if not isinstance(self.target_bins, tuple) or not self.target_bins:
            raise ValueError("target_bins must be a non-empty tuple")
        bins: list[str] = []
        for name in self.target_bins:
            trimmed = require_name("target_bins", name)
            if trimmed not in WORKCELL_BIN_NAMES:
                raise ValueError(f"target_bins must use workcell bin names: {trimmed}")
            bins.append(trimmed)
        if len(set(bins)) != len(bins):
            raise ValueError("target_bins names must be unique")
        object.__setattr__(self, "target_bins", tuple(bins))
        mass_min, mass_max = _require_closed_range(
            "mass_kilograms",
            self.mass_kilograms_min,
            self.mass_kilograms_max,
            min_exclusive=0.0,
        )
        object.__setattr__(self, "mass_kilograms_min", mass_min)
        object.__setattr__(self, "mass_kilograms_max", mass_max)
        friction_min, friction_max = _require_closed_range(
            "lateral_friction",
            self.lateral_friction_min,
            self.lateral_friction_max,
            min_exclusive=0.0,
        )
        object.__setattr__(self, "lateral_friction_min", friction_min)
        object.__setattr__(self, "lateral_friction_max", friction_max)
        yaw_min, yaw_max = _require_closed_range(
            "yaw_radians", self.yaw_radians_min, self.yaw_radians_max
        )
        object.__setattr__(self, "yaw_radians_min", yaw_min)
        object.__setattr__(self, "yaw_radians_max", yaw_max)
        if not isinstance(self.pickup_center_meters, Vector3):
            raise ValueError("pickup_center_meters must be a Vector3")
        if not isinstance(self.pickup_half_extents_meters, Vector3):
            raise ValueError("pickup_half_extents_meters must be a Vector3")
        require_positive("pickup_half_extents_meters.x", self.pickup_half_extents_meters.x)
        require_positive("pickup_half_extents_meters.y", self.pickup_half_extents_meters.y)
        object.__setattr__(
            self,
            "table_top_z_meters",
            require_finite("table_top_z_meters", self.table_top_z_meters),
        )
        object.__setattr__(
            self,
            "placement_retry_limit",
            _require_positive_int("placement_retry_limit", self.placement_retry_limit),
        )
        if not isinstance(self.allowed_perturbations, AllowedPerturbations):
            raise ValueError("allowed_perturbations must be AllowedPerturbations")

    def to_checksum_payload(self) -> dict[str, JSONValue]:
        return {
            "schema_version": DOMAIN_SCHEMA_VERSION,
            "generator_version": SCENARIO_GENERATOR_VERSION,
            "object_types": [spec.to_checksum_payload() for spec in self.object_types],
            "target_bins": list(self.target_bins),
            "mass_kilograms_min": self.mass_kilograms_min,
            "mass_kilograms_max": self.mass_kilograms_max,
            "lateral_friction_min": self.lateral_friction_min,
            "lateral_friction_max": self.lateral_friction_max,
            "yaw_radians_min": self.yaw_radians_min,
            "yaw_radians_max": self.yaw_radians_max,
            "pickup_center_meters": self.pickup_center_meters.to_checksum_payload(),
            "pickup_half_extents_meters": self.pickup_half_extents_meters.to_checksum_payload(),
            "table_top_z_meters": self.table_top_z_meters,
            "placement_retry_limit": self.placement_retry_limit,
            "allowed_perturbations": self.allowed_perturbations.to_checksum_payload(),
        }

    def canonical_json(self) -> str:
        return canonical_dumps(self.to_checksum_payload())

    def sha256_hex(self) -> str:
        return sha256_hex(self.to_checksum_payload())


def default_scenario_generator_config(
    scene: SceneConfig | None = None,
) -> ScenarioGeneratorConfig:
    workcell = scene if scene is not None else default_scene_config()
    return ScenarioGeneratorConfig(
        object_types=default_object_types(),
        target_bins=WORKCELL_BIN_NAMES,
        mass_kilograms_min=0.04,
        mass_kilograms_max=0.20,
        lateral_friction_min=0.30,
        lateral_friction_max=0.90,
        yaw_radians_min=-math.pi,
        yaw_radians_max=math.pi,
        pickup_center_meters=workcell.pickup_pose.position_meters,
        pickup_half_extents_meters=PICKUP_HALF_EXTENTS_METERS,
        table_top_z_meters=TABLE_TOP_Z_METERS,
        placement_retry_limit=DEFAULT_PLACEMENT_RETRY_LIMIT,
        allowed_perturbations=default_allowed_perturbations(),
    )


@dataclass(frozen=True)
class Scenario:
    """Seed-derived scenario content. Persistence ids are assigned when a set is stored."""

    seed: int
    generator_version: str
    object_type: ObjectTypeSpec
    initial_pose: Pose
    mass_kilograms: float
    lateral_friction: float
    target_bin: str
    target_bin_pose: Pose
    allowed_perturbations: AllowedPerturbations
    generator_config_checksum: str
    scene_checksum: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "seed", _require_seed(self.seed))
        object.__setattr__(
            self, "generator_version", require_name("generator_version", self.generator_version)
        )
        if not isinstance(self.object_type, ObjectTypeSpec):
            raise ValueError("object_type must be an ObjectTypeSpec")
        if not isinstance(self.initial_pose, Pose):
            raise ValueError("initial_pose must be a Pose")
        if self.initial_pose.frame != WORLD_FRAME:
            raise ValueError("initial_pose must use the world frame")
        object.__setattr__(self, "initial_pose", canonical_pose(self.initial_pose))
        object.__setattr__(
            self,
            "mass_kilograms",
            require_positive(
                "mass_kilograms", canonical_float("mass_kilograms", self.mass_kilograms)
            ),
        )
        object.__setattr__(
            self,
            "lateral_friction",
            require_positive(
                "lateral_friction", canonical_float("lateral_friction", self.lateral_friction)
            ),
        )
        object.__setattr__(self, "target_bin", require_name("target_bin", self.target_bin))
        if self.target_bin not in WORKCELL_BIN_NAMES:
            raise ValueError("target_bin must be a workcell bin name")
        if not isinstance(self.target_bin_pose, Pose):
            raise ValueError("target_bin_pose must be a Pose")
        if self.target_bin_pose.frame != WORLD_FRAME:
            raise ValueError("target_bin_pose must use the world frame")
        if not isinstance(self.allowed_perturbations, AllowedPerturbations):
            raise ValueError("allowed_perturbations must be AllowedPerturbations")
        object.__setattr__(
            self,
            "generator_config_checksum",
            _require_sha256_hex("generator_config_checksum", self.generator_config_checksum),
        )
        object.__setattr__(
            self, "scene_checksum", _require_sha256_hex("scene_checksum", self.scene_checksum)
        )

    def to_checksum_payload(self) -> dict[str, JSONValue]:
        return {
            "schema_version": DOMAIN_SCHEMA_VERSION,
            "generator_version": self.generator_version,
            "seed": self.seed,
            "object_type": self.object_type.to_checksum_payload(),
            "initial_pose": self.initial_pose.to_checksum_payload(),
            "mass_kilograms": self.mass_kilograms,
            "lateral_friction": self.lateral_friction,
            "target_bin": self.target_bin,
            "target_bin_pose": self.target_bin_pose.to_checksum_payload(),
            "allowed_perturbations": self.allowed_perturbations.to_checksum_payload(),
            "generator_config_checksum": self.generator_config_checksum,
            "scene_checksum": self.scene_checksum,
        }

    @classmethod
    def from_checksum_payload(cls, payload: object) -> Scenario:
        data = require_payload(payload, _SCENARIO_PAYLOAD_KEYS, "Scenario")
        require_schema_version(data["schema_version"])
        return cls(
            seed=_require_seed(data["seed"]),
            generator_version=require_name("generator_version", data["generator_version"]),
            object_type=ObjectTypeSpec.from_checksum_payload(data["object_type"]),
            initial_pose=Pose.from_checksum_payload(data["initial_pose"]),
            mass_kilograms=require_finite("mass_kilograms", data["mass_kilograms"]),
            lateral_friction=require_finite("lateral_friction", data["lateral_friction"]),
            target_bin=require_name("target_bin", data["target_bin"]),
            target_bin_pose=Pose.from_checksum_payload(data["target_bin_pose"]),
            allowed_perturbations=AllowedPerturbations.from_checksum_payload(
                data["allowed_perturbations"]
            ),
            generator_config_checksum=_require_sha256_hex(
                "generator_config_checksum", data["generator_config_checksum"]
            ),
            scene_checksum=_require_sha256_hex("scene_checksum", data["scene_checksum"]),
        )

    def canonical_json(self) -> str:
        return canonical_dumps(self.to_checksum_payload())

    def sha256_hex(self) -> str:
        return sha256_hex(self.to_checksum_payload())


def _pickup_aabb(config: ScenarioGeneratorConfig) -> tuple[float, float, float, float]:
    center = config.pickup_center_meters
    half = config.pickup_half_extents_meters
    return (
        center.x - half.x,
        center.x + half.x,
        center.y - half.y,
        center.y + half.y,
    )


def _point_in_aabb(x: float, y: float, xmin: float, xmax: float, ymin: float, ymax: float) -> bool:
    return xmin <= x <= xmax and ymin <= y <= ymax


def _obb_corners(
    x: float, y: float, half_x: float, half_y: float, yaw_radians: float
) -> tuple[tuple[float, float], ...]:
    cos_yaw = math.cos(yaw_radians)
    sin_yaw = math.sin(yaw_radians)
    offsets = ((-half_x, -half_y), (half_x, -half_y), (half_x, half_y), (-half_x, half_y))
    return tuple(
        (x + ox * cos_yaw - oy * sin_yaw, y + ox * sin_yaw + oy * cos_yaw) for ox, oy in offsets
    )


def _aabb_from_points(
    points: Sequence[tuple[float, float]],
) -> tuple[float, float, float, float]:
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return (min(xs), max(xs), min(ys), max(ys))


def _aabb_intersects(
    left: tuple[float, float, float, float], right: tuple[float, float, float, float]
) -> bool:
    return not (
        left[1] < right[0] or left[0] > right[1] or left[3] < right[2] or left[2] > right[3]
    )


def _circle_in_aabb(
    x: float, y: float, radius: float, xmin: float, xmax: float, ymin: float, ymax: float
) -> bool:
    return x - radius >= xmin and x + radius <= xmax and y - radius >= ymin and y + radius <= ymax


def _circle_intersects_aabb(
    x: float, y: float, radius: float, box: tuple[float, float, float, float]
) -> bool:
    nearest_x = min(max(x, box[0]), box[1])
    nearest_y = min(max(y, box[2]), box[3])
    delta_x = x - nearest_x
    delta_y = y - nearest_y
    return delta_x * delta_x + delta_y * delta_y <= radius * radius


def _bin_outer_aabb(pose: Pose) -> tuple[float, float, float, float]:
    half_x = BIN_INNER_XY_METERS[0] / 2.0 + BIN_WALL_METERS
    half_y = BIN_INNER_XY_METERS[1] / 2.0 + BIN_WALL_METERS
    x = pose.position_meters.x
    y = pose.position_meters.y
    return (x - half_x, x + half_x, y - half_y, y + half_y)


def _object_in_pickup(
    spec: ObjectTypeSpec,
    x: float,
    y: float,
    yaw_radians: float,
    pickup: tuple[float, float, float, float],
) -> bool:
    xmin, xmax, ymin, ymax = pickup
    if spec.shape == OBJECT_SHAPE_CYLINDER:
        return _circle_in_aabb(x, y, spec.half_extents_meters.x, xmin, xmax, ymin, ymax)
    corners = _obb_corners(
        x, y, spec.half_extents_meters.x, spec.half_extents_meters.y, yaw_radians
    )
    return all(_point_in_aabb(cx, cy, xmin, xmax, ymin, ymax) for cx, cy in corners)


def _object_intersects_bins(
    spec: ObjectTypeSpec,
    x: float,
    y: float,
    yaw_radians: float,
    bin_poses: Mapping[str, Pose],
) -> bool:
    if spec.shape == OBJECT_SHAPE_CYLINDER:
        radius = spec.half_extents_meters.x
        return any(
            _circle_intersects_aabb(x, y, radius, _bin_outer_aabb(pose))
            for pose in bin_poses.values()
        )
    corners = _obb_corners(
        x, y, spec.half_extents_meters.x, spec.half_extents_meters.y, yaw_radians
    )
    object_box = _aabb_from_points(corners)
    return any(_aabb_intersects(object_box, _bin_outer_aabb(pose)) for pose in bin_poses.values())


def _placement_is_valid(
    spec: ObjectTypeSpec,
    x: float,
    y: float,
    yaw_radians: float,
    config: ScenarioGeneratorConfig,
    bin_poses: Mapping[str, Pose],
) -> bool:
    pickup = _pickup_aabb(config)
    if not _object_in_pickup(spec, x, y, yaw_radians, pickup):
        return False
    return not _object_intersects_bins(spec, x, y, yaw_radians, bin_poses)


def generate_scenario(
    seed: int,
    *,
    config: ScenarioGeneratorConfig | None = None,
    scene: SceneConfig | None = None,
) -> Scenario:
    """Draw one scenario from ``seed``. Policy identity is not accepted as input."""

    resolved_seed = _require_seed(seed)
    workcell = scene if scene is not None else default_scene_config()
    bounds = config if config is not None else default_scenario_generator_config(workcell)
    bin_poses = {name: pose for name, pose in workcell.bin_poses}
    missing = [name for name in bounds.target_bins if name not in bin_poses]
    if missing:
        raise ScenarioGenerationError(f"target bin is missing from the scene: {missing[0]}")

    rng = scenario_rng(resolved_seed)
    spec = bounds.object_types[int(rng.integers(0, len(bounds.object_types)))]
    target_bin = bounds.target_bins[int(rng.integers(0, len(bounds.target_bins)))]
    mass = require_finite(
        "mass_kilograms",
        float(rng.uniform(bounds.mass_kilograms_min, bounds.mass_kilograms_max)),
    )
    friction = require_finite(
        "lateral_friction",
        float(rng.uniform(bounds.lateral_friction_min, bounds.lateral_friction_max)),
    )
    pickup = _pickup_aabb(bounds)
    pose: Pose | None = None
    for _ in range(bounds.placement_retry_limit):
        yaw = require_finite(
            "yaw_radians", float(rng.uniform(bounds.yaw_radians_min, bounds.yaw_radians_max))
        )
        x = require_finite("position_x", float(rng.uniform(pickup[0], pickup[1])))
        y = require_finite("position_y", float(rng.uniform(pickup[2], pickup[3])))
        if _placement_is_valid(spec, x, y, yaw, bounds, bin_poses):
            pose = Pose(
                position_meters=Vector3(
                    x=x, y=y, z=bounds.table_top_z_meters + spec.half_extents_meters.z
                ),
                orientation_xyzw=quaternion_from_yaw_radians(yaw),
                frame=WORLD_FRAME,
            )
            break
    if pose is None:
        raise ScenarioGenerationError(f"placement retry exhausted for seed {resolved_seed}")

    return Scenario(
        seed=resolved_seed,
        generator_version=SCENARIO_GENERATOR_VERSION,
        object_type=spec,
        initial_pose=pose,
        mass_kilograms=mass,
        lateral_friction=friction,
        target_bin=target_bin,
        target_bin_pose=bin_poses[target_bin],
        allowed_perturbations=bounds.allowed_perturbations,
        generator_config_checksum=bounds.sha256_hex(),
        scene_checksum=workcell.sha256_hex(),
    )


def generate_scenarios(
    seeds: Sequence[int],
    *,
    config: ScenarioGeneratorConfig | None = None,
    scene: SceneConfig | None = None,
) -> tuple[Scenario, ...]:
    """Generate independent scenarios, each from its own ``Generator(PCG64(seed))``."""

    return tuple(generate_scenario(seed, config=config, scene=scene) for seed in seeds)
