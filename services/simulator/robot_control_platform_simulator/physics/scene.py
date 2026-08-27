"""Allowlisted KUKA iiwa workcell: plane, table, arm/gripper, pickup region, bins."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Final

from robot_control_platform_simulator.domain.models import (
    DOMAIN_SCHEMA_VERSION,
    JSONValue,
    Pose,
    QuaternionXYZW,
    Vector3,
    canonical_dumps,
    sha256_hex,
)
from robot_control_platform_simulator.physics.client import (
    WORLD_FRAME,
    PhysicsClient,
    PhysicsConfig,
    SimulationError,
    connection_mode_for_gui,
    default_physics_config,
)
from robot_control_platform_simulator.physics.robot import (
    JointSpec,
    discover_controlled_joints,
    joint_states_from_specs,
    reset_controlled_joints,
)

SCENE_SCHEMA_VERSION: Final[str] = DOMAIN_SCHEMA_VERSION
RESET_SMOKE_COUNT: Final[int] = 20
ROBOT_BODY_NAME: Final[str] = "kuka_iiwa"
TABLE_TOP_Z_METERS: Final[float] = 0.625
BIN_INNER_XY_METERS: Final[tuple[float, float]] = (0.18, 0.18)
BIN_HEIGHT_METERS: Final[float] = 0.10
BIN_WALL_METERS: Final[float] = 0.012
PICKUP_HALF_EXTENTS_METERS: Final[Vector3] = Vector3(x=0.16, y=0.16, z=0.001)

ALLOWED_ASSETS: Final[dict[str, str]] = {
    "plane": "plane.urdf",
    "table": "table/table.urdf",
    "kuka_iiwa_gripper": "kuka_iiwa/kuka_with_gripper2.sdf",
}

SCENE_BODY_NAMES: Final[tuple[str, ...]] = (
    "plane",
    "table",
    ROBOT_BODY_NAME,
    "pickup_region",
    "bin_red",
    "bin_green",
    "bin_blue",
    "bin_yellow",
)

_IDENTITY = QuaternionXYZW(x=0.0, y=0.0, z=0.0, w=1.0)
_PICKUP_RGBA: Final[tuple[float, float, float, float]] = (0.75, 0.75, 0.78, 1.0)
_BIN_COLORS: Final[dict[str, tuple[float, float, float, float]]] = {
    "bin_red": (0.86, 0.16, 0.12, 1.0),
    "bin_green": (0.18, 0.62, 0.28, 1.0),
    "bin_blue": (0.16, 0.38, 0.84, 1.0),
    "bin_yellow": (0.92, 0.76, 0.14, 1.0),
}


def _pose(x: float, y: float, z: float) -> Pose:
    return Pose(
        position_meters=Vector3(x=x, y=y, z=z), orientation_xyzw=_IDENTITY, frame=WORLD_FRAME
    )


def resolve_allowlisted_asset(asset_id: str, data_root: Path) -> Path:
    """Resolve an allowlisted relative asset under the PyBullet data root.

    Rejects unknown ids, absolute fragments, and parent-directory traversal.
    Error messages use logical asset ids, never filesystem paths.
    """

    relative_name = ALLOWED_ASSETS.get(asset_id)
    if relative_name is None:
        raise SimulationError(f"asset is not allowlisted: {asset_id}")
    relative = Path(relative_name)
    if relative.is_absolute() or ".." in relative.parts:
        raise SimulationError(f"asset path is invalid: {asset_id}")
    try:
        root = data_root.resolve()
    except OSError as exc:
        raise SimulationError("physics data root is not readable") from exc
    candidate = (root / relative).resolve()
    if not candidate.is_relative_to(root):
        raise SimulationError(f"asset path is invalid: {asset_id}")
    if not candidate.is_file():
        raise SimulationError(f"allowlisted asset is missing: {asset_id}")
    return candidate


@dataclass(frozen=True)
class SceneConfig:
    """Fixed workcell layout. Positions are meters in the world frame."""

    plane_pose: Pose
    table_pose: Pose
    robot_pose: Pose
    pickup_pose: Pose
    bin_poses: tuple[tuple[str, Pose], ...]

    def __post_init__(self) -> None:
        names = tuple(name for name, _bin_pose in self.bin_poses)
        expected = tuple(name for name in SCENE_BODY_NAMES if name.startswith("bin_"))
        if names != expected:
            raise ValueError("bin poses must use the four workcell bin names in order")
        for field in (self.plane_pose, self.table_pose, self.robot_pose, self.pickup_pose):
            if field.frame != WORLD_FRAME:
                raise ValueError("workcell poses must use the world frame")
        for _, pose in self.bin_poses:
            if pose.frame != WORLD_FRAME:
                raise ValueError("workcell poses must use the world frame")

    def to_checksum_payload(self) -> dict[str, JSONValue]:
        return {
            "schema_version": SCENE_SCHEMA_VERSION,
            "assets": dict(ALLOWED_ASSETS),
            "body_names": list(SCENE_BODY_NAMES),
            "plane_pose": self.plane_pose.to_checksum_payload(),
            "table_pose": self.table_pose.to_checksum_payload(),
            "robot_pose": self.robot_pose.to_checksum_payload(),
            "pickup_pose": self.pickup_pose.to_checksum_payload(),
            "bin_poses": {name: pose.to_checksum_payload() for name, pose in self.bin_poses},
            "table_top_z_meters": TABLE_TOP_Z_METERS,
            "bin_inner_xy_meters": [BIN_INNER_XY_METERS[0], BIN_INNER_XY_METERS[1]],
            "bin_height_meters": BIN_HEIGHT_METERS,
            "bin_wall_meters": BIN_WALL_METERS,
        }

    def canonical_json(self) -> str:
        return canonical_dumps(self.to_checksum_payload())

    def sha256_hex(self) -> str:
        return sha256_hex(self.to_checksum_payload())


def default_scene_config() -> SceneConfig:
    bin_z = TABLE_TOP_Z_METERS + BIN_WALL_METERS / 2.0
    return SceneConfig(
        plane_pose=_pose(0.0, 0.0, 0.0),
        table_pose=_pose(0.85, 0.0, 0.0),
        robot_pose=_pose(-0.10, 0.0, 0.0),
        pickup_pose=_pose(0.50, 0.0, TABLE_TOP_Z_METERS + PICKUP_HALF_EXTENTS_METERS.z),
        bin_poses=(
            ("bin_red", _pose(1.35, -0.36, bin_z)),
            ("bin_green", _pose(1.35, -0.12, bin_z)),
            ("bin_blue", _pose(1.35, 0.12, bin_z)),
            ("bin_yellow", _pose(1.35, 0.36, bin_z)),
        ),
    )


@dataclass(frozen=True)
class WorkcellSnapshot:
    """Structured workcell state after reset. Contains names, never persisted body ids."""

    body_count: int
    body_names: tuple[str, ...]
    body_poses: tuple[tuple[str, Pose], ...]
    joint_names: tuple[str, ...]
    joint_positions: tuple[float, ...]
    physics_checksum: str
    scene_checksum: str

    def matches(self, other: WorkcellSnapshot) -> bool:
        return (
            self.body_count == other.body_count
            and self.body_names == other.body_names
            and self.body_poses == other.body_poses
            and self.joint_names == other.joint_names
            and self.joint_positions == other.joint_positions
            and self.physics_checksum == other.physics_checksum
            and self.scene_checksum == other.scene_checksum
        )


class WorkcellScene:
    """Load the allowlisted workcell and rebuild the ephemeral name-to-id map on reset."""

    def __init__(
        self,
        client: PhysicsClient,
        *,
        physics: PhysicsConfig | None = None,
        scene: SceneConfig | None = None,
    ) -> None:
        self._client = client
        self._physics = physics if physics is not None else default_physics_config()
        self._scene = scene if scene is not None else default_scene_config()
        self._body_ids: dict[str, int] = {}
        self._joint_specs: tuple[JointSpec, ...] = ()

    @property
    def physics_config(self) -> PhysicsConfig:
        return self._physics

    @property
    def scene_config(self) -> SceneConfig:
        return self._scene

    def body_id(self, name: str) -> int:
        try:
            return self._body_ids[name]
        except KeyError as exc:
            raise SimulationError(f"unknown workcell body: {name}") from exc

    def reset(self) -> WorkcellSnapshot:
        self._body_ids = {}
        self._joint_specs = ()
        self._client.reset_simulation()
        self._client.configure_engine(self._physics)
        self._client.set_asset_search_path()
        self._load_workcell()
        robot_id = self.body_id(ROBOT_BODY_NAME)
        self._joint_specs = discover_controlled_joints(self._client, robot_id)
        reset_controlled_joints(self._client, robot_id, self._joint_specs)
        return self.snapshot()

    def snapshot(self) -> WorkcellSnapshot:
        if set(self._body_ids) != set(SCENE_BODY_NAMES):
            raise SimulationError("workcell body map is incomplete")
        poses = tuple(
            (name, self._client.get_base_pose(self._body_ids[name])) for name in SCENE_BODY_NAMES
        )
        states = joint_states_from_specs(
            self._client, self.body_id(ROBOT_BODY_NAME), self._joint_specs
        )
        return WorkcellSnapshot(
            body_count=self._client.body_count(),
            body_names=SCENE_BODY_NAMES,
            body_poses=poses,
            joint_names=tuple(state.name for state in states),
            joint_positions=tuple(state.position for state in states),
            physics_checksum=self._physics.sha256_hex(),
            scene_checksum=self._scene.sha256_hex(),
        )

    def _load_workcell(self) -> None:
        root = self._client.asset_root
        self._register(
            "plane",
            self._client.load_urdf(
                resolve_allowlisted_asset("plane", root),
                self._scene.plane_pose,
                use_fixed_base=True,
            ),
        )
        self._register(
            "table",
            self._client.load_urdf(
                resolve_allowlisted_asset("table", root),
                self._scene.table_pose,
                use_fixed_base=True,
            ),
        )
        robot_ids = self._client.load_sdf(resolve_allowlisted_asset("kuka_iiwa_gripper", root))
        if len(robot_ids) != 1:
            raise SimulationError("kuka iiwa sdf must load as a single body")
        self._register(ROBOT_BODY_NAME, robot_ids[0])
        self._client.reset_base_pose(robot_ids[0], self._scene.robot_pose)
        self._register(
            "pickup_region",
            self._client.create_static_visual_box(
                PICKUP_HALF_EXTENTS_METERS, self._scene.pickup_pose, _PICKUP_RGBA
            ),
        )
        for name, pose in self._scene.bin_poses:
            self._register(
                name,
                self._client.create_static_open_bin(
                    inner_xy_meters=BIN_INNER_XY_METERS,
                    height_meters=BIN_HEIGHT_METERS,
                    wall_meters=BIN_WALL_METERS,
                    pose=pose,
                    rgba=_BIN_COLORS[name],
                ),
            )

    def _register(self, name: str, body_id: int) -> None:
        if name in self._body_ids:
            raise SimulationError(f"duplicate workcell body: {name}")
        self._body_ids[name] = body_id


def run_reset_smoke(*, reset_count: int = RESET_SMOKE_COUNT, gui: bool = False) -> WorkcellSnapshot:
    """Reset the workcell ``reset_count`` times and require identical structure."""

    if reset_count <= 0:
        raise SimulationError("reset smoke count must be positive")
    with PhysicsClient(gui=gui) as client:
        if client.connection_mode() != connection_mode_for_gui(gui):
            raise SimulationError("physics client used the wrong connection mode")
        scene = WorkcellScene(client)
        first = scene.reset()
        _validate_snapshot(first)
        for _ in range(reset_count - 1):
            snapshot = scene.reset()
            if not snapshot.matches(first):
                raise SimulationError("workcell reset changed body, joint, or pose structure")
            if scene.body_id(ROBOT_BODY_NAME) < 0:
                raise SimulationError("robot body id is invalid")
        if client.body_count() != first.body_count:
            raise SimulationError("workcell reset leaked bodies")
        return first


def _validate_snapshot(snapshot: WorkcellSnapshot) -> None:
    if snapshot.body_count != len(SCENE_BODY_NAMES):
        raise SimulationError("workcell body count is incorrect")
    if snapshot.body_names != SCENE_BODY_NAMES:
        raise SimulationError("workcell body names are incorrect")
    if not snapshot.joint_names:
        raise SimulationError("robot joints were not discovered")
    if len(snapshot.joint_names) != len(snapshot.joint_positions):
        raise SimulationError("robot joint snapshot is inconsistent")
    if len(snapshot.body_poses) != len(SCENE_BODY_NAMES):
        raise SimulationError("workcell pose snapshot is incomplete")
