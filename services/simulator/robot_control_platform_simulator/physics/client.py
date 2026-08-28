"""PyBullet client wrapper. Every engine call carries an explicit client id."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from types import TracebackType
from typing import Any, Final, Self

from robot_control_platform_simulator.domain.models import (
    DOMAIN_SCHEMA_VERSION,
    JSONValue,
    Pose,
    QuaternionXYZW,
    Vector3,
    canonical_dumps,
    require_finite,
    require_positive,
    sha256_hex,
)

PHYSICS_SCHEMA_VERSION: Final[str] = DOMAIN_SCHEMA_VERSION
PHYSICS_TIMESTEP_SECONDS: Final[float] = 1.0 / 240.0
PHYSICS_SOLVER_ITERATIONS: Final[int] = 150
WORLD_FRAME: Final[str] = "world"
FALLBACK_JOINT_FORCE_NEWTONS: Final[float] = 200.0

_BULLET: Any = None
_BULLET_DATA: Any = None


class SimulationError(RuntimeError):
    """Sanitized physics-adapter failure. Messages must not include local paths."""


def _pybullet() -> Any:
    global _BULLET
    if _BULLET is None:
        try:
            import pybullet as bullet
        except ImportError as exc:
            raise SimulationError("PyBullet is not installed") from exc
        _BULLET = bullet
    return _BULLET


def _pybullet_data() -> Any:
    global _BULLET_DATA
    if _BULLET_DATA is None:
        try:
            import pybullet_data as data
        except ImportError as extra:
            raise SimulationError("PyBullet data package is not installed") from extra
        _BULLET_DATA = data
    return _BULLET_DATA


def default_pybullet_data_root() -> Path:
    """Return the PyBullet data directory. Callers must not persist the path."""

    return Path(str(_pybullet_data().getDataPath()))


def connection_mode_for_gui(gui: bool) -> str:
    """Return the logical connection mode. GUI is used only when explicitly requested."""

    return "gui" if gui else "direct"


def pose_from_position_orientation(
    position: object, orientation: object, *, frame: str = WORLD_FRAME
) -> Pose:
    return Pose(
        position_meters=Vector3.from_xyz(position),
        orientation_xyzw=QuaternionXYZW.from_xyzw(orientation),
        frame=frame,
    )


@dataclass(frozen=True)
class PhysicsConfig:
    """Versioned engine settings applied after every reset. Units are SI."""

    gravity_meters_per_second_squared: Vector3
    timestep_seconds: float
    solver_iterations: int
    deterministic_overlapping_pairs: bool

    def __post_init__(self) -> None:
        if not isinstance(self.gravity_meters_per_second_squared, Vector3):
            msg = "gravity_meters_per_second_squared must be a Vector3"
            raise ValueError(msg)
        object.__setattr__(
            self,
            "timestep_seconds",
            require_positive(
                "timestep_seconds", require_finite("timestep_seconds", self.timestep_seconds)
            ),
        )
        if not isinstance(self.solver_iterations, int) or isinstance(self.solver_iterations, bool):
            msg = "solver_iterations must be a positive integer"
            raise ValueError(msg)
        if self.solver_iterations <= 0:
            msg = "solver_iterations must be a positive integer"
            raise ValueError(msg)
        if not isinstance(self.deterministic_overlapping_pairs, bool):
            msg = "deterministic_overlapping_pairs must be a boolean"
            raise ValueError(msg)

    def to_checksum_payload(self) -> dict[str, JSONValue]:
        return {
            "schema_version": PHYSICS_SCHEMA_VERSION,
            "gravity_meters_per_second_squared": (
                self.gravity_meters_per_second_squared.to_checksum_payload()
            ),
            "timestep_seconds": self.timestep_seconds,
            "solver_iterations": self.solver_iterations,
            "deterministic_overlapping_pairs": self.deterministic_overlapping_pairs,
        }

    def canonical_json(self) -> str:
        return canonical_dumps(self.to_checksum_payload())

    def sha256_hex(self) -> str:
        return sha256_hex(self.to_checksum_payload())


def default_physics_config() -> PhysicsConfig:
    return PhysicsConfig(
        gravity_meters_per_second_squared=Vector3(x=0.0, y=0.0, z=-9.81),
        timestep_seconds=PHYSICS_TIMESTEP_SECONDS,
        solver_iterations=PHYSICS_SOLVER_ITERATIONS,
        deterministic_overlapping_pairs=True,
    )


@dataclass(frozen=True)
class JointRecord:
    """Ephemeral joint description discovered from the current body id."""

    index: int
    name: str
    joint_type: int
    lower_limit: float
    upper_limit: float
    max_force_newtons: float
    rest_position: float
    is_prismatic: bool


class PhysicsClient:
    """Context-managed PyBullet connection that never uses the implicit default client."""

    def __init__(self, *, gui: bool = False) -> None:
        self._gui = gui
        self._client_id: int | None = None
        self._asset_root: Path | None = None

    @property
    def physics_client_id(self) -> int:
        if self._client_id is None:
            raise SimulationError("physics client is not connected")
        return self._client_id

    @property
    def asset_root(self) -> Path:
        if self._asset_root is None:
            raise SimulationError("physics client is not connected")
        return self._asset_root

    @property
    def gui(self) -> bool:
        return self._gui

    def connect(self) -> int:
        if self._client_id is not None:
            raise SimulationError("physics client is already connected")
        bullet = _pybullet()
        mode = bullet.GUI if self._gui else bullet.DIRECT
        try:
            client_id = int(bullet.connect(mode))
        except Exception as exc:
            raise SimulationError("physics client failed to connect") from exc
        if client_id < 0:
            raise SimulationError("physics client failed to connect")
        self._client_id = client_id
        self._asset_root = default_pybullet_data_root()
        return client_id

    def disconnect(self) -> None:
        if self._client_id is None:
            return
        bullet = _pybullet()
        client_id = self._client_id
        self._client_id = None
        self._asset_root = None
        try:
            if bool(bullet.isConnected(client_id)):
                bullet.disconnect(physicsClientId=client_id)
        except Exception as exc:
            raise SimulationError("physics client failed to disconnect") from exc

    def __enter__(self) -> Self:
        self.connect()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.disconnect()

    def is_connected(self) -> bool:
        if self._client_id is None:
            return False
        return bool(_pybullet().isConnected(self._client_id))

    def connection_mode(self) -> str:
        if not self.is_connected():
            raise SimulationError("physics client is not connected")
        info = _pybullet().getConnectionInfo(physicsClientId=self.physics_client_id)
        method = int(info["connectionMethod"]) if isinstance(info, dict) else int(info[1])
        bullet = _pybullet()
        if method == int(bullet.GUI):
            return "gui"
        if method == int(bullet.DIRECT):
            return "direct"
        raise SimulationError("physics client used an unsupported connection mode")

    def reset_simulation(self) -> None:
        _pybullet().resetSimulation(physicsClientId=self.physics_client_id)

    def configure_engine(self, config: PhysicsConfig) -> None:
        bullet = _pybullet()
        client_id = self.physics_client_id
        gravity = config.gravity_meters_per_second_squared
        bullet.setGravity(gravity.x, gravity.y, gravity.z, physicsClientId=client_id)
        bullet.setTimeStep(config.timestep_seconds, physicsClientId=client_id)
        bullet.setPhysicsEngineParameter(
            fixedTimeStep=config.timestep_seconds,
            numSolverIterations=config.solver_iterations,
            numSubSteps=0,
            deterministicOverlappingPairs=1 if config.deterministic_overlapping_pairs else 0,
            physicsClientId=client_id,
        )

    def set_asset_search_path(self) -> None:
        _pybullet().setAdditionalSearchPath(
            str(self.asset_root), physicsClientId=self.physics_client_id
        )

    def load_urdf(self, path: Path, pose: Pose, *, use_fixed_base: bool) -> int:
        body_id = _pybullet().loadURDF(
            str(path),
            list(pose.position_meters.to_checksum_payload()),
            list(pose.orientation_xyzw.to_checksum_payload()),
            useFixedBase=use_fixed_base,
            physicsClientId=self.physics_client_id,
        )
        if not isinstance(body_id, int) or body_id < 0:
            raise SimulationError("failed to load an allowlisted URDF asset")
        return body_id

    def load_sdf(self, path: Path) -> tuple[int, ...]:
        loaded = _pybullet().loadSDF(str(path), physicsClientId=self.physics_client_id)
        if not loaded:
            raise SimulationError("failed to load an allowlisted SDF asset")
        return tuple(int(body_id) for body_id in loaded)

    def reset_base_pose(self, body_id: int, pose: Pose) -> None:
        _pybullet().resetBasePositionAndOrientation(
            body_id,
            list(pose.position_meters.to_checksum_payload()),
            list(pose.orientation_xyzw.to_checksum_payload()),
            physicsClientId=self.physics_client_id,
        )

    def get_base_pose(self, body_id: int) -> Pose:
        position, orientation = _pybullet().getBasePositionAndOrientation(
            body_id, physicsClientId=self.physics_client_id
        )
        return pose_from_position_orientation(position, orientation)

    def body_count(self) -> int:
        return int(_pybullet().getNumBodies(physicsClientId=self.physics_client_id))

    def joint_count(self, body_id: int) -> int:
        return int(_pybullet().getNumJoints(body_id, physicsClientId=self.physics_client_id))

    def joint_records(self, body_id: int) -> tuple[JointRecord, ...]:
        bullet = _pybullet()
        records: list[JointRecord] = []
        for index in range(self.joint_count(body_id)):
            info = bullet.getJointInfo(body_id, index, physicsClientId=self.physics_client_id)
            name = _decode_joint_name(info[1])
            joint_type = int(info[2])
            lower = require_finite("lower_limit", info[8])
            upper = require_finite("upper_limit", info[9])
            max_force = require_finite("max_force_newtons", info[10])
            is_prismatic = joint_type == int(bullet.JOINT_PRISMATIC)
            records.append(
                JointRecord(
                    index=index,
                    name=name,
                    joint_type=joint_type,
                    lower_limit=lower,
                    upper_limit=upper,
                    max_force_newtons=max_force,
                    rest_position=_rest_position(lower, upper),
                    is_prismatic=is_prismatic,
                )
            )
        return tuple(records)

    def is_fixed_joint(self, joint_type: int) -> bool:
        return joint_type == int(_pybullet().JOINT_FIXED)

    def get_joint_position(self, body_id: int, joint_index: int) -> float:
        state = _pybullet().getJointState(
            body_id, joint_index, physicsClientId=self.physics_client_id
        )
        return require_finite("joint_position", state[0])

    def reset_joint_state(self, body_id: int, joint_index: int, position: float) -> None:
        _pybullet().resetJointState(
            body_id,
            joint_index,
            require_finite("joint_position", position),
            targetVelocity=0.0,
            physicsClientId=self.physics_client_id,
        )

    def set_position_control(
        self, body_id: int, joint_index: int, position: float, *, force_newtons: float
    ) -> None:
        _pybullet().setJointMotorControl2(
            bodyUniqueId=body_id,
            jointIndex=joint_index,
            controlMode=_pybullet().POSITION_CONTROL,
            targetPosition=require_finite("joint_position", position),
            targetVelocity=0.0,
            force=require_finite("force_newtons", force_newtons),
            physicsClientId=self.physics_client_id,
        )

    def create_static_visual_box(
        self, half_extents_meters: Vector3, pose: Pose, rgba: tuple[float, float, float, float]
    ) -> int:
        bullet = _pybullet()
        visual_id = int(
            bullet.createVisualShape(
                bullet.GEOM_BOX,
                halfExtents=list(half_extents_meters.to_checksum_payload()),
                rgbaColor=list(rgba),
                physicsClientId=self.physics_client_id,
            )
        )
        return self._create_static_body(-1, visual_id, pose)

    def create_static_open_bin(
        self,
        *,
        inner_xy_meters: tuple[float, float],
        height_meters: float,
        wall_meters: float,
        pose: Pose,
        rgba: tuple[float, float, float, float],
    ) -> int:
        inner_x, inner_y = inner_xy_meters
        hx = require_positive("inner_x_meters", require_finite("inner_x_meters", inner_x)) / 2.0
        hy = require_positive("inner_y_meters", require_finite("inner_y_meters", inner_y)) / 2.0
        height = require_positive("height_meters", require_finite("height_meters", height_meters))
        wall = require_positive("wall_meters", require_finite("wall_meters", wall_meters))
        floor_half = [hx + wall, hy + wall, wall / 2.0]
        wall_z = wall / 2.0 + height / 2.0
        halves = [
            floor_half,
            [wall / 2.0, hy + wall, height / 2.0],
            [wall / 2.0, hy + wall, height / 2.0],
            [hx + wall, wall / 2.0, height / 2.0],
            [hx + wall, wall / 2.0, height / 2.0],
        ]
        frames = [
            [0.0, 0.0, 0.0],
            [hx + wall / 2.0, 0.0, wall_z],
            [-(hx + wall / 2.0), 0.0, wall_z],
            [0.0, hy + wall / 2.0, wall_z],
            [0.0, -(hy + wall / 2.0), wall_z],
        ]
        bullet = _pybullet()
        collision_id = int(
            bullet.createCollisionShapeArray(
                shapeTypes=[bullet.GEOM_BOX] * 5,
                halfExtents=halves,
                collisionFramePositions=frames,
                physicsClientId=self.physics_client_id,
            )
        )
        visual_id = int(
            bullet.createVisualShapeArray(
                shapeTypes=[bullet.GEOM_BOX] * 5,
                halfExtents=halves,
                visualFramePositions=frames,
                rgbaColors=[list(rgba)] * 5,
                physicsClientId=self.physics_client_id,
            )
        )
        return self._create_static_body(collision_id, visual_id, pose)

    def _create_static_body(self, collision_id: int, visual_id: int, pose: Pose) -> int:
        body_id = _pybullet().createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=collision_id,
            baseVisualShapeIndex=visual_id,
            basePosition=list(pose.position_meters.to_checksum_payload()),
            baseOrientation=list(pose.orientation_xyzw.to_checksum_payload()),
            physicsClientId=self.physics_client_id,
        )
        if not isinstance(body_id, int) or body_id < 0:
            raise SimulationError("failed to create a static workcell body")
        return body_id

    def get_camera_image(
        self,
        *,
        width_px: int,
        height_px: int,
        view_matrix: tuple[float, ...],
        projection_matrix: tuple[float, ...],
        renderer: str,
    ) -> tuple[object, object, object]:
        """Capture RGBA, depth, and segmentation. Callers persist only RGB in v1."""

        if width_px <= 0 or height_px <= 0:
            raise SimulationError("camera dimensions must be positive")
        view = _as_matrix16(view_matrix, "view_matrix")
        projection = _as_matrix16(projection_matrix, "projection_matrix")
        renderer_id = renderer_constant(renderer)
        try:
            result = _pybullet().getCameraImage(
                width_px,
                height_px,
                viewMatrix=list(view),
                projectionMatrix=list(projection),
                renderer=renderer_id,
                physicsClientId=self.physics_client_id,
            )
        except Exception as exc:
            raise SimulationError("camera capture failed") from exc
        if not isinstance(result, (tuple, list)) or len(result) < 5:
            raise SimulationError("camera capture returned an incomplete buffer set")
        return result[2], result[3], result[4]


def engine_view_matrix(eye: Vector3, target: Vector3, up: Vector3) -> tuple[float, ...]:
    """Return PyBullet's look-at view matrix. Requires the engine package."""

    raw = _pybullet().computeViewMatrix(
        list(eye.to_checksum_payload()),
        list(target.to_checksum_payload()),
        list(up.to_checksum_payload()),
    )
    return _as_matrix16(raw, "view_matrix")


def engine_projection_matrix_fov(
    field_of_view_degrees: float,
    aspect_ratio: float,
    near_plane_meters: float,
    far_plane_meters: float,
) -> tuple[float, ...]:
    """Return PyBullet's FOV projection matrix. Requires the engine package."""

    raw = _pybullet().computeProjectionMatrixFOV(
        field_of_view_degrees, aspect_ratio, near_plane_meters, far_plane_meters
    )
    return _as_matrix16(raw, "projection_matrix")


def renderer_constant(renderer: str) -> int:
    if renderer != "tiny":
        raise SimulationError(f"unsupported camera renderer: {renderer}")
    return int(_pybullet().ER_TINY_RENDERER)


def _as_matrix16(raw: object, field: str) -> tuple[float, ...]:
    if not isinstance(raw, (list, tuple)) or len(raw) != 16:
        raise SimulationError(f"{field} must have length 16")
    return tuple(require_finite(f"{field}[{index}]", raw[index]) for index in range(16))


def _decode_joint_name(raw: object) -> str:
    if isinstance(raw, bytes):
        name = raw.decode("utf-8")
    elif isinstance(raw, str):
        name = raw
    else:
        raise SimulationError("joint name is not a string")
    if name.strip() == "" or name != name.strip():
        raise SimulationError("joint name is invalid")
    return name


def _rest_position(lower: float, upper: float) -> float:
    if lower <= 0.0 <= upper:
        return 0.0
    if upper <= lower:
        return 0.0
    return (lower + upper) / 2.0
