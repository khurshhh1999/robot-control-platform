"""PyBullet client wrapper. Every engine call carries an explicit client id."""

from __future__ import annotations

from collections.abc import Sequence
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
    require_nonnegative,
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
class RawContactPoint:
    """Ephemeral engine contact sample. Body ids are not persisted."""

    body_unique_id_a: int
    body_unique_id_b: int
    link_index_a: int
    link_index_b: int
    position_world_on_a_meters: Vector3
    contact_normal_on_b: Vector3
    normal_force_newtons: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "body_unique_id_a", _require_body_id(self.body_unique_id_a))
        object.__setattr__(self, "body_unique_id_b", _require_body_id(self.body_unique_id_b))
        object.__setattr__(self, "link_index_a", _require_link_index(self.link_index_a))
        object.__setattr__(self, "link_index_b", _require_link_index(self.link_index_b))
        if not isinstance(self.position_world_on_a_meters, Vector3):
            raise SimulationError("contact position is invalid")
        if not isinstance(self.contact_normal_on_b, Vector3):
            raise SimulationError("contact normal is invalid")
        object.__setattr__(
            self,
            "normal_force_newtons",
            require_finite("normal_force_newtons", self.normal_force_newtons),
        )


@dataclass(frozen=True)
class JointRecord:
    """Ephemeral joint description discovered from the current body id."""

    index: int
    name: str
    link_name: str
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
        self._timestep_seconds = PHYSICS_TIMESTEP_SECONDS
        self._step_count = 0

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
        self._step_count = 0
        self._timestep_seconds = PHYSICS_TIMESTEP_SECONDS
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
        self._step_count = 0

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
        self._timestep_seconds = config.timestep_seconds

    def step_simulation(self, steps: int = 1) -> None:
        if isinstance(steps, bool) or not isinstance(steps, int) or steps <= 0:
            raise SimulationError("physics step count must be a positive integer")
        client_id = self.physics_client_id
        bullet = _pybullet()
        for _ in range(steps):
            bullet.stepSimulation(physicsClientId=client_id)
            self._step_count += 1

    def simulation_time_seconds(self) -> float:
        return self._step_count * self._timestep_seconds

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
            link_name = _decode_link_name(info[12], index)
            joint_type = int(info[2])
            lower = require_finite("lower_limit", info[8])
            upper = require_finite("upper_limit", info[9])
            max_force = require_finite("max_force_newtons", info[10])
            is_prismatic = joint_type == int(bullet.JOINT_PRISMATIC)
            records.append(
                JointRecord(
                    index=index,
                    name=name,
                    link_name=link_name,
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

    def get_joint_state(self, body_id: int, joint_index: int) -> tuple[float, float]:
        state = _pybullet().getJointState(
            body_id, joint_index, physicsClientId=self.physics_client_id
        )
        position = require_finite("joint_position", state[0])
        velocity = require_finite("joint_velocity", state[1])
        return position, velocity

    def get_joint_position(self, body_id: int, joint_index: int) -> float:
        position, _velocity = self.get_joint_state(body_id, joint_index)
        return position

    def reset_joint_state(self, body_id: int, joint_index: int, position: float) -> None:
        _pybullet().resetJointState(
            body_id,
            joint_index,
            require_finite("joint_position", position),
            targetVelocity=0.0,
            physicsClientId=self.physics_client_id,
        )

    def set_position_control(
        self,
        body_id: int,
        joint_index: int,
        position: float,
        *,
        force_newtons: float,
        max_velocity: float | None = None,
        position_gain: float | None = None,
    ) -> None:
        kwargs: dict[str, float] = {}
        if max_velocity is not None:
            kwargs["maxVelocity"] = require_positive(
                "max_velocity", require_finite("max_velocity", max_velocity)
            )
        if position_gain is not None:
            kwargs["positionGain"] = require_positive(
                "position_gain", require_finite("position_gain", position_gain)
            )
        _pybullet().setJointMotorControl2(
            bodyUniqueId=body_id,
            jointIndex=joint_index,
            controlMode=_pybullet().POSITION_CONTROL,
            targetPosition=require_finite("joint_position", position),
            targetVelocity=0.0,
            force=require_finite("force_newtons", force_newtons),
            physicsClientId=self.physics_client_id,
            **kwargs,
        )

    def get_link_pose(self, body_id: int, link_index: int) -> Pose:
        if isinstance(link_index, bool) or not isinstance(link_index, int) or link_index < 0:
            raise SimulationError("link index is invalid")
        state = _pybullet().getLinkState(
            body_id,
            link_index,
            computeForwardKinematics=True,
            physicsClientId=self.physics_client_id,
        )
        if not isinstance(state, (tuple, list)) or len(state) < 6:
            raise SimulationError("link pose is unavailable")
        return pose_from_position_orientation(state[4], state[5])

    def get_base_velocity(self, body_id: int) -> tuple[Vector3, Vector3]:
        linear, angular = _pybullet().getBaseVelocity(
            body_id, physicsClientId=self.physics_client_id
        )
        return Vector3.from_xyz(linear), Vector3.from_xyz(angular)

    def calculate_inverse_kinematics(
        self,
        body_id: int,
        end_effector_link_index: int,
        target_pose: Pose,
        *,
        lower_limits: Sequence[float],
        upper_limits: Sequence[float],
        joint_ranges: Sequence[float],
        rest_poses: Sequence[float],
        damping: Sequence[float],
        current_positions: Sequence[float],
        max_iterations: int,
        residual_threshold: float,
        include_orientation: bool,
    ) -> tuple[float, ...]:
        """Solve IK with explicit limits, ranges, rest poses, damping, and iteration bound."""

        if (
            isinstance(end_effector_link_index, bool)
            or not isinstance(end_effector_link_index, int)
            or end_effector_link_index < 0
        ):
            raise SimulationError("end-effector link index is invalid")
        if isinstance(max_iterations, bool) or not isinstance(max_iterations, int):
            raise SimulationError("inverse kinematics iteration bound is invalid")
        if max_iterations <= 0:
            raise SimulationError("inverse kinematics iteration bound is invalid")
        if not isinstance(include_orientation, bool):
            raise SimulationError("include_orientation is invalid")
        threshold = require_positive(
            "residual_threshold", require_finite("residual_threshold", residual_threshold)
        )
        lowers = _finite_tuple("lower_limits", lower_limits)
        uppers = _finite_tuple("upper_limits", upper_limits)
        ranges = _finite_tuple("joint_ranges", joint_ranges)
        rests = _finite_tuple("rest_poses", rest_poses)
        damps = _finite_tuple("damping", damping)
        current = _finite_tuple("current_positions", current_positions)
        if len(lowers) != len(uppers) or len(lowers) != len(ranges):
            raise SimulationError("inverse kinematics arrays must have matching length")
        if len(lowers) != len(rests) or len(lowers) != len(damps) or len(lowers) != len(current):
            raise SimulationError("inverse kinematics arrays must have matching length")
        position = list(target_pose.position_meters.to_checksum_payload())
        solver_kwargs = {
            "lowerLimits": list(lowers),
            "upperLimits": list(uppers),
            "jointRanges": list(ranges),
            "restPoses": list(rests),
            "jointDamping": list(damps),
            "currentPositions": list(current),
            "maxNumIterations": max_iterations,
            "residualThreshold": threshold,
            "physicsClientId": self.physics_client_id,
        }
        try:
            if include_orientation:
                raw = _pybullet().calculateInverseKinematics(
                    body_id,
                    end_effector_link_index,
                    position,
                    list(target_pose.orientation_xyzw.to_checksum_payload()),
                    **solver_kwargs,
                )
            else:
                raw = _pybullet().calculateInverseKinematics(
                    body_id,
                    end_effector_link_index,
                    position,
                    **solver_kwargs,
                )
        except Exception as exc:
            raise SimulationError("inverse kinematics failed") from exc
        return _finite_tuple("ik_solution", raw)

    def create_dynamic_box(
        self,
        half_extents_meters: Vector3,
        pose: Pose,
        *,
        mass_kilograms: float,
        rgba: tuple[float, float, float, float],
        lateral_friction: float,
        spinning_friction: float,
        rolling_friction: float,
    ) -> int:
        mass = require_positive("mass_kilograms", require_finite("mass_kilograms", mass_kilograms))
        friction = require_positive(
            "lateral_friction", require_finite("lateral_friction", lateral_friction)
        )
        spinning = require_nonnegative(
            "spinning_friction", require_finite("spinning_friction", spinning_friction)
        )
        rolling = require_nonnegative(
            "rolling_friction", require_finite("rolling_friction", rolling_friction)
        )
        bullet = _pybullet()
        client_id = self.physics_client_id
        collision_id = int(
            bullet.createCollisionShape(
                bullet.GEOM_BOX,
                halfExtents=list(half_extents_meters.to_checksum_payload()),
                physicsClientId=client_id,
            )
        )
        visual_id = int(
            bullet.createVisualShape(
                bullet.GEOM_BOX,
                halfExtents=list(half_extents_meters.to_checksum_payload()),
                rgbaColor=list(rgba),
                physicsClientId=client_id,
            )
        )
        body_id = int(
            bullet.createMultiBody(
                baseMass=mass,
                baseCollisionShapeIndex=collision_id,
                baseVisualShapeIndex=visual_id,
                basePosition=list(pose.position_meters.to_checksum_payload()),
                baseOrientation=list(pose.orientation_xyzw.to_checksum_payload()),
                physicsClientId=client_id,
            )
        )
        if body_id < 0:
            raise SimulationError("failed to create a dynamic parcel body")
        try:
            bullet.changeDynamics(
                body_id,
                -1,
                lateralFriction=friction,
                spinningFriction=spinning,
                rollingFriction=rolling,
                restitution=0.0,
                physicsClientId=client_id,
            )
        except Exception as exc:
            raise SimulationError("failed to set parcel dynamics") from exc
        return body_id

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

    def get_contact_points(self) -> tuple[RawContactPoint, ...]:
        """Return the current contact manifold. Body ids remain ephemeral."""

        try:
            raw = _pybullet().getContactPoints(physicsClientId=self.physics_client_id)
        except Exception as exc:
            raise SimulationError("contact query failed") from exc
        if raw is None:
            return ()
        if not isinstance(raw, (list, tuple)):
            raise SimulationError("contact query returned an invalid manifold")
        return tuple(parse_engine_contact_point(item) for item in raw)


def parse_engine_contact_point(raw: object) -> RawContactPoint:
    """Parse one PyBullet contact tuple into a typed ephemeral sample."""

    if not isinstance(raw, (list, tuple)) or len(raw) < 10:
        raise SimulationError("contact query returned an incomplete contact point")
    try:
        return RawContactPoint(
            body_unique_id_a=_require_body_id(raw[1]),
            body_unique_id_b=_require_body_id(raw[2]),
            link_index_a=_require_link_index(raw[3]),
            link_index_b=_require_link_index(raw[4]),
            position_world_on_a_meters=Vector3.from_xyz(raw[5]),
            contact_normal_on_b=Vector3.from_xyz(raw[7]),
            normal_force_newtons=require_finite("normal_force_newtons", raw[9]),
        )
    except (TypeError, ValueError) as exc:
        raise SimulationError("contact query returned an invalid contact point") from exc


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


def _finite_tuple(field: str, values: object) -> tuple[float, ...]:
    if isinstance(values, (str, bytes, bytearray)) or not isinstance(values, Sequence):
        raise SimulationError(f"{field} is invalid")
    if len(values) == 0:
        raise SimulationError(f"{field} is invalid")
    return tuple(require_finite(f"{field}[{index}]", values[index]) for index in range(len(values)))


def _require_body_id(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise SimulationError("contact body id is invalid")
    return value


def _require_link_index(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < -1:
        raise SimulationError("contact link index is invalid")
    return value


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


def _decode_link_name(raw: object, index: int) -> str:
    try:
        return _decode_joint_name(raw)
    except SimulationError:
        return f"link_{index}"


def _rest_position(lower: float, upper: float) -> float:
    if lower <= 0.0 <= upper:
        return 0.0
    if upper <= lower:
        return 0.0
    return (lower + upper) / 2.0
