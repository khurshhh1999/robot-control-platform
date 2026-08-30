"""Scripted centered-cube pick-and-place using typed motion primitives."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final

from robot_control_platform_simulator.control.actions import (
    ActionResult,
    ActionStatus,
    MotionCommand,
    MotionController,
    MotionPrimitive,
    downward_pose,
)
from robot_control_platform_simulator.domain.models import Pose, QuaternionXYZW, Vector3
from robot_control_platform_simulator.physics.client import (
    WORLD_FRAME,
    PhysicsClient,
    SimulationError,
)
from robot_control_platform_simulator.physics.robot import (
    apply_layout_rest,
    discover_and_validate_robot_layout,
)
from robot_control_platform_simulator.physics.scene import (
    ROBOT_BODY_NAME,
    TABLE_TOP_Z_METERS,
    WorkcellScene,
    default_scene_config,
)

CENTERED_CUBE_HALF_EXTENTS_METERS: Final[Vector3] = Vector3(x=0.025, y=0.025, z=0.025)
CENTERED_CUBE_MASS_KILOGRAMS: Final[float] = 0.08
CENTERED_CUBE_LATERAL_FRICTION: Final[float] = 1.00
CENTERED_CUBE_SPINNING_FRICTION: Final[float] = 0.10
CENTERED_CUBE_ROLLING_FRICTION: Final[float] = 0.001
CENTERED_CUBE_RGBA: Final[tuple[float, float, float, float]] = (0.86, 0.45, 0.12, 1.0)
EE_TOOL_OFFSET_METERS: Final[float] = 0.13
EE_XY_OFFSET_METERS: Final[Vector3] = Vector3(x=-0.16, y=0.0, z=0.0)
APPROACH_CLEARANCE_METERS: Final[float] = 0.10
LIFT_CLEARANCE_METERS: Final[float] = 0.14
PLACE_OFFSET_METERS: Final[Vector3] = Vector3(x=0.08, y=0.12, z=0.0)
PLACE_SUCCESS_HALF_EXTENTS_METERS: Final[Vector3] = Vector3(x=0.08, y=0.08, z=0.10)
MOVE_TIMEOUT_SECONDS: Final[float] = 8.0
GRIPPER_TIMEOUT_SECONDS: Final[float] = 2.0
HOLD_TIMEOUT_SECONDS: Final[float] = 0.40
SETTLE_TIMEOUT_SECONDS: Final[float] = 1.00
MOVE_TOLERANCE_METERS: Final[float] = 0.04
GRIPPER_TOLERANCE_RADIANS: Final[float] = 0.05
SETTLE_VELOCITY_TOLERANCE: Final[float] = 0.25
_IDENTITY = QuaternionXYZW(x=0.0, y=0.0, z=0.0, w=1.0)


@dataclass(frozen=True)
class PickPlaceResult:
    """Geometric placement result plus the recorded motion primitive outcomes."""

    succeeded: bool
    cube_pose: Pose
    place_pose: Pose
    actions: tuple[ActionResult, ...]


def centered_cube_pose(scene: WorkcellScene | None = None) -> Pose:
    workcell = scene.scene_config if scene is not None else default_scene_config()
    pickup = workcell.pickup_pose.position_meters
    return Pose(
        position_meters=Vector3(
            x=pickup.x,
            y=pickup.y,
            z=TABLE_TOP_Z_METERS + CENTERED_CUBE_HALF_EXTENTS_METERS.z,
        ),
        orientation_xyzw=_IDENTITY,
        frame=WORLD_FRAME,
    )


def place_target_pose(cube_pose: Pose) -> Pose:
    return Pose(
        position_meters=Vector3(
            x=cube_pose.position_meters.x + PLACE_OFFSET_METERS.x,
            y=cube_pose.position_meters.y + PLACE_OFFSET_METERS.y,
            z=cube_pose.position_meters.z,
        ),
        orientation_xyzw=_IDENTITY,
        frame=WORLD_FRAME,
    )


def ee_pose_above_object(object_position: Vector3, *, height_offset_meters: float) -> Pose:
    return downward_pose(
        Vector3(
            x=object_position.x + EE_XY_OFFSET_METERS.x,
            y=object_position.y + EE_XY_OFFSET_METERS.y,
            z=object_position.z + height_offset_meters,
        )
    )


def object_in_target_region(object_pose: Pose, target_pose: Pose) -> bool:
    dx = abs(object_pose.position_meters.x - target_pose.position_meters.x)
    dy = abs(object_pose.position_meters.y - target_pose.position_meters.y)
    dz = abs(object_pose.position_meters.z - target_pose.position_meters.z)
    return (
        dx <= PLACE_SUCCESS_HALF_EXTENTS_METERS.x
        and dy <= PLACE_SUCCESS_HALF_EXTENTS_METERS.y
        and dz <= PLACE_SUCCESS_HALF_EXTENTS_METERS.z
    )


def spawn_centered_cube(client: PhysicsClient, scene: WorkcellScene) -> tuple[int, Pose]:
    pose = centered_cube_pose(scene)
    body_id = client.create_dynamic_box(
        CENTERED_CUBE_HALF_EXTENTS_METERS,
        pose,
        mass_kilograms=CENTERED_CUBE_MASS_KILOGRAMS,
        rgba=CENTERED_CUBE_RGBA,
        lateral_friction=CENTERED_CUBE_LATERAL_FRICTION,
        spinning_friction=CENTERED_CUBE_SPINNING_FRICTION,
        rolling_friction=CENTERED_CUBE_ROLLING_FRICTION,
    )
    return body_id, pose


def run_centered_cube_pick_place(client: PhysicsClient, scene: WorkcellScene) -> PickPlaceResult:
    scene.reset()
    robot_id = scene.body_id(ROBOT_BODY_NAME)
    layout = discover_and_validate_robot_layout(client, robot_id)
    apply_layout_rest(client, robot_id, layout)
    controller = MotionController(client, robot_id, layout)
    cube_id, cube_pose = spawn_centered_cube(client, scene)
    place_pose = place_target_pose(cube_pose)
    pick_xy = cube_pose.position_meters
    place_xy = place_pose.position_meters
    grasp_offset = EE_TOOL_OFFSET_METERS
    approach_offset = EE_TOOL_OFFSET_METERS + APPROACH_CLEARANCE_METERS
    lift_offset = EE_TOOL_OFFSET_METERS + LIFT_CLEARANCE_METERS
    commands = (
        MotionCommand(MotionPrimitive.SETTLE, SETTLE_TIMEOUT_SECONDS, SETTLE_VELOCITY_TOLERANCE),
        MotionCommand(MotionPrimitive.OPEN, GRIPPER_TIMEOUT_SECONDS, GRIPPER_TOLERANCE_RADIANS),
        MotionCommand(
            MotionPrimitive.MOVE_END_EFFECTOR,
            MOVE_TIMEOUT_SECONDS,
            MOVE_TOLERANCE_METERS,
            target_pose=ee_pose_above_object(pick_xy, height_offset_meters=approach_offset),
        ),
        MotionCommand(
            MotionPrimitive.MOVE_END_EFFECTOR,
            MOVE_TIMEOUT_SECONDS,
            MOVE_TOLERANCE_METERS,
            target_pose=ee_pose_above_object(pick_xy, height_offset_meters=grasp_offset),
        ),
        MotionCommand(MotionPrimitive.CLOSE, GRIPPER_TIMEOUT_SECONDS, GRIPPER_TOLERANCE_RADIANS),
        MotionCommand(MotionPrimitive.HOLD, HOLD_TIMEOUT_SECONDS, SETTLE_VELOCITY_TOLERANCE),
        MotionCommand(
            MotionPrimitive.MOVE_END_EFFECTOR,
            MOVE_TIMEOUT_SECONDS,
            MOVE_TOLERANCE_METERS,
            target_pose=ee_pose_above_object(pick_xy, height_offset_meters=lift_offset),
        ),
        MotionCommand(
            MotionPrimitive.MOVE_END_EFFECTOR,
            MOVE_TIMEOUT_SECONDS,
            MOVE_TOLERANCE_METERS,
            target_pose=ee_pose_above_object(place_xy, height_offset_meters=lift_offset),
        ),
        MotionCommand(
            MotionPrimitive.MOVE_END_EFFECTOR,
            MOVE_TIMEOUT_SECONDS,
            MOVE_TOLERANCE_METERS,
            target_pose=ee_pose_above_object(place_xy, height_offset_meters=grasp_offset),
        ),
        MotionCommand(MotionPrimitive.OPEN, GRIPPER_TIMEOUT_SECONDS, GRIPPER_TOLERANCE_RADIANS),
        MotionCommand(
            MotionPrimitive.RETRACT,
            MOVE_TIMEOUT_SECONDS,
            MOVE_TOLERANCE_METERS,
            target_pose=ee_pose_above_object(place_xy, height_offset_meters=lift_offset),
        ),
        MotionCommand(MotionPrimitive.SETTLE, SETTLE_TIMEOUT_SECONDS, SETTLE_VELOCITY_TOLERANCE),
    )
    actions: list[ActionResult] = []
    for command in commands:
        result = controller.execute(command)
        actions.append(result)
        if result.status is ActionStatus.IK_REJECTED:
            observed = client.get_base_pose(cube_id)
            return PickPlaceResult(
                succeeded=False,
                cube_pose=observed,
                place_pose=place_pose,
                actions=tuple(actions),
            )
        if result.status is ActionStatus.TIMEOUT and command.primitive in {
            MotionPrimitive.MOVE_END_EFFECTOR,
            MotionPrimitive.RETRACT,
            MotionPrimitive.OPEN,
        }:
            observed = client.get_base_pose(cube_id)
            return PickPlaceResult(
                succeeded=False,
                cube_pose=observed,
                place_pose=place_pose,
                actions=tuple(actions),
            )
    observed = client.get_base_pose(cube_id)
    return PickPlaceResult(
        succeeded=object_in_target_region(observed, place_pose),
        cube_pose=observed,
        place_pose=place_pose,
        actions=tuple(actions),
    )


def require_pick_place_primitives(actions: tuple[ActionResult, ...]) -> None:
    names = {item.primitive for item in actions}
    required = {
        MotionPrimitive.MOVE_END_EFFECTOR,
        MotionPrimitive.OPEN,
        MotionPrimitive.CLOSE,
        MotionPrimitive.HOLD,
        MotionPrimitive.RETRACT,
        MotionPrimitive.SETTLE,
    }
    if required - names:
        raise SimulationError("pick-and-place did not exercise every motion primitive")
