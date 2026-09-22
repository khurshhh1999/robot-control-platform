"""Trial execution boundary used by the simulator worker.

PyBullet stays behind this module. The worker checks cancellation between
controller states and does not call the probe during artifact finalization.
"""

from __future__ import annotations

import asyncio
import math
from collections.abc import Mapping
from dataclasses import dataclass
from enum import StrEnum
from typing import Any, Protocol, runtime_checkable
from uuid import UUID

from robot_control_platform_common.artifacts.base import (
    ArtifactIntegrityError,
    ArtifactMetadata,
    ArtifactStore,
    ArtifactStoreError,
)

from robot_control_platform_simulator.control.actions import (
    ActionStatus,
    MotionController,
    end_effector_pose,
)
from robot_control_platform_simulator.control.outcomes import (
    GraspSample,
    OutcomeEvidence,
    PlacementSample,
    classify_outcome,
    default_bin_poses,
    is_gripper_object_contact,
    sanitize_infrastructure_failure,
)
from robot_control_platform_simulator.control.state_machine import ControllerStateMachine
from robot_control_platform_simulator.domain.enums import ControllerState
from robot_control_platform_simulator.domain.events import ContactEvent
from robot_control_platform_simulator.domain.models import (
    ObjectState,
    Pose,
    QuaternionXYZW,
    Vector3,
)
from robot_control_platform_simulator.physics.camera import capture_rgb_frame, default_camera_config
from robot_control_platform_simulator.physics.client import (
    PHYSICS_SCHEMA_VERSION,
    WORLD_FRAME,
    PhysicsClient,
    SimulationError,
)
from robot_control_platform_simulator.physics.contacts import (
    CollisionMonitor,
    default_collision_config,
    sample_contacts,
)
from robot_control_platform_simulator.physics.robot import (
    GRIPPER_CLOSED_RADIANS,
    GRIPPER_OPEN_RADIANS,
    RobotLayout,
    apply_layout_rest,
    discover_and_validate_robot_layout,
    joint_states_from_specs,
    solve_inverse_kinematics,
)
from robot_control_platform_simulator.physics.scene import (
    ROBOT_BODY_NAME,
    WorkcellScene,
    default_scene_config,
)
from robot_control_platform_simulator.policies.base import (
    POLICY_ALLOWLIST,
    POLICY_VERSION_NAMES,
    BasePolicy,
    PolicyConfig,
    PolicyObservation,
    ReachabilityAssessment,
    create_policy,
    default_config_for,
    default_reachability,
)
from robot_control_platform_simulator.recording.manifests import TrialProvenance, TrialRecorder
from robot_control_platform_simulator.recording.milestones import MilestoneCapture
from robot_control_platform_simulator.recording.trajectory import TrajectorySample
from robot_control_platform_simulator.scenarios.generator import (
    OBJECT_SHAPE_BOX,
    OBJECT_SHAPE_CYLINDER,
    AllowedPerturbations,
    ObjectTypeSpec,
    Scenario,
    default_allowed_perturbations,
)

_IMPLEMENTATION_BY_NAME: dict[str, str] = {
    version_name: implementation for implementation, version_name in POLICY_VERSION_NAMES.items()
}
for _implementation in POLICY_ALLOWLIST:
    _IMPLEMENTATION_BY_NAME.setdefault(_implementation, _implementation)

_PARCEL_RGBA: tuple[float, float, float, float] = (0.80, 0.46, 0.18, 1.0)
_DEFAULT_HALF_EXTENTS: tuple[float, float, float] = (0.025, 0.025, 0.025)
_DEFAULT_SPINNING_FRICTION = 0.10
_DEFAULT_ROLLING_FRICTION = 0.001
_GRIPPER_CLOSED_MAX_RADIANS = 0.05


class ExecutionSignal(StrEnum):
    CONTINUE = "continue"
    CANCEL = "cancel"
    STOP = "stop"


@dataclass(frozen=True)
class RecordedEvent:
    """One trial event ready for database insert. Times are seconds."""

    ordinal: int
    event_type: str
    controller_state: str
    simulation_time_seconds: float
    detail: str | None = None


@dataclass(frozen=True)
class StoredArtifact:
    """Checksummed artifact bytes plus optional image dimensions."""

    metadata: ArtifactMetadata
    width_px: int | None = None
    height_px: int | None = None


@dataclass(frozen=True)
class TrialContext:
    """Immutable inputs for one scenario/policy trial."""

    experiment_id: UUID
    run_id: UUID
    trial_id: UUID
    policy_version_id: UUID
    scenario_id: UUID
    execution_order: int
    scenario_ordinal: int
    scenario_seed: int
    policy_name: str
    policy_config: dict[str, Any]
    policy_config_sha256: str
    source_revision: str
    scenario_checksum: str
    generator_version: str
    object_name: str
    object_category: str
    target_bin: str
    initial_pose: dict[str, Any]
    physical_properties: dict[str, Any]


@dataclass(frozen=True)
class TrialExecution:
    """Classified trial plus the evidence to commit in one transaction."""

    outcome: str
    collision_count: int
    collision_max_force_newtons: float
    duration_seconds: float
    action_count: int
    events: tuple[RecordedEvent, ...]
    artifacts: tuple[StoredArtifact, ...]
    system_error_code: str | None = None


class TrialInterrupted(Exception):
    """Shutdown was observed between controller states, before finalization."""

    def __init__(self, partial_events: tuple[RecordedEvent, ...] = ()) -> None:
        self.partial_events = partial_events
        super().__init__("stop")


class TrialCancelled(Exception):
    """Run cancellation was observed between controller states, before finalization."""

    def __init__(self, partial_events: tuple[RecordedEvent, ...] = ()) -> None:
        self.partial_events = partial_events
        super().__init__("cancel")


class TrialExecutionFailure(Exception):
    """Sanitized trial failure. The message is an allowlisted system-error code."""

    def __init__(
        self,
        code: str,
        *,
        partial_events: tuple[RecordedEvent, ...] = (),
        storage_keys: tuple[str, ...] = (),
    ) -> None:
        self.code = sanitize_infrastructure_failure(code)
        self.partial_events = partial_events
        self.storage_keys = storage_keys
        super().__init__(self.code)


@runtime_checkable
class ControlProbe(Protocol):
    """Asked before each controller state. Not called during artifact finalization."""

    async def before_state(self, controller_state: str) -> ExecutionSignal: ...


@runtime_checkable
class TrialExecutor(Protocol):
    """Execute one trial without holding a database session."""

    async def execute(
        self,
        context: TrialContext,
        store: ArtifactStore,
        probe: ControlProbe,
    ) -> TrialExecution: ...


def implementation_for_policy_name(name: str) -> str:
    """Map a stored policy name to an allowlisted implementation."""

    try:
        return _IMPLEMENTATION_BY_NAME[name]
    except KeyError as exc:
        msg = "policy implementation is not allowlisted"
        raise ValueError(msg) from exc


def policy_config_from_payload(payload: Mapping[str, Any]) -> PolicyConfig:
    """Rebuild a policy config from its canonical checksum payload."""

    offset = payload["ee_xy_offset_meters"]
    return PolicyConfig(
        approach_clearance_meters=float(payload["approach_clearance_meters"]),
        lift_clearance_meters=float(payload["lift_clearance_meters"]),
        via_clearance_meters=float(payload["via_clearance_meters"]),
        grasp_tool_offset_meters=float(payload["grasp_tool_offset_meters"]),
        nominal_object_half_height_meters=float(payload["nominal_object_half_height_meters"]),
        table_top_z_meters=float(payload["table_top_z_meters"]),
        ee_xy_offset_meters=Vector3.from_xyz(offset),
        move_timeout_seconds=float(payload["move_timeout_seconds"]),
        gripper_timeout_seconds=float(payload["gripper_timeout_seconds"]),
        hold_timeout_seconds=float(payload["hold_timeout_seconds"]),
        settle_timeout_seconds=float(payload["settle_timeout_seconds"]),
        move_tolerance_meters=float(payload["move_tolerance_meters"]),
        gripper_tolerance_radians=float(payload["gripper_tolerance_radians"]),
        settle_velocity_tolerance=float(payload["settle_velocity_tolerance"]),
        regrasp_limit=int(payload["regrasp_limit"]),
        approach_stage_count=int(payload["approach_stage_count"]),
        transfer_stage_count=int(payload["transfer_stage_count"]),
        release_settle_count=int(payload["release_settle_count"]),
    )


def resolve_policy_config(
    policy_name: str,
    payload: Mapping[str, Any],
    checksum: str,
) -> tuple[str, PolicyConfig]:
    """Return the allowlisted implementation and a checksum-matching config."""

    implementation = implementation_for_policy_name(policy_name)
    parsed: PolicyConfig | None
    try:
        parsed = policy_config_from_payload(payload)
    except (KeyError, TypeError, ValueError):
        parsed = None
    if parsed is not None and parsed.sha256_hex() == checksum:
        return implementation, parsed
    fallback = default_config_for(implementation)
    if fallback.sha256_hex() == checksum:
        return implementation, fallback
    msg = "evaluation_unavailable"
    raise ValueError(msg)


def simulator_version_label() -> str:
    """Return a path-free simulator version string."""

    try:
        import pybullet
    except ImportError:
        return PHYSICS_SCHEMA_VERSION
    version = getattr(pybullet, "__version__", "")
    if (
        isinstance(version, str)
        and version.strip() != ""
        and "/" not in version
        and "\\" not in version
    ):
        return f"pybullet-{version.strip()}"
    return PHYSICS_SCHEMA_VERSION


class PhysicsTrialExecutor:
    """Run one trial on the headless workcell and finalize artifacts last."""

    def __init__(self, *, gui: bool = False) -> None:
        self._gui = gui

    async def execute(
        self,
        context: TrialContext,
        store: ArtifactStore,
        probe: ControlProbe,
    ) -> TrialExecution:
        world = _PhysicsTrialWorld(context, store, gui=self._gui)
        try:
            await asyncio.to_thread(world.open)
            while not world.is_terminal():
                signal = await probe.before_state(world.current_state())
                if signal is ExecutionSignal.STOP:
                    raise TrialInterrupted(world.events())
                if signal is ExecutionSignal.CANCEL:
                    raise TrialCancelled(world.events())
                await asyncio.to_thread(world.execute_current_state)
            return await asyncio.to_thread(world.finalize)
        except (TrialInterrupted, TrialCancelled, TrialExecutionFailure):
            raise
        except (SimulationError, ArtifactIntegrityError, ArtifactStoreError, ValueError, OSError):
            raise TrialExecutionFailure(
                "simulation_error",
                partial_events=world.events(),
                storage_keys=world.storage_keys(),
            ) from None
        finally:
            await asyncio.to_thread(world.close)


class _PhysicsTrialWorld:
    """One connected workcell. Methods that touch PyBullet run on a worker thread."""

    def __init__(self, context: TrialContext, store: ArtifactStore, *, gui: bool) -> None:
        self._context = context
        self._store = store
        self._gui = gui
        self._client: PhysicsClient | None = None
        self._scene: WorkcellScene | None = None
        self._controller: MotionController | None = None
        self._layout: RobotLayout | None = None
        self._policy: BasePolicy | None = None
        self._scenario: Scenario | None = None
        self._machine: ControllerStateMachine | None = None
        self._recorder: TrialRecorder | None = None
        self._collisions = CollisionMonitor(default_collision_config())
        self._object_body_id: int | None = None
        self._object_name = context.object_name
        self._mass_kilograms = 0.1
        self._robot_id = 0
        self._gripper_opening = GRIPPER_OPEN_RADIANS
        self._gripper_closed = False
        self._released = False
        self._grasp_verified = False
        self._regrasp_count = 0
        self._collision_detected = False
        self._collision_count = 0
        self._max_force = 0.0
        self._action_count = 0
        self._last_action_status: ActionStatus | None = None
        self._reachability = default_reachability()
        self._grasp_samples: list[GraspSample] = []
        self._placement_samples: list[PlacementSample] = []
        self._initial_event: RecordedEvent | None = None

    def open(self) -> None:
        try:
            implementation, config = resolve_policy_config(
                self._context.policy_name,
                self._context.policy_config,
                self._context.policy_config_sha256,
            )
            scenario = _scenario_from_context(self._context)
            policy = create_policy(implementation, config)
            if not isinstance(policy, BasePolicy):
                raise TrialExecutionFailure("evaluation_unavailable")
            client = PhysicsClient(gui=self._gui)
            client.connect()
            self._client = client
            scene = WorkcellScene(client)
            scene.reset()
            self._scene = scene
            robot_id = scene.body_id(ROBOT_BODY_NAME)
            layout = discover_and_validate_robot_layout(client, robot_id)
            apply_layout_rest(client, robot_id, layout)
            self._layout = layout
            self._robot_id = robot_id
            self._controller = MotionController(client, robot_id, layout)
            self._policy = policy
            self._scenario = scenario
            self._mass_kilograms = scenario.mass_kilograms
            self._object_name = scenario.object_type.name
            self._object_body_id = _spawn_parcel(client, scenario)
            self._machine = ControllerStateMachine(simulation_time_seconds=self._time())
            self._recorder = TrialRecorder(
                self._store,
                experiment_id=self._context.experiment_id,
                trial_id=self._context.trial_id,
                capture=self._capture,
            )
            pose = client.get_base_pose(self._object_body_id)
            self._initial_event = RecordedEvent(
                ordinal=0,
                event_type="observation",
                controller_state=ControllerState.RESET.value,
                simulation_time_seconds=self._time(),
                detail="initial_state",
            )
            self._record_trajectory(ControllerState.RESET, pose)
            _ = pose
        except TrialExecutionFailure:
            raise
        except (SimulationError, ValueError, OSError) as exc:
            raise TrialExecutionFailure(sanitize_infrastructure_failure(exc)) from None

    def close(self) -> None:
        client = self._client
        self._client = None
        if client is not None:
            client.disconnect()

    def is_terminal(self) -> bool:
        machine = self._machine
        return machine is not None and machine.state is ControllerState.TERMINAL

    def current_state(self) -> str:
        machine = self._machine
        if machine is None:
            return ControllerState.RESET.value
        return machine.state.value

    def events(self) -> tuple[RecordedEvent, ...]:
        rows: list[RecordedEvent] = []
        if self._initial_event is not None:
            rows.append(self._initial_event)
        machine = self._machine
        if machine is not None:
            for event in machine.events:
                rows.append(
                    RecordedEvent(
                        ordinal=0,
                        event_type=event.event_type.value,
                        controller_state=event.controller_state.value,
                        simulation_time_seconds=event.simulation_time_seconds,
                        detail=event.detail,
                    )
                )
        return tuple(
            RecordedEvent(
                ordinal=index,
                event_type=event.event_type,
                controller_state=event.controller_state,
                simulation_time_seconds=event.simulation_time_seconds,
                detail=event.detail,
            )
            for index, event in enumerate(rows)
        )

    def storage_keys(self) -> tuple[str, ...]:
        recorder = self._recorder
        if recorder is None:
            return ()
        keys = [metadata.storage_key for metadata in recorder.milestones.written.values()]
        trajectory = recorder.trajectory.metadata
        if trajectory is not None:
            keys.append(trajectory.storage_key)
        return tuple(keys)

    def execute_current_state(self) -> None:
        machine = self._require_machine()
        policy = self._require_policy()
        scenario = self._require_scenario()
        controller = self._require_controller()
        recorder = self._require_recorder()
        state = machine.state
        if state is ControllerState.TERMINAL:
            return
        observation = self._observe(state)
        decision = policy.plan(observation, scenario, policy.config)
        for command in decision.commands:
            result = controller.execute(command)
            self._action_count += 1
            self._last_action_status = result.status
            if command.primitive.value == "close":
                self._gripper_closed = True
                self._gripper_opening = GRIPPER_CLOSED_RADIANS
            elif command.primitive.value == "open":
                self._released = True
                self._gripper_closed = False
                self._gripper_opening = GRIPPER_OPEN_RADIANS
            self._sample_motion(state)
        if decision.regrasp:
            self._regrasp_count += 1
        source = state
        target = decision.next_state
        machine.transition(
            target,
            simulation_time_seconds=self._time(),
            failed=decision.abort,
            detail=decision.reason,
        )
        recorder.on_transition(source, target)
        self._sample_motion(target)

    def finalize(self) -> TrialExecution:
        """Write the trajectory and manifest. Callers must not cancel this step."""

        recorder = self._require_recorder()
        scenario = self._require_scenario()
        self._sample_motion(ControllerState.TERMINAL)
        camera_checksum = (
            recorder.milestones.camera_checksum or default_camera_config().sha256_hex()
        )
        provenance = TrialProvenance(
            camera_checksum=camera_checksum,
            scenario_checksum=self._context.scenario_checksum,
            policy_checksum=self._context.policy_config_sha256,
            simulator_version=simulator_version_label(),
            source_revision=self._context.source_revision,
        )
        written = recorder.finalize(provenance)
        evidence = OutcomeEvidence(
            events=self._require_machine().events,
            collision_detected=self._collision_detected,
            infrastructure_failure=None,
            gripper_closed=self._gripper_closed,
            released=self._released,
            object_id=self._object_name,
            target_bin=scenario.target_bin,
            bin_poses=default_bin_poses(),
            initial_object_position_meters=scenario.initial_pose.position_meters,
            grasp_samples=tuple(self._grasp_samples),
            placement_samples=tuple(self._placement_samples),
        )
        classification = classify_outcome(evidence)
        artifacts = tuple(
            StoredArtifact(
                metadata=metadata,
                width_px=640 if metadata.kind.endswith("_rgb") else None,
                height_px=480 if metadata.kind.endswith("_rgb") else None,
            )
            for metadata in written.values()
        )
        system_code = classification.system_error_code
        return TrialExecution(
            outcome=classification.outcome.value,
            collision_count=self._collision_count,
            collision_max_force_newtons=self._max_force,
            duration_seconds=self._time(),
            action_count=self._action_count,
            events=self.events(),
            artifacts=artifacts,
            system_error_code=system_code,
        )

    def _observe(self, state: ControllerState) -> PolicyObservation:
        contacts = self._sample_contacts()
        now = self._time()
        assessment = self._collisions.observe(contacts, simulation_time_seconds=now)
        if assessment.collision_detected:
            self._collision_detected = True
        if assessment.collision_contacts:
            self._collision_count += len(assessment.collision_contacts)
            peak = max(contact.force_newtons for contact in assessment.collision_contacts)
            if peak > self._max_force:
                self._max_force = peak
        object_pose = self._object_pose()
        grasp_contact = any(
            is_gripper_object_contact(contact, self._object_name) for contact in contacts
        )
        grasp_force = 0.0
        for contact in contacts:
            if is_gripper_object_contact(contact, self._object_name):
                grasp_force = max(grasp_force, contact.force_newtons)
        if grasp_contact:
            self._grasp_verified = True
        self._grasp_samples.append(
            GraspSample(
                simulation_time_seconds=now,
                object_position_meters=object_pose.position_meters,
                gripper_object_contact=grasp_contact,
                contact_force_newtons=grasp_force,
            )
        )
        linear, angular = self._object_velocity()
        self._placement_samples.append(
            PlacementSample(
                simulation_time_seconds=now,
                object_position_meters=object_pose.position_meters,
                linear_speed_meters_per_second=_speed(linear),
                angular_speed_radians_per_second=_speed(angular),
            )
        )
        if state is ControllerState.PLAN:
            self._reachability = self._assess_reachability()
        return PolicyObservation(
            controller_state=state,
            simulation_time_seconds=now,
            object_state=self._object_state(object_pose, linear, angular),
            end_effector_pose=self._end_effector_pose(),
            gripper_opening_radians=self._gripper_opening,
            gripper_closed=self._gripper_opening <= _GRIPPER_CLOSED_MAX_RADIANS,
            contacts=contacts,
            collision_detected=self._collision_detected,
            grasp_verified=self._grasp_verified,
            regrasp_count=self._regrasp_count,
            last_action_status=self._last_action_status,
            reachability=self._reachability,
        )

    def _assess_reachability(self) -> ReachabilityAssessment:
        policy = self._require_policy()
        scenario = self._require_scenario()
        client = self._require_client()
        layout = self._require_layout()
        config = policy.config
        object_position = policy.planned_object_position(
            self._observe_without_plan(), scenario, config
        )
        place_position = policy.planned_place_position(
            self._observe_without_plan(), scenario, config
        )
        approach_z = (
            object_position.z + config.grasp_tool_offset_meters + config.approach_clearance_meters
        )
        grasp_z = object_position.z + config.grasp_tool_offset_meters
        lift_z = object_position.z + config.grasp_tool_offset_meters + config.lift_clearance_meters
        place_z = place_position.z + config.grasp_tool_offset_meters
        staged: list[bool] = []
        for stage in range(config.approach_stage_count, 0, -1):
            staged_z = approach_z + float(stage) * config.via_clearance_meters
            staged.append(
                _ik_reaches(
                    client,
                    self._robot_id,
                    layout,
                    end_effector_pose(object_position, staged_z, config.ee_xy_offset_meters),
                )
            )
        return ReachabilityAssessment(
            approach=_ik_reaches(
                client,
                self._robot_id,
                layout,
                end_effector_pose(object_position, approach_z, config.ee_xy_offset_meters),
            ),
            grasp=_ik_reaches(
                client,
                self._robot_id,
                layout,
                end_effector_pose(object_position, grasp_z, config.ee_xy_offset_meters),
            ),
            lift=_ik_reaches(
                client,
                self._robot_id,
                layout,
                end_effector_pose(object_position, lift_z, config.ee_xy_offset_meters),
            ),
            place=_ik_reaches(
                client,
                self._robot_id,
                layout,
                end_effector_pose(place_position, place_z, config.ee_xy_offset_meters),
            ),
            staged_waypoints=tuple(staged),
        )

    def _observe_without_plan(self) -> PolicyObservation:
        """Observation used only to read planned positions. Reachability stays cached."""

        object_pose = self._object_pose()
        linear, angular = self._object_velocity()
        return PolicyObservation(
            controller_state=ControllerState.PLAN,
            simulation_time_seconds=self._time(),
            object_state=self._object_state(object_pose, linear, angular),
            end_effector_pose=self._end_effector_pose(),
            gripper_opening_radians=self._gripper_opening,
            gripper_closed=self._gripper_opening <= _GRIPPER_CLOSED_MAX_RADIANS,
            contacts=(),
            collision_detected=self._collision_detected,
            grasp_verified=self._grasp_verified,
            regrasp_count=self._regrasp_count,
            last_action_status=self._last_action_status,
            reachability=self._reachability,
        )

    def _sample_contacts(self) -> tuple[ContactEvent, ...]:
        client = self._require_client()
        scene = self._require_scene()
        names = scene.body_id_to_name()
        if self._object_body_id is not None:
            names[self._object_body_id] = self._object_name
        return sample_contacts(
            client,
            names,
            simulation_time_seconds=self._time(),
        )

    def _sample_motion(self, state: ControllerState) -> None:
        recorder = self._recorder
        layout = self._layout
        if recorder is None or layout is None or self._object_body_id is None:
            return
        object_pose = self._object_pose()
        linear, angular = self._object_velocity()
        client = self._require_client()
        recorder.record_trajectory_sample(
            TrajectorySample(
                simulation_time_seconds=self._time(),
                controller_state=state,
                joints=joint_states_from_specs(client, self._robot_id, layout.controlled_joints),
                end_effector_pose=self._end_effector_pose(),
                object_state=self._object_state(object_pose, linear, angular),
                gripper_opening_radians=self._gripper_opening,
            )
        )

    def _capture(self, kind: Any) -> MilestoneCapture:
        _ = kind
        frame = capture_rgb_frame(self._require_client())
        return MilestoneCapture(
            png_bytes=frame.png_bytes,
            camera_checksum=frame.camera_checksum,
            width_px=frame.width_px,
            height_px=frame.height_px,
        )

    def _record_trajectory(self, state: ControllerState, object_pose: Pose) -> None:
        _ = object_pose
        self._sample_motion(state)

    def _time(self) -> float:
        return self._require_client().simulation_time_seconds()

    def _object_pose(self) -> Pose:
        if self._object_body_id is None:
            raise TrialExecutionFailure("simulation_error")
        return self._require_client().get_base_pose(self._object_body_id)

    def _object_velocity(self) -> tuple[Vector3, Vector3]:
        if self._object_body_id is None:
            raise TrialExecutionFailure("simulation_error")
        return self._require_client().get_base_velocity(self._object_body_id)

    def _object_state(self, pose: Pose, linear: Vector3, angular: Vector3) -> ObjectState:
        return ObjectState(
            object_id=self._object_name,
            pose=pose,
            mass_kilograms=self._mass_kilograms,
            linear_velocity_meters_per_second=linear,
            angular_velocity_radians_per_second=angular,
        )

    def _end_effector_pose(self) -> Pose:
        layout = self._require_layout()
        return self._require_client().get_link_pose(
            self._robot_id,
            layout.end_effector_link_index,
        )

    def _require_client(self) -> PhysicsClient:
        if self._client is None:
            raise TrialExecutionFailure("simulation_error")
        return self._client

    def _require_scene(self) -> WorkcellScene:
        if self._scene is None:
            raise TrialExecutionFailure("simulation_error")
        return self._scene

    def _require_layout(self) -> RobotLayout:
        if self._layout is None:
            raise TrialExecutionFailure("simulation_error")
        return self._layout

    def _require_controller(self) -> MotionController:
        if self._controller is None:
            raise TrialExecutionFailure("simulation_error")
        return self._controller

    def _require_policy(self) -> BasePolicy:
        if self._policy is None:
            raise TrialExecutionFailure("evaluation_unavailable")
        return self._policy

    def _require_scenario(self) -> Scenario:
        if self._scenario is None:
            raise TrialExecutionFailure("evaluation_unavailable")
        return self._scenario

    def _require_machine(self) -> ControllerStateMachine:
        if self._machine is None:
            raise TrialExecutionFailure("simulation_error")
        return self._machine

    def _require_recorder(self) -> TrialRecorder:
        if self._recorder is None:
            raise TrialExecutionFailure("simulation_error")
        return self._recorder


def _ik_reaches(client: PhysicsClient, robot_id: int, layout: RobotLayout, pose: Pose) -> bool:
    try:
        solve_inverse_kinematics(client, robot_id, layout, pose)
    except (SimulationError, ValueError):
        return False
    return True


def _speed(vector: Vector3) -> float:
    magnitude = math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)
    if not math.isfinite(magnitude):
        return 0.0
    return magnitude


def _spawn_parcel(client: PhysicsClient, scenario: Scenario) -> int:
    extents = scenario.object_type.half_extents_meters
    if scenario.object_type.shape == OBJECT_SHAPE_CYLINDER:
        return client.create_dynamic_cylinder(
            radius_meters=extents.x,
            height_meters=extents.z * 2.0,
            pose=scenario.initial_pose,
            mass_kilograms=scenario.mass_kilograms,
            rgba=_PARCEL_RGBA,
            lateral_friction=scenario.lateral_friction,
            spinning_friction=_DEFAULT_SPINNING_FRICTION,
            rolling_friction=_DEFAULT_ROLLING_FRICTION,
        )
    if scenario.object_type.shape != OBJECT_SHAPE_BOX:
        raise TrialExecutionFailure("evaluation_unavailable")
    return client.create_dynamic_box(
        extents,
        scenario.initial_pose,
        mass_kilograms=scenario.mass_kilograms,
        rgba=_PARCEL_RGBA,
        lateral_friction=scenario.lateral_friction,
        spinning_friction=_DEFAULT_SPINNING_FRICTION,
        rolling_friction=_DEFAULT_ROLLING_FRICTION,
    )


def _scenario_from_context(context: TrialContext) -> Scenario:
    payload = context.initial_pose
    if "object_type" in payload and "generator_version" in payload:
        return Scenario.from_checksum_payload(payload)
    properties = context.physical_properties
    half = properties.get("half_extents_meters", list(_DEFAULT_HALF_EXTENTS))
    shape = properties.get("shape", OBJECT_SHAPE_BOX)
    if not isinstance(shape, str):
        raise TrialExecutionFailure("evaluation_unavailable")
    mass = properties.get("mass_kilograms", properties.get("mass_kg", 0.1))
    friction = properties.get("lateral_friction", properties.get("friction", 0.5))
    pose = _pose_from_storage(context.initial_pose)
    scene = default_scene_config()
    bin_pose = None
    for name, candidate in scene.bin_poses:
        if name == context.target_bin:
            bin_pose = candidate
            break
    if bin_pose is None:
        raise TrialExecutionFailure("evaluation_unavailable")
    perturbations = _perturbations(properties.get("allowed_perturbations"))
    return Scenario(
        seed=context.scenario_seed,
        generator_version=context.generator_version,
        object_type=ObjectTypeSpec(
            name=context.object_name,
            category=context.object_category,
            shape=shape,
            half_extents_meters=Vector3.from_xyz(half),
        ),
        initial_pose=pose,
        mass_kilograms=float(mass),
        lateral_friction=float(friction),
        target_bin=context.target_bin,
        target_bin_pose=bin_pose,
        allowed_perturbations=perturbations,
        generator_config_checksum=context.scenario_checksum,
        scene_checksum=scene.sha256_hex(),
    )


def _perturbations(payload: object) -> AllowedPerturbations:
    if isinstance(payload, Mapping):
        return AllowedPerturbations.from_checksum_payload(dict(payload))
    return default_allowed_perturbations()


def _pose_from_storage(payload: Mapping[str, Any]) -> Pose:
    if "position_meters" in payload and "orientation_xyzw" in payload:
        if "schema_version" in payload:
            return Pose.from_checksum_payload(dict(payload))
        return Pose(
            position_meters=Vector3.from_xyz(payload["position_meters"]),
            orientation_xyzw=QuaternionXYZW.from_xyzw(payload["orientation_xyzw"]),
            frame=WORLD_FRAME,
        )
    position = payload.get("position_m", payload.get("position_meters"))
    orientation = payload.get("orientation_xyzw")
    if position is None or orientation is None:
        raise TrialExecutionFailure("evaluation_unavailable")
    return Pose(
        position_meters=Vector3.from_xyz(position),
        orientation_xyzw=QuaternionXYZW.from_xyzw(orientation),
        frame=WORLD_FRAME,
    )
