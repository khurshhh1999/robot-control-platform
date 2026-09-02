from __future__ import annotations

import ast
import inspect
from dataclasses import replace
from pathlib import Path

import pytest
from robot_control_platform_common.ids import new_id
from robot_control_platform_simulator.control.actions import (
    EE_XY_OFFSET_METERS,
    ActionStatus,
    MotionCommand,
    MotionPrimitive,
    downward_pose,
)
from robot_control_platform_simulator.control.state_machine import is_allowed_transition
from robot_control_platform_simulator.domain.enums import ControllerState, ExperimentStatus
from robot_control_platform_simulator.domain.models import Action, ObjectState, Pose, Vector3
from robot_control_platform_simulator.physics.client import WORLD_FRAME
from robot_control_platform_simulator.policies import (
    POLICY_ALLOWLIST,
    POLICY_COLLISION_AWARE,
    POLICY_FIXED,
    POLICY_POSE_AWARE,
    POLICY_VERSION_NAMES,
    CollisionAwarePolicy,
    FixedPolicy,
    Policy,
    PolicyDecision,
    PolicyImmutableError,
    PolicyObservation,
    PolicyVersion,
    PoseAwarePolicy,
    ReachabilityAssessment,
    create_policy,
    default_config_for,
    default_fixed_policy_config,
    default_policy_registry,
    default_pose_aware_policy_config,
    default_reachability,
    make_policy_version,
    policy_config_checksum,
)
from robot_control_platform_simulator.scenarios.generator import Scenario, generate_scenario
from robot_control_platform_simulator.scenarios.golden import GOLDEN_SEEDS

_POLICY_DIR = (
    Path(__file__).resolve().parents[1]
    / "services"
    / "simulator"
    / "robot_control_platform_simulator"
    / "policies"
)
_HAPPY_STATES = (
    ControllerState.RESET,
    ControllerState.OBSERVE,
    ControllerState.PLAN,
    ControllerState.APPROACH,
    ControllerState.GRASP,
    ControllerState.VERIFY_GRASP,
    ControllerState.LIFT,
    ControllerState.TRANSFER,
    ControllerState.RELEASE,
    ControllerState.VERIFY_PLACE,
    ControllerState.RETRACT,
)


def _object_state(scenario: Scenario, pose: Pose | None = None) -> ObjectState:
    return ObjectState(
        object_id=scenario.object_type.name,
        pose=pose if pose is not None else scenario.initial_pose,
        mass_kilograms=scenario.mass_kilograms,
        linear_velocity_meters_per_second=Vector3(x=0.0, y=0.0, z=0.0),
        angular_velocity_radians_per_second=Vector3(x=0.0, y=0.0, z=0.0),
    )


def _observation(
    scenario: Scenario,
    *,
    state: ControllerState = ControllerState.APPROACH,
    grasp_verified: bool = False,
    regrasp_count: int = 0,
    collision_detected: bool = False,
    reachability: ReachabilityAssessment | None = None,
    live_pose: Pose | None = None,
    last_action_status: ActionStatus | None = None,
    simulation_time_seconds: float = 0.4,
) -> PolicyObservation:
    return PolicyObservation(
        controller_state=state,
        simulation_time_seconds=simulation_time_seconds,
        object_state=_object_state(scenario, live_pose),
        end_effector_pose=downward_pose(Vector3(x=0.30, y=0.0, z=0.90)),
        gripper_opening_radians=0.30,
        gripper_closed=False,
        contacts=(),
        collision_detected=collision_detected,
        grasp_verified=grasp_verified,
        regrasp_count=regrasp_count,
        last_action_status=last_action_status,
        reachability=reachability if reachability is not None else default_reachability(),
    )


def _moves(decision: PolicyDecision) -> tuple[MotionCommand, ...]:
    return tuple(
        command
        for command in decision.commands
        if command.primitive is MotionPrimitive.MOVE_END_EFFECTOR
    )


def _world_xy(command: MotionCommand) -> tuple[float, float]:
    pose = command.target_pose
    assert pose is not None
    offset = EE_XY_OFFSET_METERS
    return (pose.position_meters.x - offset.x, pose.position_meters.y - offset.y)


def _shifted_pose(scenario: Scenario) -> Pose:
    initial = scenario.initial_pose.position_meters
    return Pose(
        position_meters=Vector3(x=initial.x + 0.04, y=initial.y - 0.03, z=initial.z + 0.02),
        orientation_xyzw=scenario.initial_pose.orientation_xyzw,
        frame=WORLD_FRAME,
    )


def test_allowlist_contains_only_the_three_frozen_policies() -> None:
    assert POLICY_ALLOWLIST == (POLICY_FIXED, POLICY_POSE_AWARE, POLICY_COLLISION_AWARE)
    assert POLICY_VERSION_NAMES == {
        POLICY_FIXED: "v1_fixed",
        POLICY_POSE_AWARE: "v2_pose_aware",
        POLICY_COLLISION_AWARE: "v3_collision_aware",
    }
    with pytest.raises(ValueError, match="allowlisted"):
        create_policy("custom")
    with pytest.raises(ValueError, match="allowlisted"):
        make_policy_version("scripted")


def test_create_policy_returns_runtime_protocol_instances() -> None:
    for implementation in POLICY_ALLOWLIST:
        policy = create_policy(implementation)
        assert isinstance(policy, Policy)
        assert policy.implementation == implementation
        parameters = list(inspect.signature(policy.plan).parameters)
        assert parameters == ["observation", "scenario", "config"]
        assert "rng" not in parameters
        assert "random" not in parameters


def test_policy_modules_do_not_use_uncontrolled_rng_or_physics() -> None:
    forbidden = {
        "random",
        "numpy.random",
        "pybullet",
        "PhysicsClient",
    }
    for path in sorted(_POLICY_DIR.glob("*.py")):
        source = path.read_text(encoding="utf-8")
        tree = ast.parse(source)
        imported: set[str] = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported.update(alias.name for alias in node.names)
            if isinstance(node, ast.ImportFrom) and node.module is not None:
                imported.add(node.module)
                imported.update(f"{node.module}.{alias.name}" for alias in node.names)
        for name in forbidden:
            assert name not in imported, f"{path.name} imports {name}"
        assert "numpy.random" not in source
        assert "np.random" not in source


@pytest.mark.parametrize("seed", GOLDEN_SEEDS)
@pytest.mark.parametrize("implementation", POLICY_ALLOWLIST)
def test_every_policy_plans_the_same_golden_fixture_set(implementation: str, seed: int) -> None:
    scenario = generate_scenario(seed)
    policy = create_policy(implementation)
    observation = _observation(scenario, state=ControllerState.APPROACH)
    decision = policy.plan(observation, scenario, policy.config)
    assert decision.actions
    assert len(decision.actions) == len(decision.commands)
    assert all(isinstance(action, Action) for action in decision.actions)
    assert decision.next_state is ControllerState.GRASP
    assert not decision.abort
    assert not decision.regrasp
    assert is_allowed_transition(ControllerState.APPROACH, decision.next_state)
    again = policy.plan(observation, scenario, policy.config)
    assert again.to_checksum_payload() == decision.to_checksum_payload()


@pytest.mark.parametrize("implementation", POLICY_ALLOWLIST)
def test_happy_path_uses_only_allowed_transitions(implementation: str) -> None:
    scenario = generate_scenario(GOLDEN_SEEDS[0])
    policy = create_policy(implementation)
    state = ControllerState.RESET
    time_seconds = 0.0
    visited: list[ControllerState] = [state]
    for _ in range(20):
        observation = _observation(
            scenario,
            state=state,
            grasp_verified=True,
            simulation_time_seconds=time_seconds,
        )
        decision = policy.plan(observation, scenario, policy.config)
        assert not decision.regrasp
        assert not decision.abort
        if decision.next_state is state:
            break
        assert is_allowed_transition(state, decision.next_state)
        state = decision.next_state
        visited.append(state)
        time_seconds += 0.1
    assert visited == list(_HAPPY_STATES) + [ControllerState.TERMINAL]
    assert state is ControllerState.TERMINAL


def test_fixed_uses_scenario_pose_and_nominal_height() -> None:
    scenario = generate_scenario(GOLDEN_SEEDS[0])
    live = _shifted_pose(scenario)
    observation = _observation(scenario, state=ControllerState.APPROACH, live_pose=live)
    policy = FixedPolicy()
    decision = policy.plan(observation, scenario, policy.config)
    first_move = _moves(decision)[0]
    commanded_xy = _world_xy(first_move)
    assert commanded_xy[0] == pytest.approx(scenario.initial_pose.position_meters.x)
    assert commanded_xy[1] == pytest.approx(scenario.initial_pose.position_meters.y)
    expected_z = (
        policy.config.table_top_z_meters
        + policy.config.nominal_object_half_height_meters
        + policy.config.grasp_tool_offset_meters
        + policy.config.approach_clearance_meters
    )
    assert first_move.target_pose is not None
    assert first_move.target_pose.position_meters.z == expected_z


def test_pose_aware_uses_live_pose_adaptive_height_and_centered_place() -> None:
    scenario = generate_scenario(GOLDEN_SEEDS[0])
    live = _shifted_pose(scenario)
    observation = _observation(scenario, state=ControllerState.APPROACH, live_pose=live)
    policy = PoseAwarePolicy()
    approach = policy.plan(observation, scenario, policy.config)
    first_move = _moves(approach)[0]
    commanded_xy = _world_xy(first_move)
    assert commanded_xy[0] == pytest.approx(live.position_meters.x)
    assert commanded_xy[1] == pytest.approx(live.position_meters.y)
    expected_z = (
        live.position_meters.z
        + policy.config.grasp_tool_offset_meters
        + policy.config.approach_clearance_meters
    )
    assert first_move.target_pose is not None
    assert first_move.target_pose.position_meters.z == expected_z

    lift = policy.plan(
        _observation(scenario, state=ControllerState.LIFT, live_pose=live),
        scenario,
        policy.config,
    )
    transfer = policy.plan(
        _observation(scenario, state=ControllerState.TRANSFER, live_pose=live),
        scenario,
        policy.config,
    )
    fixed = FixedPolicy()
    fixed_lift = FixedPolicy().plan(
        _observation(scenario, state=ControllerState.LIFT, live_pose=live),
        scenario,
        fixed.config,
    )
    lift_pose = _moves(lift)[0].target_pose
    fixed_lift_pose = _moves(fixed_lift)[0].target_pose
    assert lift_pose is not None
    assert fixed_lift_pose is not None
    assert lift_pose.position_meters.z > fixed_lift_pose.position_meters.z
    place_xy = _world_xy(_moves(transfer)[-1])
    assert place_xy[0] == pytest.approx(scenario.target_bin_pose.position_meters.x)
    assert place_xy[1] == pytest.approx(scenario.target_bin_pose.position_meters.y)


def test_collision_aware_adds_waypoints_preflight_abort_and_one_regrasp() -> None:
    scenario = generate_scenario(GOLDEN_SEEDS[0])
    collision = CollisionAwarePolicy()
    pose_aware = PoseAwarePolicy()
    approach = collision.plan(
        _observation(scenario, state=ControllerState.APPROACH),
        scenario,
        collision.config,
    )
    baseline = pose_aware.plan(
        _observation(scenario, state=ControllerState.APPROACH),
        scenario,
        pose_aware.config,
    )
    assert len(_moves(approach)) > len(_moves(baseline))
    transfer = collision.plan(
        _observation(scenario, state=ControllerState.TRANSFER),
        scenario,
        collision.config,
    )
    baseline_transfer = pose_aware.plan(
        _observation(scenario, state=ControllerState.TRANSFER),
        scenario,
        pose_aware.config,
    )
    assert len(_moves(transfer)) > len(_moves(baseline_transfer))
    release = collision.plan(
        _observation(scenario, state=ControllerState.RELEASE),
        scenario,
        collision.config,
    )
    assert tuple(command.primitive for command in release.commands) == (
        MotionPrimitive.SETTLE,
        MotionPrimitive.OPEN,
        MotionPrimitive.SETTLE,
    )

    unreachable = ReachabilityAssessment(
        approach=False, grasp=True, lift=True, place=True, staged_waypoints=()
    )
    preflight = collision.plan(
        _observation(scenario, state=ControllerState.PLAN, reachability=unreachable),
        scenario,
        collision.config,
    )
    assert preflight.abort
    assert preflight.reason == "preflight_unreachable"
    assert preflight.next_state is ControllerState.RETRACT
    ignored = pose_aware.plan(
        _observation(scenario, state=ControllerState.PLAN, reachability=unreachable),
        scenario,
        pose_aware.config,
    )
    assert not ignored.abort
    assert ignored.next_state is ControllerState.APPROACH

    contact = collision.plan(
        _observation(scenario, state=ControllerState.TRANSFER, collision_detected=True),
        scenario,
        collision.config,
    )
    assert contact.abort
    assert contact.reason == "prohibited_contact"
    assert contact.next_state is ControllerState.RETRACT

    first = collision.plan(
        _observation(scenario, state=ControllerState.VERIFY_GRASP, regrasp_count=0),
        scenario,
        collision.config,
    )
    assert first.regrasp
    assert not first.abort
    assert first.reason == "regrasp:grasp_unverified"
    assert first.next_state is ControllerState.APPROACH
    assert is_allowed_transition(ControllerState.VERIFY_GRASP, first.next_state)

    second = collision.plan(
        _observation(scenario, state=ControllerState.VERIFY_GRASP, regrasp_count=1),
        scenario,
        collision.config,
    )
    assert second.abort
    assert not second.regrasp
    assert second.reason == "regrasp_budget_exhausted"

    missed = FixedPolicy().plan(
        _observation(scenario, state=ControllerState.VERIFY_GRASP, regrasp_count=0),
        scenario,
        FixedPolicy().config,
    )
    assert missed.abort
    assert not missed.regrasp
    assert missed.reason == "grasp_unverified"


def test_config_checksum_is_canonical_and_differs_across_policies() -> None:
    first = default_fixed_policy_config()
    second = default_fixed_policy_config()
    assert first.canonical_json() == second.canonical_json()
    assert policy_config_checksum(first) == first.sha256_hex()
    assert len(first.sha256_hex()) == 64
    assert first.sha256_hex() == first.sha256_hex().lower()
    assert first.sha256_hex() != default_pose_aware_policy_config().sha256_hex()
    assert first.sha256_hex() != default_config_for(POLICY_COLLISION_AWARE).sha256_hex()
    version = make_policy_version(POLICY_FIXED, first)
    assert version.config_checksum == first.sha256_hex()
    with pytest.raises(ValueError, match="checksum"):
        PolicyVersion(
            id=version.id,
            name=version.name,
            implementation=version.implementation,
            semantic_version=version.semantic_version,
            description=version.description,
            config=first,
            config_checksum="0" * 64,
        )


def test_registry_prevents_mutation_after_active_or_completed_experiments() -> None:
    registry = default_policy_registry()
    assert tuple(version.implementation for version in registry.versions()) == POLICY_ALLOWLIST
    experiment = new_id()
    registry.reference_experiment(experiment, (POLICY_FIXED,), ExperimentStatus.DRAFT)
    unlocked = replace(registry.get(POLICY_FIXED).config, lift_clearance_meters=0.15)
    updated = registry.replace_config(POLICY_FIXED, unlocked)
    assert updated.config.lift_clearance_meters == 0.15

    registry.set_experiment_status(experiment, ExperimentStatus.QUEUED)
    with pytest.raises(PolicyImmutableError, match="immutable"):
        registry.replace_config(POLICY_FIXED, replace(updated.config, lift_clearance_meters=0.16))
    still_open = replace(registry.get(POLICY_POSE_AWARE).config, lift_clearance_meters=0.19)
    registry.replace_config(POLICY_POSE_AWARE, still_open)

    registry.set_experiment_status(experiment, ExperimentStatus.CANCELLED)
    registry.replace_config(
        POLICY_FIXED, replace(registry.get(POLICY_FIXED).config, lift_clearance_meters=0.16)
    )

    completed = new_id()
    registry.reference_experiment(
        completed, (POLICY_POSE_AWARE, POLICY_COLLISION_AWARE), ExperimentStatus.COMPLETED
    )
    with pytest.raises(PolicyImmutableError, match="immutable"):
        registry.replace_config(
            POLICY_POSE_AWARE,
            replace(registry.get(POLICY_POSE_AWARE).config, lift_clearance_meters=0.17),
        )
    with pytest.raises(PolicyImmutableError, match="immutable"):
        registry.replace_config(
            POLICY_COLLISION_AWARE,
            replace(registry.get(POLICY_COLLISION_AWARE).config, via_clearance_meters=0.07),
        )

    errored = new_id()
    registry.reference_experiment(errored, (POLICY_FIXED,), ExperimentStatus.COMPLETED_WITH_ERRORS)
    with pytest.raises(PolicyImmutableError, match="immutable"):
        registry.replace_config(
            POLICY_FIXED, replace(registry.get(POLICY_FIXED).config, lift_clearance_meters=0.18)
        )

    failed = new_id()
    registry.reference_experiment(failed, (POLICY_FIXED,), ExperimentStatus.FAILED)
    # Another locking reference from `errored` still holds fixed.
    # Register a fresh registry for the failed-only case.
    isolated = default_policy_registry()
    isolated.reference_experiment(new_id(), (POLICY_FIXED,), ExperimentStatus.FAILED)
    isolated.replace_config(
        POLICY_FIXED, replace(isolated.get(POLICY_FIXED).config, lift_clearance_meters=0.15)
    )


def test_collision_aware_rejects_configs_without_one_regrasp() -> None:
    with pytest.raises(ValueError, match="regrasp_limit"):
        CollisionAwarePolicy(default_fixed_policy_config())
    with pytest.raises(ValueError, match="regrasp_limit"):
        FixedPolicy(default_config_for(POLICY_COLLISION_AWARE))
