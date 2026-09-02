"""Collision-aware policy: reachability preflight, staged waypoints, one re-grasp."""

from __future__ import annotations

from robot_control_platform_simulator.control.actions import ActionStatus
from robot_control_platform_simulator.domain.enums import ControllerState
from robot_control_platform_simulator.policies.base import (
    POLICY_COLLISION_AWARE,
    PolicyConfig,
    PolicyObservation,
    default_collision_aware_policy_config,
)
from robot_control_platform_simulator.policies.pose_aware import PoseAwarePolicy
from robot_control_platform_simulator.scenarios.generator import Scenario


class CollisionAwarePolicy(PoseAwarePolicy):
    """v3_collision_aware: staged motion, contact abort, and exactly one re-grasp."""

    @classmethod
    def default_config(cls) -> PolicyConfig:
        return default_collision_aware_policy_config()

    @property
    def implementation(self) -> str:
        return POLICY_COLLISION_AWARE

    def _validate_config(self, config: PolicyConfig) -> None:
        if config.regrasp_limit != 1:
            msg = "collision_aware regrasp_limit must be 1"
            raise ValueError(msg)
        if config.approach_stage_count < 1:
            msg = "collision_aware approach_stage_count must be at least 1"
            raise ValueError(msg)
        if config.transfer_stage_count < 1:
            msg = "collision_aware transfer_stage_count must be at least 1"
            raise ValueError(msg)
        if config.release_settle_count < 2:
            msg = "collision_aware release_settle_count must be at least 2"
            raise ValueError(msg)

    def abort_reason(
        self,
        observation: PolicyObservation,
        scenario: Scenario,
        config: PolicyConfig,
    ) -> str | None:
        _ = (scenario, config)
        state = observation.controller_state
        if state in {ControllerState.RETRACT, ControllerState.TERMINAL}:
            return None
        if observation.collision_detected:
            return "prohibited_contact"
        if observation.last_action_status is ActionStatus.IK_REJECTED:
            return "preflight_unreachable"
        if state is ControllerState.PLAN and not observation.reachability.required_reachable():
            return "preflight_unreachable"
        return None
