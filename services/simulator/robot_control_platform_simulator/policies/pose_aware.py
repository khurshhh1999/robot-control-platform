"""Pose-aware policy: live object pose, adaptive grasp height, safer lift, centered place."""

from __future__ import annotations

from robot_control_platform_simulator.policies.base import (
    POLICY_POSE_AWARE,
    BasePolicy,
    PolicyConfig,
    default_pose_aware_policy_config,
)


class PoseAwarePolicy(BasePolicy):
    """v2_pose_aware: uses the observed object pose and places at the bin center."""

    @classmethod
    def default_config(cls) -> PolicyConfig:
        return default_pose_aware_policy_config()

    @property
    def implementation(self) -> str:
        return POLICY_POSE_AWARE

    def uses_live_object_pose(self) -> bool:
        return True
