"""Fixed-height top-down policy. Does not use live object pose or retry."""

from __future__ import annotations

from robot_control_platform_simulator.policies.base import (
    POLICY_FIXED,
    BasePolicy,
    PolicyConfig,
    default_fixed_policy_config,
)


class FixedPolicy(BasePolicy):
    """v1_fixed: fixed approach height, direct grasp, fixed lift, direct transfer."""

    @classmethod
    def default_config(cls) -> PolicyConfig:
        return default_fixed_policy_config()

    @property
    def implementation(self) -> str:
        return POLICY_FIXED
