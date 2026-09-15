"""Trial and trial-event API schemas."""

from __future__ import annotations

from typing import Any

from pydantic import Field

from robot_control_platform_api.schemas import ApiModel, PageMeta


class TrialResponse(ApiModel):
    """Scenario-policy trial execution record."""

    id: str = Field(examples=["0193f1a2-b3c4-7d8e-9f01-23456789abcd"])
    experiment_id: str
    policy_version_id: str
    scenario_id: str
    status: str = Field(examples=["completed"])
    terminal_outcome: str | None = Field(default=None, examples=["success"])
    success: bool | None = Field(default=None, examples=[True])
    collision_count: int | None = None
    collision_max_force_newtons: str | None = Field(
        default=None,
        description="Maximum collision force in newtons as a decimal string.",
        examples=["12.500000"],
    )
    duration_seconds: str | None = Field(
        default=None,
        description="Trial duration in seconds as a decimal string.",
        examples=["4.250000"],
    )
    action_count: int | None = None
    started_at: str | None = None
    completed_at: str | None = None
    simulator_metadata: dict[str, Any] | None = None


class TrialListResponse(ApiModel):
    """Cursor-paginated trial collection."""

    items: list[TrialResponse]
    page: PageMeta


class TrialEventResponse(ApiModel):
    """Ordered trial event."""

    id: int
    trial_id: str
    ordinal: int
    timestamp_offset_seconds: str
    event_type: str
    controller_state: str | None = None
    action: dict[str, Any] | None = None
    observation: dict[str, Any] | None = None


class TrialEventListResponse(ApiModel):
    """Cursor-paginated trial event collection."""

    items: list[TrialEventResponse]
    page: PageMeta
