"""Experiment API schemas."""

from __future__ import annotations

from pydantic import Field, field_validator

from robot_control_platform_api.schemas import ApiModel


class ExperimentCreate(ApiModel):
    """Create an experiment from an existing scenario set and policies."""

    name: str = Field(min_length=1, max_length=200, examples=["demo-experiment"])
    description: str | None = Field(default=None, examples=["Neutral smoke evaluation"])
    scenario_set_id: str = Field(examples=["0193f1a2-b3c4-7d8e-9f01-23456789abcd"])
    policy_version_ids: list[str] = Field(
        min_length=1,
        examples=[["0193f1a2-b3c4-7d8e-9f01-23456789abce"]],
    )
    requested_by: str | None = Field(default=None, examples=["reviewer"])
    source_revision: str = Field(min_length=1, examples=["deadbeef"])
    simulator_image_digest: str = Field(min_length=1, examples=["sha256:" + "c" * 64])

    @field_validator("policy_version_ids")
    @classmethod
    def _unique_policies(cls, value: list[str]) -> list[str]:
        if len(set(value)) != len(value):
            msg = "policy_version_ids must be unique"
            raise ValueError(msg)
        return value


class ExperimentPolicyResponse(ApiModel):
    """Ordered policy membership for an experiment."""

    policy_version_id: str
    execution_order: int


class ExperimentResponse(ApiModel):
    """Experiment lifecycle and membership summary."""

    id: str = Field(examples=["0193f1a2-b3c4-7d8e-9f01-23456789abcd"])
    name: str
    description: str | None = None
    scenario_set_id: str
    status: str = Field(examples=["draft"])
    requested_by: str | None = None
    created_at: str
    started_at: str | None = None
    completed_at: str | None = None
    source_revision: str
    simulator_image_digest: str
    policies: list[ExperimentPolicyResponse]


class ExperimentListResponse(ApiModel):
    """Collection of experiments."""

    items: list[ExperimentResponse]
