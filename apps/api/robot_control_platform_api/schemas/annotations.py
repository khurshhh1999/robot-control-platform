"""Annotation API schemas."""

from __future__ import annotations

from pydantic import Field

from robot_control_platform_api.schemas import ApiModel


class AnnotationCreate(ApiModel):
    """Create a reviewer annotation on a trial."""

    label: str = Field(min_length=1, max_length=100, examples=["needs_review"])
    note: str | None = Field(default=None, examples=["Gripper closed early"])
    reviewer: str = Field(min_length=1, max_length=100, examples=["reviewer"])


class AnnotationUpdate(ApiModel):
    """Update an annotation with optimistic concurrency."""

    label: str = Field(min_length=1, max_length=100, examples=["confirmed_collision"])
    note: str | None = Field(default=None, examples=["Updated note"])
    revision: int = Field(
        ge=1,
        description="Current revision that must match the stored value.",
        examples=[1],
    )


class AnnotationResponse(ApiModel):
    """Reviewer annotation with revision history counter."""

    id: str = Field(examples=["0193f1a2-b3c4-7d8e-9f01-23456789abcd"])
    trial_id: str
    label: str
    note: str | None = None
    reviewer: str
    created_at: str
    updated_at: str
    revision: int = Field(examples=[1])
