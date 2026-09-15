"""Artifact API schemas."""

from __future__ import annotations

from pydantic import Field

from robot_control_platform_api.schemas import ApiModel


class ArtifactResponse(ApiModel):
    """Checksummed artifact metadata. Bytes are served separately."""

    id: str = Field(examples=["0193f1a2-b3c4-7d8e-9f01-23456789abcd"])
    trial_id: str
    kind: str = Field(examples=["initial_rgb"])
    media_type: str = Field(examples=["image/png"])
    width_px: int | None = Field(default=None, examples=[640])
    height_px: int | None = Field(default=None, examples=[480])
    byte_size: int = Field(examples=[1024])
    sha256: str = Field(examples=["a" * 64])
    created_at: str


class ArtifactListResponse(ApiModel):
    """Artifacts belonging to a trial."""

    items: list[ArtifactResponse]
