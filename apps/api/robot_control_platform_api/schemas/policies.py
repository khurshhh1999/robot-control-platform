"""Policy version API schemas."""

from __future__ import annotations

from typing import Any

from pydantic import Field

from robot_control_platform_api.schemas import ApiModel


class PolicyVersionResponse(ApiModel):
    """Immutable allowlisted policy version."""

    id: str = Field(examples=["0193f1a2-b3c4-7d8e-9f01-23456789abcd"])
    name: str = Field(examples=["fixed"])
    semantic_version: str = Field(examples=["1.0.0"])
    description: str | None = Field(default=None, examples=["Top-down fixed approach"])
    config: dict[str, Any] = Field(examples=[{"approach_height_m": 0.15}])
    config_sha256: str = Field(examples=["a" * 64])
    source_revision: str = Field(examples=["deadbeef"])
    container_image_digest: str = Field(examples=["sha256:" + "b" * 64])
    created_at: str = Field(examples=["2026-01-15T12:00:00.000000Z"])


class PolicyVersionListResponse(ApiModel):
    """Collection of policy versions."""

    items: list[PolicyVersionResponse] = Field(examples=[[]])
