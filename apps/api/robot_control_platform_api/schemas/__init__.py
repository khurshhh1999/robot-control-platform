"""Shared Pydantic schema helpers and pagination envelopes."""

from __future__ import annotations

from datetime import datetime
from typing import Any
from uuid import UUID

from pydantic import BaseModel, ConfigDict, Field
from robot_control_platform_common.time import to_iso8601_z


class ApiModel(BaseModel):
    """Base response/request model with strict configuration."""

    model_config = ConfigDict(
        extra="forbid",
        frozen=True,
        from_attributes=False,
    )


def serialize_datetime(value: datetime | None) -> str | None:
    if value is None:
        return None
    return to_iso8601_z(value)


def serialize_uuid(value: UUID) -> str:
    return str(value)


class PageMeta(ApiModel):
    """Cursor pagination metadata."""

    next_cursor: str | None = Field(
        default=None,
        description="Opaque cursor for the next page, or null when exhausted.",
        examples=[None],
    )
    page_size: int = Field(
        description="Number of items requested for this page.",
        examples=[25],
    )


class CursorPage(ApiModel):
    """Generic cursor page envelope."""

    items: list[Any]
    page: PageMeta
