"""Opaque base64url cursor helpers for collection pagination."""

from __future__ import annotations

import base64
import json
from typing import Any, Final
from uuid import UUID

from robot_control_platform_api.errors import ApiError

CURSOR_VERSION: Final[int] = 1
DEFAULT_PAGE_SIZE: Final[int] = 25
MAX_PAGE_SIZE: Final[int] = 100


def clamp_page_size(limit: int | None) -> int:
    """Return a page size within the public collection bounds."""

    if limit is None:
        return DEFAULT_PAGE_SIZE
    if limit < 1:
        raise ApiError(
            "VALIDATION_ERROR",
            status=422,
            detail="Page size must be at least 1",
        )
    if limit > MAX_PAGE_SIZE:
        raise ApiError(
            "VALIDATION_ERROR",
            status=422,
            detail=f"Page size must not exceed {MAX_PAGE_SIZE}",
        )
    return limit


def encode_cursor(*, kind: str, sort_values: list[Any]) -> str:
    """Encode an opaque base64url cursor with stable sort values."""

    payload = {"v": CURSOR_VERSION, "k": kind, "s": sort_values}
    raw = json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")
    return base64.urlsafe_b64encode(raw).decode("ascii").rstrip("=")


def decode_cursor(cursor: str, *, expected_kind: str) -> list[Any]:
    """Decode and validate an opaque cursor for ``expected_kind``."""

    if not cursor.strip():
        raise ApiError(
            "VALIDATION_ERROR",
            status=422,
            detail="Cursor must be a non-empty string",
        )
    padded = cursor + "=" * (-len(cursor) % 4)
    try:
        raw = base64.urlsafe_b64decode(padded.encode("ascii"))
        payload = json.loads(raw.decode("utf-8"))
    except (ValueError, json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise ApiError(
            "VALIDATION_ERROR",
            status=422,
            detail="Cursor is invalid",
        ) from exc

    if not isinstance(payload, dict):
        raise ApiError(
            "VALIDATION_ERROR",
            status=422,
            detail="Cursor is invalid",
        )
    if payload.get("v") != CURSOR_VERSION:
        raise ApiError(
            "VALIDATION_ERROR",
            status=422,
            detail="Cursor version is unsupported",
        )
    if payload.get("k") != expected_kind:
        raise ApiError(
            "VALIDATION_ERROR",
            status=422,
            detail="Cursor does not match this collection",
        )
    sort_values = payload.get("s")
    if not isinstance(sort_values, list) or not sort_values:
        raise ApiError(
            "VALIDATION_ERROR",
            status=422,
            detail="Cursor sort values are invalid",
        )
    return sort_values


def parse_uuid_cursor_value(value: Any) -> UUID:
    """Parse a UUID sort value from a decoded cursor."""

    try:
        return UUID(str(value))
    except (TypeError, ValueError) as exc:
        raise ApiError(
            "VALIDATION_ERROR",
            status=422,
            detail="Cursor sort values are invalid",
        ) from exc


def parse_int_cursor_value(value: Any) -> int:
    """Parse an integer sort value from a decoded cursor."""

    if isinstance(value, bool) or not isinstance(value, int):
        raise ApiError(
            "VALIDATION_ERROR",
            status=422,
            detail="Cursor sort values are invalid",
        )
    return value
