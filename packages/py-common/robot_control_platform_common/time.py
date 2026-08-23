"""Aware UTC clocks and JSON timestamp formatting."""

from __future__ import annotations

from datetime import UTC, datetime, timedelta

_ZERO = timedelta(0)


def utc_now() -> datetime:
    """Return the current timezone-aware UTC datetime."""

    now = datetime.now(tz=UTC)
    if now.tzinfo is None or now.utcoffset() != _ZERO:
        msg = "utc_now() must return an aware UTC datetime"
        raise RuntimeError(msg)
    return now


def to_iso8601_z(value: datetime) -> str:
    """Serialize an aware datetime as UTC ISO 8601 ending in ``Z``."""

    if value.tzinfo is None:
        msg = "datetime must be timezone-aware UTC"
        raise ValueError(msg)
    if value.utcoffset() is None:
        msg = "datetime timezone offset must be defined"
        raise ValueError(msg)
    utc_value = value.astimezone(UTC)
    return utc_value.replace(tzinfo=None).isoformat(timespec="microseconds") + "Z"
