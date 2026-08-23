"""Structured JSON logging with correlation IDs and secret redaction."""

from __future__ import annotations

import logging as stdlib_logging
import sys
from collections.abc import Mapping, Sequence
from datetime import datetime
from pathlib import Path
from typing import cast
from uuid import UUID

import structlog
from pydantic import SecretStr
from structlog.typing import EventDict, WrappedLogger

from robot_control_platform_common.time import to_iso8601_z, utc_now

_REDACTED = "[REDACTED]"
_REDACT_FRAGMENTS = (
    "password",
    "secret",
    "token",
    "authorization",
    "cookie",
    "dsn",
)
_CORRELATION_KEYS = ("request_id", "experiment_id", "run_id", "trial_id")
_LEVELS = {
    "DEBUG": stdlib_logging.DEBUG,
    "INFO": stdlib_logging.INFO,
    "WARNING": stdlib_logging.WARNING,
    "ERROR": stdlib_logging.ERROR,
}

_service_name = "unknown"


def _parse_level(level: str) -> int:
    try:
        return _LEVELS[level.upper()]
    except KeyError as exc:
        msg = "Unsupported log level; use DEBUG, INFO, WARNING, or ERROR"
        raise ValueError(msg) from exc


def _key_should_redact(key: object) -> bool:
    lowered = str(key).lower()
    return any(fragment in lowered for fragment in _REDACT_FRAGMENTS)


def _sanitize(value: object, *, redact: bool = False) -> object:
    if redact or isinstance(value, SecretStr):
        return _REDACTED
    if isinstance(value, Mapping):
        return {
            str(key): _sanitize(item, redact=_key_should_redact(key)) for key, item in value.items()
        }
    if isinstance(value, tuple):
        return tuple(_sanitize(item) for item in value)
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        return [_sanitize(item) for item in value]
    if isinstance(value, UUID):
        return str(value)
    if isinstance(value, datetime):
        if value.tzinfo is None:
            return value.isoformat()
        return to_iso8601_z(value)
    if isinstance(value, Path):
        return str(value)
    return value


def _add_service(_logger: WrappedLogger, _method: str, event_dict: EventDict) -> EventDict:
    event_dict.setdefault("service", _service_name)
    return event_dict


def _ensure_correlation_ids(
    _logger: WrappedLogger, _method: str, event_dict: EventDict
) -> EventDict:
    for key in _CORRELATION_KEYS:
        event_dict.setdefault(key, None)
    return event_dict


def _add_utc_timestamp(_logger: WrappedLogger, _method: str, event_dict: EventDict) -> EventDict:
    event_dict["timestamp"] = to_iso8601_z(utc_now())
    return event_dict


def _redact_event_dict(_logger: WrappedLogger, _method: str, event_dict: EventDict) -> EventDict:
    sanitized = _sanitize(event_dict)
    if not isinstance(sanitized, dict):
        msg = "log event must serialize to an object"
        raise TypeError(msg)
    return cast(EventDict, sanitized)


def configure_logging(service_name: str, level: str) -> None:
    """Configure process-wide JSON logging to stdout.

    Bound ``request_id``, ``experiment_id``, ``run_id``, and ``trial_id``
    values are merged into every event. Keys whose names contain password,
    secret, token, authorization, cookie, or DSN are redacted.
    """

    if service_name.strip() == "":
        msg = "service_name must be non-empty"
        raise ValueError(msg)

    global _service_name
    _service_name = service_name
    numeric_level = _parse_level(level)

    stdlib_logging.basicConfig(
        format="%(message)s",
        stream=sys.stdout,
        level=numeric_level,
        force=True,
    )
    structlog.contextvars.clear_contextvars()
    structlog.configure(
        processors=[
            structlog.contextvars.merge_contextvars,
            _add_service,
            _ensure_correlation_ids,
            structlog.stdlib.add_log_level,
            _add_utc_timestamp,
            _redact_event_dict,
            structlog.processors.JSONRenderer(),
        ],
        wrapper_class=structlog.make_filtering_bound_logger(numeric_level),
        logger_factory=structlog.PrintLoggerFactory(file=sys.stdout),
        cache_logger_on_first_use=False,
    )
    structlog.contextvars.bind_contextvars(service=service_name)


def bind_log_context(
    *,
    request_id: str | None = None,
    experiment_id: str | None = None,
    run_id: str | None = None,
    trial_id: str | None = None,
) -> None:
    """Bind request, experiment, run, and trial identifiers for later events."""

    bound: dict[str, str] = {}
    if request_id is not None:
        bound["request_id"] = request_id
    if experiment_id is not None:
        bound["experiment_id"] = experiment_id
    if run_id is not None:
        bound["run_id"] = run_id
    if trial_id is not None:
        bound["trial_id"] = trial_id
    if bound:
        structlog.contextvars.bind_contextvars(**bound)


def clear_log_context() -> None:
    """Clear bound correlation identifiers. The service name remains set."""

    structlog.contextvars.clear_contextvars()
    structlog.contextvars.bind_contextvars(service=_service_name)


def get_logger(name: str | None = None) -> structlog.stdlib.BoundLogger:
    """Return the process logger configured by :func:`configure_logging`."""

    logger = structlog.get_logger(name)
    return cast(structlog.stdlib.BoundLogger, logger)
