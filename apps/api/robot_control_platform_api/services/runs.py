"""Run service rules including idempotent start and cancellation."""

from __future__ import annotations

import hashlib
from uuid import UUID

from robot_control_platform_common.db.repositories import experiments as experiment_repo
from robot_control_platform_common.db.repositories import runs as run_repo
from sqlalchemy.ext.asyncio import AsyncSession

from robot_control_platform_api.errors import ApiError
from robot_control_platform_api.schemas.runs import RunResponse
from robot_control_platform_api.services.serializers import run_response


def _parse_uuid(value: str, *, field_name: str) -> UUID:
    try:
        return UUID(value)
    except ValueError as exc:
        raise ApiError(
            "VALIDATION_ERROR",
            status=422,
            detail=f"{field_name} must be a valid UUID",
        ) from exc


def fingerprint_request_body(raw_body: bytes) -> str:
    """Return SHA-256 of the raw request body for idempotency checks."""

    return hashlib.sha256(raw_body).hexdigest()


async def start_run(
    session: AsyncSession,
    *,
    experiment_id: str,
    idempotency_key: str,
    request_fingerprint: str,
) -> tuple[RunResponse, bool]:
    """Queue a run for an experiment.

    Returns ``(response, created)``. Missing idempotency keys are rejected by
    the route layer before this service is called.
    """

    if not idempotency_key.strip():
        raise ApiError(
            "VALIDATION_ERROR",
            status=422,
            detail="Idempotency-Key must be a non-empty string",
        )

    experiment_uuid = _parse_uuid(experiment_id, field_name="experiment_id")
    experiment = await experiment_repo.get_experiment(session, experiment_uuid)
    if experiment.status in {"cancelled", "failed"}:
        raise ApiError(
            "INVALID_STATE_TRANSITION",
            status=409,
            detail="Cannot start a run for a cancelled or failed experiment",
        )

    run, created = await run_repo.create_or_get_queued_run(
        session,
        experiment_id=experiment_uuid,
        idempotency_key=idempotency_key.strip(),
        request_fingerprint=request_fingerprint,
    )
    await experiment_repo.sync_experiment_status(session, experiment_uuid)
    return run_response(run), created


async def get_run(session: AsyncSession, run_id: str) -> RunResponse:
    identifier = _parse_uuid(run_id, field_name="run_id")
    run = await run_repo.get_run(session, identifier)
    return run_response(run)


async def cancel_run(session: AsyncSession, run_id: str) -> RunResponse:
    identifier = _parse_uuid(run_id, field_name="run_id")
    run = await run_repo.request_run_cancellation(session, identifier)
    await experiment_repo.sync_experiment_status(session, run.experiment_id)
    return run_response(run)
