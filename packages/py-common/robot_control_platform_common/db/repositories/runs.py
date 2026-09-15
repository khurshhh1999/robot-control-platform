"""Run queue persistence with transactional lease claiming.

Claiming uses ``SELECT ... FOR UPDATE SKIP LOCKED`` so concurrent workers never
share the same queued row. Lease heartbeats succeed only for the owning worker
on a nonterminal run. Expired leases are requeued while attempts remain below
the caller-supplied limit; otherwise the run fails without deleting trials.
"""

from __future__ import annotations

from datetime import datetime, timedelta
from uuid import UUID

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from robot_control_platform_common.db.models import Run
from robot_control_platform_common.db.repositories.exceptions import (
    EntityNotFoundError,
    IdempotencyConflictError,
    InvalidLeaseStateError,
    LeaseOwnershipError,
    RunAlreadyTerminalError,
)
from robot_control_platform_common.ids import new_id
from robot_control_platform_common.time import utc_now

_CLAIMABLE_STATUSES = frozenset({"queued"})
_LEASED_STATUSES = frozenset({"claimed", "running"})
_HEARTBEAT_STATUSES = frozenset({"claimed", "running", "cancelling"})
_TERMINAL_RUN_STATUSES = frozenset({"completed", "completed_with_errors", "cancelled", "failed"})
_SHA256_HEX_LENGTH = 64


async def add_run(session: AsyncSession, run: Run) -> Run:
    """Persist a new run row."""

    session.add(run)
    await session.flush()
    return run


async def get_run(session: AsyncSession, run_id: UUID) -> Run:
    """Return a run by id or raise ``EntityNotFoundError``."""

    run = await session.get(Run, run_id)
    if run is None:
        msg = f"run {run_id} not found"
        raise EntityNotFoundError(msg)
    return run


def _validate_fingerprint(request_fingerprint: str) -> str:
    fingerprint = request_fingerprint.lower()
    if len(fingerprint) != _SHA256_HEX_LENGTH or any(
        ch not in "0123456789abcdef" for ch in fingerprint
    ):
        msg = "request_fingerprint must be lowercase hexadecimal of length 64"
        raise ValueError(msg)
    return fingerprint


async def create_queued_run(
    session: AsyncSession,
    *,
    experiment_id: UUID,
    idempotency_key: str,
    request_fingerprint: str,
    created_at: datetime | None = None,
) -> Run:
    """Insert a queued run for an experiment with a request fingerprint."""

    fingerprint = _validate_fingerprint(request_fingerprint)
    if not idempotency_key.strip():
        msg = "idempotency_key must be non-empty"
        raise ValueError(msg)

    run = Run(
        id=new_id(),
        experiment_id=experiment_id,
        status="queued",
        idempotency_key=idempotency_key,
        request_fingerprint=fingerprint,
        lease_owner=None,
        lease_expires_at=None,
        attempt=0,
        error_detail=None,
        created_at=created_at or utc_now(),
        started_at=None,
        completed_at=None,
    )
    return await add_run(session, run)


async def create_or_get_queued_run(
    session: AsyncSession,
    *,
    experiment_id: UUID,
    idempotency_key: str,
    request_fingerprint: str,
    created_at: datetime | None = None,
) -> tuple[Run, bool]:
    """Create a queued run or return the existing idempotent match.

    Returns ``(run, created)``. Reusing the same key with a different request
    fingerprint raises ``IdempotencyConflictError``.
    """

    fingerprint = _validate_fingerprint(request_fingerprint)
    existing = await get_run_by_idempotency_key(
        session,
        experiment_id=experiment_id,
        idempotency_key=idempotency_key,
    )
    if existing is not None:
        if existing.request_fingerprint != fingerprint:
            msg = "idempotency key reused with a different request body"
            raise IdempotencyConflictError(msg)
        return existing, False

    run = await create_queued_run(
        session,
        experiment_id=experiment_id,
        idempotency_key=idempotency_key,
        request_fingerprint=fingerprint,
        created_at=created_at,
    )
    return run, True


async def request_run_cancellation(session: AsyncSession, run_id: UUID) -> Run:
    """Request cancellation for a nonterminal run.

    Queued runs transition directly to ``cancelled``. Claimed or running runs
    transition to ``cancelling`` for the worker to observe. Already-cancelling
    runs are returned unchanged. Terminal runs raise ``RunAlreadyTerminalError``.
    """

    result = await session.execute(select(Run).where(Run.id == run_id).with_for_update())
    run = result.scalar_one_or_none()
    if run is None:
        msg = f"run {run_id} not found"
        raise EntityNotFoundError(msg)

    if run.status in _TERMINAL_RUN_STATUSES:
        msg = f"run {run_id} is already terminal"
        raise RunAlreadyTerminalError(msg)
    if run.status == "cancelling":
        return run
    if run.status in {"queued", "lease_expired"}:
        moment = utc_now()
        run.status = "cancelled"
        run.lease_owner = None
        run.lease_expires_at = None
        run.completed_at = moment
        await session.flush()
        return run
    if run.status in {"claimed", "running"}:
        run.status = "cancelling"
        await session.flush()
        return run

    msg = f"run {run_id} cannot be cancelled from status {run.status}"
    raise InvalidLeaseStateError(msg)


async def requeue_expired_leases(
    session: AsyncSession,
    *,
    max_attempts: int,
    now: datetime | None = None,
) -> list[Run]:
    """Requeue or fail leased runs whose lease has expired.

    Rows are locked with ``SKIP LOCKED`` so concurrent reclaimers do not fight
    over the same expired lease. Completed trials are never modified.
    """

    if max_attempts < 1:
        msg = "max_attempts must be at least 1"
        raise ValueError(msg)

    moment = now or utc_now()
    result = await session.execute(
        select(Run)
        .where(
            Run.status.in_(sorted(_LEASED_STATUSES)),
            Run.lease_expires_at.is_not(None),
            Run.lease_expires_at <= moment,
        )
        .order_by(Run.lease_expires_at, Run.created_at)
        .with_for_update(skip_locked=True)
    )
    expired = list(result.scalars().all())
    changed: list[Run] = []
    for run in expired:
        # Record the lease_expired transition before either requeue or failure.
        run.status = "lease_expired"
        run.lease_owner = None
        run.lease_expires_at = None
        if run.attempt < max_attempts:
            run.status = "queued"
            run.error_detail = None
        else:
            run.status = "failed"
            run.completed_at = moment
            run.error_detail = "run lease retries exhausted"
        changed.append(run)
    if changed:
        await session.flush()
    return changed


async def claim_next_run(
    session: AsyncSession,
    *,
    worker_id: str,
    lease_seconds: int,
    max_attempts: int,
    now: datetime | None = None,
) -> Run | None:
    """Claim the oldest available queued run for ``worker_id``.

    Expired leases are reclaimed first. The selected queued row is locked with
    ``FOR UPDATE SKIP LOCKED``, then receives lease ownership, expiry, and an
    incremented attempt counter.
    """

    if not worker_id.strip():
        msg = "worker_id must be non-empty"
        raise ValueError(msg)
    if lease_seconds < 1:
        msg = "lease_seconds must be at least 1"
        raise ValueError(msg)

    moment = now or utc_now()
    await requeue_expired_leases(session, max_attempts=max_attempts, now=moment)

    result = await session.execute(
        select(Run)
        .where(Run.status.in_(sorted(_CLAIMABLE_STATUSES)))
        .order_by(Run.created_at, Run.id)
        .limit(1)
        .with_for_update(skip_locked=True)
    )
    run = result.scalar_one_or_none()
    if run is None:
        return None

    run.status = "claimed"
    run.lease_owner = worker_id
    run.lease_expires_at = moment + timedelta(seconds=lease_seconds)
    run.attempt = run.attempt + 1
    if run.started_at is None:
        run.started_at = moment
    await session.flush()
    return run


async def heartbeat_run(
    session: AsyncSession,
    *,
    run_id: UUID,
    worker_id: str,
    lease_seconds: int,
    now: datetime | None = None,
) -> Run:
    """Extend a lease only when ``worker_id`` owns a nonterminal run lease."""

    if lease_seconds < 1:
        msg = "lease_seconds must be at least 1"
        raise ValueError(msg)

    moment = now or utc_now()
    result = await session.execute(select(Run).where(Run.id == run_id).with_for_update())
    run = result.scalar_one_or_none()
    if run is None:
        msg = f"run {run_id} not found"
        raise EntityNotFoundError(msg)

    if run.status in _TERMINAL_RUN_STATUSES or run.status == "lease_expired":
        msg = f"run {run_id} is not in a heartbeat-eligible state"
        raise InvalidLeaseStateError(msg)
    if run.status not in _HEARTBEAT_STATUSES:
        msg = f"run {run_id} is not in a heartbeat-eligible state"
        raise InvalidLeaseStateError(msg)
    if run.lease_owner != worker_id:
        msg = f"worker {worker_id!r} does not own the lease for run {run_id}"
        raise LeaseOwnershipError(msg)

    run.lease_expires_at = moment + timedelta(seconds=lease_seconds)
    await session.flush()
    return run


async def mark_run_running(session: AsyncSession, run_id: UUID) -> Run:
    """Transition a claimed run to ``running``."""

    run = await get_run(session, run_id)
    if run.status != "claimed":
        msg = f"run {run_id} must be claimed before running"
        raise InvalidLeaseStateError(msg)
    run.status = "running"
    await session.flush()
    return run


async def complete_run(
    session: AsyncSession,
    *,
    run_id: UUID,
    status: str,
    now: datetime | None = None,
    error_detail: str | None = None,
) -> Run:
    """Mark a run terminal and clear its lease."""

    if status not in {"completed", "completed_with_errors", "cancelled", "failed"}:
        msg = f"unsupported terminal run status: {status}"
        raise ValueError(msg)

    moment = now or utc_now()
    run = await get_run(session, run_id)
    if run.status in _TERMINAL_RUN_STATUSES:
        msg = f"run {run_id} is already terminal"
        raise InvalidLeaseStateError(msg)
    run.status = status
    run.lease_owner = None
    run.lease_expires_at = None
    run.completed_at = moment
    run.error_detail = error_detail
    await session.flush()
    return run


async def get_run_by_idempotency_key(
    session: AsyncSession,
    *,
    experiment_id: UUID,
    idempotency_key: str,
) -> Run | None:
    """Return an existing run for an experiment/idempotency pair, if any."""

    result = await session.execute(
        select(Run).where(
            Run.experiment_id == experiment_id,
            Run.idempotency_key == idempotency_key,
        )
    )
    return result.scalar_one_or_none()


async def list_claimable_or_leased_runs(session: AsyncSession) -> list[Run]:
    """Return queued or leased runs (debug/test helper)."""

    result = await session.execute(
        select(Run)
        .where(Run.status.in_(sorted(_CLAIMABLE_STATUSES | _LEASED_STATUSES)))
        .order_by(Run.created_at)
    )
    return list(result.scalars().all())
