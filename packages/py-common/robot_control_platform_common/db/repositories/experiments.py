"""Experiment persistence and status derivation from runs and trials."""

from __future__ import annotations

from collections.abc import Sequence
from uuid import UUID

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from robot_control_platform_common.db.models import Experiment, ExperimentPolicy, Run, Trial
from robot_control_platform_common.db.repositories.exceptions import EntityNotFoundError
from robot_control_platform_common.time import utc_now

_ACTIVE_RUN_STATUSES = frozenset({"claimed", "running", "cancelling"})
_WAITING_RUN_STATUSES = frozenset({"queued", "lease_expired"})
_TERMINAL_RUN_STATUSES = frozenset({"completed", "completed_with_errors", "cancelled", "failed"})
_SUCCESSFUL_RUN_STATUSES = frozenset({"completed", "completed_with_errors"})


def derive_experiment_status(
    *,
    current_status: str,
    runs: Sequence[Run],
    trials: Sequence[Trial],
) -> str:
    """Derive experiment lifecycle status from run and trial rows.

    Active leases keep the experiment ``running``. Waiting queue rows keep it
    ``queued``. Terminal aggregation prefers ``failed`` over ``cancelled`` over
    successful completion, and uses trial ``failed`` / run ``completed_with_errors``
    to distinguish clean completion from ``completed_with_errors``.
    """

    if not runs:
        return current_status

    statuses = {run.status for run in runs}
    if statuses & _ACTIVE_RUN_STATUSES:
        return "running"
    if statuses & _WAITING_RUN_STATUSES:
        return "queued"
    if not statuses <= _TERMINAL_RUN_STATUSES:
        # Unexpected non-terminal leftovers: keep current rather than inventing.
        return current_status

    if "failed" in statuses:
        return "failed"
    if statuses <= {"cancelled"}:
        return "cancelled"
    if statuses & _SUCCESSFUL_RUN_STATUSES:
        if "completed_with_errors" in statuses:
            return "completed_with_errors"
        if any(trial.status == "failed" for trial in trials):
            return "completed_with_errors"
        if "cancelled" in statuses:
            return "completed_with_errors"
        return "completed"
    if "cancelled" in statuses:
        return "cancelled"
    return current_status


async def add_experiment(session: AsyncSession, experiment: Experiment) -> Experiment:
    """Persist a new experiment."""

    session.add(experiment)
    await session.flush()
    return experiment


async def add_experiment_policy(
    session: AsyncSession,
    membership: ExperimentPolicy,
) -> ExperimentPolicy:
    """Persist an experiment/policy membership row."""

    session.add(membership)
    await session.flush()
    return membership


async def get_experiment(session: AsyncSession, experiment_id: UUID) -> Experiment:
    """Return an experiment by id or raise ``EntityNotFoundError``."""

    experiment = await session.get(Experiment, experiment_id)
    if experiment is None:
        msg = f"experiment {experiment_id} not found"
        raise EntityNotFoundError(msg)
    return experiment


async def list_experiments(session: AsyncSession) -> list[Experiment]:
    """Return experiments ordered by creation time descending then id."""

    result = await session.execute(
        select(Experiment).order_by(Experiment.created_at.desc(), Experiment.id.desc())
    )
    return list(result.scalars().all())


async def list_policies_for_experiment(
    session: AsyncSession,
    experiment_id: UUID,
) -> list[ExperimentPolicy]:
    """Return ordered policy memberships for an experiment."""

    result = await session.execute(
        select(ExperimentPolicy)
        .where(ExperimentPolicy.experiment_id == experiment_id)
        .order_by(ExperimentPolicy.execution_order)
    )
    return list(result.scalars().all())


async def list_runs_for_experiment(
    session: AsyncSession,
    experiment_id: UUID,
) -> list[Run]:
    """Return all runs for an experiment ordered by creation time."""

    result = await session.execute(
        select(Run).where(Run.experiment_id == experiment_id).order_by(Run.created_at)
    )
    return list(result.scalars().all())


async def list_trials_for_experiment(
    session: AsyncSession,
    experiment_id: UUID,
) -> list[Trial]:
    """Return all trials for an experiment ordered by creation surrogate id."""

    result = await session.execute(
        select(Trial).where(Trial.experiment_id == experiment_id).order_by(Trial.id)
    )
    return list(result.scalars().all())


async def sync_experiment_status(session: AsyncSession, experiment_id: UUID) -> Experiment:
    """Recompute and persist experiment status from current runs and trials."""

    experiment = await get_experiment(session, experiment_id)
    runs = await list_runs_for_experiment(session, experiment_id)
    trials = await list_trials_for_experiment(session, experiment_id)
    derived = derive_experiment_status(
        current_status=experiment.status,
        runs=runs,
        trials=trials,
    )
    if derived != experiment.status:
        experiment.status = derived
        if derived == "running" and experiment.started_at is None:
            experiment.started_at = utc_now()
        if derived in {"completed", "completed_with_errors", "cancelled", "failed"}:
            experiment.completed_at = utc_now()
        await session.flush()
    return experiment
