"""Trial persistence with unique-key duplicate delivery protection."""

from __future__ import annotations

from datetime import datetime
from decimal import Decimal
from typing import Any
from uuid import UUID

from sqlalchemy import select
from sqlalchemy.dialects.postgresql import insert
from sqlalchemy.ext.asyncio import AsyncSession

from robot_control_platform_common.db.models import Trial
from robot_control_platform_common.db.repositories.exceptions import EntityNotFoundError
from robot_control_platform_common.ids import new_id
from robot_control_platform_common.time import utc_now


async def get_trial(session: AsyncSession, trial_id: UUID) -> Trial:
    """Return a trial by id or raise ``EntityNotFoundError``."""

    trial = await session.get(Trial, trial_id)
    if trial is None:
        msg = f"trial {trial_id} not found"
        raise EntityNotFoundError(msg)
    return trial


async def get_trial_by_unique_key(
    session: AsyncSession,
    *,
    experiment_id: UUID,
    policy_version_id: UUID,
    scenario_id: UUID,
) -> Trial | None:
    """Return the trial for an experiment/policy/scenario triple, if present."""

    result = await session.execute(
        select(Trial).where(
            Trial.experiment_id == experiment_id,
            Trial.policy_version_id == policy_version_id,
            Trial.scenario_id == scenario_id,
        )
    )
    return result.scalar_one_or_none()


async def create_trial_if_absent(
    session: AsyncSession,
    *,
    experiment_id: UUID,
    policy_version_id: UUID,
    scenario_id: UUID,
    trial_id: UUID | None = None,
    status: str = "pending",
) -> tuple[Trial, bool]:
    """Insert a pending trial or return the existing unique-key row.

    Uses ``ON CONFLICT DO NOTHING`` on
    ``uq_trials_experiment_policy_scenario`` so duplicate delivery from a
    reclaimed run cannot create a second trial row.
    """

    identifier = trial_id or new_id()
    stmt = (
        insert(Trial)
        .values(
            id=identifier,
            experiment_id=experiment_id,
            policy_version_id=policy_version_id,
            scenario_id=scenario_id,
            status=status,
            terminal_outcome=None,
            success=None,
            collision_count=None,
            collision_max_force_newtons=None,
            duration_seconds=None,
            action_count=None,
            started_at=None,
            completed_at=None,
            simulator_metadata=None,
        )
        .on_conflict_do_nothing(constraint="uq_trials_experiment_policy_scenario")
        .returning(Trial.id)
    )
    result = await session.execute(stmt)
    inserted_id = result.scalar_one_or_none()
    if inserted_id is not None:
        await session.flush()
        trial = await get_trial(session, inserted_id)
        return trial, True

    existing = await get_trial_by_unique_key(
        session,
        experiment_id=experiment_id,
        policy_version_id=policy_version_id,
        scenario_id=scenario_id,
    )
    if existing is None:
        msg = "trial unique conflict resolved without locating existing row"
        raise RuntimeError(msg)
    return existing, False


async def mark_trial_running(
    session: AsyncSession,
    trial_id: UUID,
    *,
    started_at: datetime | None = None,
) -> Trial:
    """Transition a pending trial to ``running``."""

    trial = await get_trial(session, trial_id)
    trial.status = "running"
    trial.started_at = started_at or utc_now()
    await session.flush()
    return trial


async def mark_trial_completed(
    session: AsyncSession,
    *,
    trial_id: UUID,
    terminal_outcome: str,
    success: bool,
    collision_count: int,
    collision_max_force_newtons: Decimal,
    duration_seconds: Decimal,
    action_count: int,
    simulator_metadata: dict[str, Any] | None = None,
    completed_at: datetime | None = None,
) -> Trial:
    """Mark a trial completed with exactly one terminal outcome."""

    trial = await get_trial(session, trial_id)
    trial.status = "completed"
    trial.terminal_outcome = terminal_outcome
    trial.success = success
    trial.collision_count = collision_count
    trial.collision_max_force_newtons = collision_max_force_newtons
    trial.duration_seconds = duration_seconds
    trial.action_count = action_count
    trial.simulator_metadata = simulator_metadata
    trial.completed_at = completed_at or utc_now()
    await session.flush()
    return trial


async def mark_trial_failed(
    session: AsyncSession,
    *,
    trial_id: UUID,
    simulator_metadata: dict[str, Any] | None = None,
    completed_at: datetime | None = None,
) -> Trial:
    """Mark a trial failed with sanitized ``system_error`` outcome."""

    trial = await get_trial(session, trial_id)
    trial.status = "failed"
    trial.terminal_outcome = "system_error"
    trial.success = None
    trial.simulator_metadata = simulator_metadata
    trial.completed_at = completed_at or utc_now()
    await session.flush()
    return trial
