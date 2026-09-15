"""Trial persistence with unique-key duplicate delivery protection."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from decimal import Decimal
from typing import Any
from uuid import UUID

from sqlalchemy import Select, exists, select
from sqlalchemy.dialects.postgresql import insert
from sqlalchemy.ext.asyncio import AsyncSession

from robot_control_platform_common.db.models import Annotation, Scenario, Trial
from robot_control_platform_common.db.repositories.exceptions import EntityNotFoundError
from robot_control_platform_common.ids import new_id
from robot_control_platform_common.time import utc_now

DEFAULT_PAGE_SIZE = 25
MAX_PAGE_SIZE = 100


@dataclass(frozen=True, slots=True)
class TrialListFilters:
    """Typed filters for trial collection queries."""

    experiment_id: UUID | None = None
    policy_version_id: UUID | None = None
    terminal_outcome: str | None = None
    object_name: str | None = None
    reviewed: bool | None = None


@dataclass(frozen=True, slots=True)
class TrialPage:
    """A page of trials with opaque cursor pagination metadata."""

    items: list[Trial]
    next_sort_values: tuple[str, ...] | None


def _clamp_page_size(limit: int) -> int:
    if limit < 1:
        msg = "limit must be at least 1"
        raise ValueError(msg)
    return min(limit, MAX_PAGE_SIZE)


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


def _apply_trial_filters(
    stmt: Select[tuple[Trial]],
    filters: TrialListFilters,
) -> Select[tuple[Trial]]:
    if filters.experiment_id is not None:
        stmt = stmt.where(Trial.experiment_id == filters.experiment_id)
    if filters.policy_version_id is not None:
        stmt = stmt.where(Trial.policy_version_id == filters.policy_version_id)
    if filters.terminal_outcome is not None:
        stmt = stmt.where(Trial.terminal_outcome == filters.terminal_outcome)
    if filters.object_name is not None:
        stmt = stmt.join(Scenario, Scenario.id == Trial.scenario_id).where(
            Scenario.object_name == filters.object_name
        )
    if filters.reviewed is True:
        stmt = stmt.where(exists().where(Annotation.trial_id == Trial.id))
    elif filters.reviewed is False:
        stmt = stmt.where(~exists().where(Annotation.trial_id == Trial.id))
    return stmt


async def list_trials(
    session: AsyncSession,
    *,
    filters: TrialListFilters | None = None,
    limit: int = DEFAULT_PAGE_SIZE,
    after_id: UUID | None = None,
) -> TrialPage:
    """List trials ordered by UUIDv7 id with optional filters and cursor."""

    page_size = _clamp_page_size(limit)
    active_filters = filters or TrialListFilters()
    stmt = select(Trial)
    stmt = _apply_trial_filters(stmt, active_filters)
    if after_id is not None:
        stmt = stmt.where(Trial.id > after_id)
    stmt = stmt.order_by(Trial.id).limit(page_size + 1)
    result = await session.execute(stmt)
    rows = list(result.scalars().all())
    has_more = len(rows) > page_size
    items = rows[:page_size]
    next_sort: tuple[str, ...] | None = None
    if has_more and items:
        next_sort = (str(items[-1].id),)
    return TrialPage(items=items, next_sort_values=next_sort)


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
