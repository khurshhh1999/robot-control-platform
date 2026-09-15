"""Trial event persistence queries."""

from __future__ import annotations

from dataclasses import dataclass
from uuid import UUID

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from robot_control_platform_common.db.models import TrialEvent
from robot_control_platform_common.db.repositories.trials import DEFAULT_PAGE_SIZE, MAX_PAGE_SIZE


@dataclass(frozen=True, slots=True)
class TrialEventPage:
    """A page of trial events with cursor pagination metadata."""

    items: list[TrialEvent]
    next_sort_values: tuple[int, ...] | None


def _clamp_page_size(limit: int) -> int:
    if limit < 1:
        msg = "limit must be at least 1"
        raise ValueError(msg)
    return min(limit, MAX_PAGE_SIZE)


async def add_trial_event(session: AsyncSession, event: TrialEvent) -> TrialEvent:
    """Persist a single trial event."""

    session.add(event)
    await session.flush()
    return event


async def list_trial_events(session: AsyncSession, trial_id: UUID) -> list[TrialEvent]:
    """Return events for a trial ordered by ordinal."""

    result = await session.execute(
        select(TrialEvent).where(TrialEvent.trial_id == trial_id).order_by(TrialEvent.ordinal)
    )
    return list(result.scalars().all())


async def list_trial_events_page(
    session: AsyncSession,
    trial_id: UUID,
    *,
    limit: int = DEFAULT_PAGE_SIZE,
    after_ordinal: int | None = None,
) -> TrialEventPage:
    """List trial events ordered by ordinal with opaque cursor support."""

    page_size = _clamp_page_size(limit)
    stmt = select(TrialEvent).where(TrialEvent.trial_id == trial_id)
    if after_ordinal is not None:
        stmt = stmt.where(TrialEvent.ordinal > after_ordinal)
    stmt = stmt.order_by(TrialEvent.ordinal).limit(page_size + 1)
    result = await session.execute(stmt)
    rows = list(result.scalars().all())
    has_more = len(rows) > page_size
    items = rows[:page_size]
    next_sort: tuple[int, ...] | None = None
    if has_more and items:
        next_sort = (items[-1].ordinal,)
    return TrialEventPage(items=items, next_sort_values=next_sort)
