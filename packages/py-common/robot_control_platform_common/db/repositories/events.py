"""Trial event persistence queries."""

from __future__ import annotations

from uuid import UUID

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from robot_control_platform_common.db.models import TrialEvent


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
