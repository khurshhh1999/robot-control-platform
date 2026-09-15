"""Annotation persistence queries."""

from __future__ import annotations

from uuid import UUID

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from robot_control_platform_common.db.models import Annotation
from robot_control_platform_common.db.repositories.exceptions import (
    EntityNotFoundError,
    OptimisticConcurrencyError,
)
from robot_control_platform_common.time import utc_now


async def add_annotation(session: AsyncSession, annotation: Annotation) -> Annotation:
    """Persist a new annotation."""

    session.add(annotation)
    await session.flush()
    return annotation


async def get_annotation(session: AsyncSession, annotation_id: UUID) -> Annotation:
    """Return an annotation by id or raise ``EntityNotFoundError``."""

    annotation = await session.get(Annotation, annotation_id)
    if annotation is None:
        msg = f"annotation {annotation_id} not found"
        raise EntityNotFoundError(msg)
    return annotation


async def list_annotations_for_trial(
    session: AsyncSession,
    trial_id: UUID,
) -> list[Annotation]:
    """Return annotations for a trial ordered by creation time."""

    result = await session.execute(
        select(Annotation)
        .where(Annotation.trial_id == trial_id)
        .order_by(Annotation.created_at, Annotation.id)
    )
    return list(result.scalars().all())


async def update_annotation(
    session: AsyncSession,
    *,
    annotation_id: UUID,
    expected_revision: int,
    label: str,
    note: str | None,
) -> Annotation:
    """Update an annotation when ``expected_revision`` matches the stored value."""

    if expected_revision < 1:
        msg = "expected_revision must be at least 1"
        raise ValueError(msg)

    result = await session.execute(
        select(Annotation).where(Annotation.id == annotation_id).with_for_update()
    )
    annotation = result.scalar_one_or_none()
    if annotation is None:
        msg = f"annotation {annotation_id} not found"
        raise EntityNotFoundError(msg)
    if annotation.revision != expected_revision:
        msg = (
            f"annotation {annotation_id} revision mismatch: "
            f"expected {expected_revision}, found {annotation.revision}"
        )
        raise OptimisticConcurrencyError(msg)

    annotation.label = label
    annotation.note = note
    annotation.revision = expected_revision + 1
    annotation.updated_at = utc_now()
    await session.flush()
    return annotation
