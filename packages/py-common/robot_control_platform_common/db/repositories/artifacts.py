"""Artifact metadata persistence queries.

Artifact bytes live in the artifact store. These queries persist only
checksummed metadata rows.
"""

from __future__ import annotations

from uuid import UUID

from sqlalchemy import select
from sqlalchemy.dialects.postgresql import insert
from sqlalchemy.ext.asyncio import AsyncSession

from robot_control_platform_common.db.models import Artifact
from robot_control_platform_common.db.repositories.exceptions import EntityNotFoundError


async def add_artifact(session: AsyncSession, artifact: Artifact) -> Artifact:
    """Persist a new artifact metadata row."""

    session.add(artifact)
    await session.flush()
    return artifact


async def get_artifact(session: AsyncSession, artifact_id: UUID) -> Artifact:
    """Return artifact metadata by id or raise ``EntityNotFoundError``."""

    artifact = await session.get(Artifact, artifact_id)
    if artifact is None:
        msg = f"artifact {artifact_id} not found"
        raise EntityNotFoundError(msg)
    return artifact


async def list_artifacts_for_trial(session: AsyncSession, trial_id: UUID) -> list[Artifact]:
    """Return artifact metadata for a trial ordered by kind."""

    result = await session.execute(
        select(Artifact).where(Artifact.trial_id == trial_id).order_by(Artifact.kind)
    )
    return list(result.scalars().all())


async def create_artifact_if_absent(
    session: AsyncSession,
    artifact: Artifact,
) -> tuple[Artifact, bool]:
    """Insert artifact metadata or return the existing ``(trial_id, kind)`` row."""

    stmt = (
        insert(Artifact)
        .values(
            id=artifact.id,
            trial_id=artifact.trial_id,
            kind=artifact.kind,
            storage_key=artifact.storage_key,
            media_type=artifact.media_type,
            width_px=artifact.width_px,
            height_px=artifact.height_px,
            byte_size=artifact.byte_size,
            sha256=artifact.sha256,
            created_at=artifact.created_at,
        )
        .on_conflict_do_nothing(constraint="uq_artifacts_trial_kind")
        .returning(Artifact.id)
    )
    result = await session.execute(stmt)
    inserted_id = result.scalar_one_or_none()
    if inserted_id is not None:
        await session.flush()
        return await get_artifact(session, inserted_id), True

    existing_result = await session.execute(
        select(Artifact).where(
            Artifact.trial_id == artifact.trial_id,
            Artifact.kind == artifact.kind,
        )
    )
    existing = existing_result.scalar_one()
    return existing, False
