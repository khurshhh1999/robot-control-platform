"""Policy version persistence queries."""

from __future__ import annotations

from uuid import UUID

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from robot_control_platform_common.db.models import PolicyVersion
from robot_control_platform_common.db.repositories.exceptions import EntityNotFoundError


async def add_policy_version(session: AsyncSession, policy: PolicyVersion) -> PolicyVersion:
    """Persist a new policy version row."""

    session.add(policy)
    await session.flush()
    return policy


async def get_policy_version(session: AsyncSession, policy_id: UUID) -> PolicyVersion:
    """Return a policy version by id or raise ``EntityNotFoundError``."""

    policy = await session.get(PolicyVersion, policy_id)
    if policy is None:
        msg = f"policy version {policy_id} not found"
        raise EntityNotFoundError(msg)
    return policy


async def list_policy_versions(session: AsyncSession) -> list[PolicyVersion]:
    """Return all policy versions ordered by name then semantic version."""

    result = await session.execute(
        select(PolicyVersion).order_by(PolicyVersion.name, PolicyVersion.semantic_version)
    )
    return list(result.scalars().all())
