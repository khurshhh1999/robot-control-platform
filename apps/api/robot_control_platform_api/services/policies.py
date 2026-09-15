"""Policy version service rules."""

from __future__ import annotations

from robot_control_platform_common.db.repositories import policies as policy_repo
from sqlalchemy.ext.asyncio import AsyncSession

from robot_control_platform_api.schemas.policies import (
    PolicyVersionListResponse,
    PolicyVersionResponse,
)
from robot_control_platform_api.services.serializers import policy_response


async def list_policies(session: AsyncSession) -> PolicyVersionListResponse:
    rows = await policy_repo.list_policy_versions(session)
    items: list[PolicyVersionResponse] = [policy_response(row) for row in rows]
    return PolicyVersionListResponse(items=items)
