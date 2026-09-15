"""Policy version routes."""

from __future__ import annotations

from typing import Annotated

from fastapi import APIRouter, Depends
from sqlalchemy.ext.asyncio import AsyncSession

from robot_control_platform_api.dependencies import get_db_session
from robot_control_platform_api.schemas.policies import PolicyVersionListResponse
from robot_control_platform_api.services import policies as policy_service

router = APIRouter(prefix="/api/v1", tags=["policies"])


@router.get("/policies", response_model=PolicyVersionListResponse)
async def list_policies(
    session: Annotated[AsyncSession, Depends(get_db_session)],
) -> PolicyVersionListResponse:
    return await policy_service.list_policies(session)
