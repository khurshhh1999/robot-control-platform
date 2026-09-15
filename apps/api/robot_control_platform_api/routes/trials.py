"""Trial and trial-event routes."""

from __future__ import annotations

from typing import Annotated

from fastapi import APIRouter, Depends, Query
from sqlalchemy.ext.asyncio import AsyncSession

from robot_control_platform_api.dependencies import get_db_session
from robot_control_platform_api.schemas.trials import (
    TrialEventListResponse,
    TrialListResponse,
    TrialResponse,
)
from robot_control_platform_api.services import trials as trial_service

router = APIRouter(prefix="/api/v1", tags=["trials"])


@router.get("/trials", response_model=TrialListResponse)
async def list_trials(
    session: Annotated[AsyncSession, Depends(get_db_session)],
    experiment_id: Annotated[str | None, Query()] = None,
    policy_version_id: Annotated[str | None, Query()] = None,
    terminal_outcome: Annotated[str | None, Query()] = None,
    object_name: Annotated[str | None, Query(alias="object")] = None,
    reviewed: Annotated[bool | None, Query()] = None,
    limit: Annotated[int | None, Query(ge=1, le=100)] = None,
    cursor: Annotated[str | None, Query()] = None,
) -> TrialListResponse:
    return await trial_service.list_trials(
        session,
        experiment_id=experiment_id,
        policy_version_id=policy_version_id,
        terminal_outcome=terminal_outcome,
        object_name=object_name,
        reviewed=reviewed,
        limit=limit,
        cursor=cursor,
    )


@router.get("/trials/{trial_id}", response_model=TrialResponse)
async def get_trial(
    trial_id: str,
    session: Annotated[AsyncSession, Depends(get_db_session)],
) -> TrialResponse:
    return await trial_service.get_trial(session, trial_id)


@router.get("/trials/{trial_id}/events", response_model=TrialEventListResponse)
async def list_trial_events(
    trial_id: str,
    session: Annotated[AsyncSession, Depends(get_db_session)],
    limit: Annotated[int | None, Query(ge=1, le=100)] = None,
    cursor: Annotated[str | None, Query()] = None,
) -> TrialEventListResponse:
    return await trial_service.list_trial_events(
        session,
        trial_id,
        limit=limit,
        cursor=cursor,
    )
