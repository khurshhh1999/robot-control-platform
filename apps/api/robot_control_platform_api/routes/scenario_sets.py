"""Scenario set routes."""

from __future__ import annotations

from typing import Annotated

from fastapi import APIRouter, Depends, status
from sqlalchemy.ext.asyncio import AsyncSession

from robot_control_platform_api.dependencies import get_db_session
from robot_control_platform_api.schemas.scenarios import ScenarioSetCreate, ScenarioSetResponse
from robot_control_platform_api.services import scenarios as scenario_service

router = APIRouter(prefix="/api/v1", tags=["scenario-sets"])


@router.post(
    "/scenario-sets",
    response_model=ScenarioSetResponse,
    status_code=status.HTTP_201_CREATED,
)
async def create_scenario_set(
    payload: ScenarioSetCreate,
    session: Annotated[AsyncSession, Depends(get_db_session)],
) -> ScenarioSetResponse:
    return await scenario_service.create_scenario_set(session, payload)


@router.get("/scenario-sets/{scenario_set_id}", response_model=ScenarioSetResponse)
async def get_scenario_set(
    scenario_set_id: str,
    session: Annotated[AsyncSession, Depends(get_db_session)],
) -> ScenarioSetResponse:
    return await scenario_service.get_scenario_set(session, scenario_set_id)
