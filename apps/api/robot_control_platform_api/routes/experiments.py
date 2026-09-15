"""Experiment routes."""

from __future__ import annotations

from typing import Annotated

from fastapi import APIRouter, Depends, status
from sqlalchemy.ext.asyncio import AsyncSession

from robot_control_platform_api.dependencies import get_db_session
from robot_control_platform_api.schemas.experiments import (
    ExperimentCreate,
    ExperimentListResponse,
    ExperimentResponse,
)
from robot_control_platform_api.services import experiments as experiment_service

router = APIRouter(prefix="/api/v1", tags=["experiments"])


@router.post(
    "/experiments",
    response_model=ExperimentResponse,
    status_code=status.HTTP_201_CREATED,
)
async def create_experiment(
    payload: ExperimentCreate,
    session: Annotated[AsyncSession, Depends(get_db_session)],
) -> ExperimentResponse:
    return await experiment_service.create_experiment(session, payload)


@router.get("/experiments", response_model=ExperimentListResponse)
async def list_experiments(
    session: Annotated[AsyncSession, Depends(get_db_session)],
) -> ExperimentListResponse:
    return await experiment_service.list_experiments(session)


@router.get("/experiments/{experiment_id}", response_model=ExperimentResponse)
async def get_experiment(
    experiment_id: str,
    session: Annotated[AsyncSession, Depends(get_db_session)],
) -> ExperimentResponse:
    return await experiment_service.get_experiment(session, experiment_id)
