"""Annotation routes."""

from __future__ import annotations

from typing import Annotated

from fastapi import APIRouter, Depends, status
from sqlalchemy.ext.asyncio import AsyncSession

from robot_control_platform_api.dependencies import get_db_session
from robot_control_platform_api.schemas.annotations import (
    AnnotationCreate,
    AnnotationResponse,
    AnnotationUpdate,
)
from robot_control_platform_api.services import annotations as annotation_service

router = APIRouter(prefix="/api/v1", tags=["annotations"])


@router.post(
    "/trials/{trial_id}/annotations",
    response_model=AnnotationResponse,
    status_code=status.HTTP_201_CREATED,
)
async def create_annotation(
    trial_id: str,
    payload: AnnotationCreate,
    session: Annotated[AsyncSession, Depends(get_db_session)],
) -> AnnotationResponse:
    return await annotation_service.create_annotation(session, trial_id, payload)


@router.patch("/annotations/{annotation_id}", response_model=AnnotationResponse)
async def update_annotation(
    annotation_id: str,
    payload: AnnotationUpdate,
    session: Annotated[AsyncSession, Depends(get_db_session)],
) -> AnnotationResponse:
    return await annotation_service.update_annotation(session, annotation_id, payload)
