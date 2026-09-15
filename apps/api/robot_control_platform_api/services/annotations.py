"""Annotation service rules with optimistic concurrency."""

from __future__ import annotations

from uuid import UUID

from robot_control_platform_common.db.models import Annotation
from robot_control_platform_common.db.repositories import annotations as annotation_repo
from robot_control_platform_common.db.repositories import trials as trial_repo
from robot_control_platform_common.ids import new_id
from robot_control_platform_common.time import utc_now
from sqlalchemy.ext.asyncio import AsyncSession

from robot_control_platform_api.errors import ApiError
from robot_control_platform_api.schemas.annotations import (
    AnnotationCreate,
    AnnotationResponse,
    AnnotationUpdate,
)
from robot_control_platform_api.services.serializers import annotation_response


def _parse_uuid(value: str, *, field_name: str) -> UUID:
    try:
        return UUID(value)
    except ValueError as exc:
        raise ApiError(
            "VALIDATION_ERROR",
            status=422,
            detail=f"{field_name} must be a valid UUID",
        ) from exc


async def create_annotation(
    session: AsyncSession,
    trial_id: str,
    payload: AnnotationCreate,
) -> AnnotationResponse:
    identifier = _parse_uuid(trial_id, field_name="trial_id")
    await trial_repo.get_trial(session, identifier)
    now = utc_now()
    annotation = Annotation(
        id=new_id(),
        trial_id=identifier,
        label=payload.label,
        note=payload.note,
        reviewer=payload.reviewer,
        created_at=now,
        updated_at=now,
        revision=1,
    )
    await annotation_repo.add_annotation(session, annotation)
    return annotation_response(annotation)


async def update_annotation(
    session: AsyncSession,
    annotation_id: str,
    payload: AnnotationUpdate,
) -> AnnotationResponse:
    identifier = _parse_uuid(annotation_id, field_name="annotation_id")
    annotation = await annotation_repo.update_annotation(
        session,
        annotation_id=identifier,
        expected_revision=payload.revision,
        label=payload.label,
        note=payload.note,
    )
    return annotation_response(annotation)
