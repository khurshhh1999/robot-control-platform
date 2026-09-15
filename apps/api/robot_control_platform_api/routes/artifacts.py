"""Artifact metadata and content routes."""

from __future__ import annotations

from typing import Annotated

from fastapi import APIRouter, Depends, Request
from fastapi.responses import Response
from sqlalchemy.ext.asyncio import AsyncSession

from robot_control_platform_api.dependencies import get_db_session, get_runtime
from robot_control_platform_api.schemas.artifacts import ArtifactListResponse
from robot_control_platform_api.services import artifacts as artifact_service

router = APIRouter(prefix="/api/v1", tags=["artifacts"])


@router.get("/trials/{trial_id}/artifacts", response_model=ArtifactListResponse)
async def list_trial_artifacts(
    trial_id: str,
    session: Annotated[AsyncSession, Depends(get_db_session)],
) -> ArtifactListResponse:
    return await artifact_service.list_artifacts_for_trial(session, trial_id)


@router.get("/artifacts/{artifact_id}/content")
async def get_artifact_content(
    artifact_id: str,
    request: Request,
    session: Annotated[AsyncSession, Depends(get_db_session)],
) -> Response:
    runtime = get_runtime(request)
    content = await artifact_service.get_artifact_content(
        session,
        runtime.artifact_store,
        artifact_id,
    )
    return Response(
        content=content.payload,
        media_type=content.media_type,
        headers={
            "Content-Length": str(content.byte_size),
            "ETag": f'"{content.sha256}"',
            "X-Checksum-SHA256": content.sha256,
            "X-Content-Type-Options": "nosniff",
            "Content-Disposition": f'attachment; filename="{content.filename}"',
        },
    )
