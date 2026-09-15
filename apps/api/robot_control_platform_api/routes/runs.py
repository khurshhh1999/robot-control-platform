"""Run routes with idempotent start and cancellation."""

from __future__ import annotations

from typing import Annotated

from fastapi import APIRouter, Depends, Header, Request, Response, status
from sqlalchemy.ext.asyncio import AsyncSession

from robot_control_platform_api.dependencies import get_db_session
from robot_control_platform_api.errors import ApiError
from robot_control_platform_api.schemas.runs import RunCreate, RunResponse
from robot_control_platform_api.services import runs as run_service

router = APIRouter(prefix="/api/v1", tags=["runs"])


@router.post(
    "/experiments/{experiment_id}/runs",
    response_model=RunResponse,
)
async def start_experiment_run(
    experiment_id: str,
    request: Request,
    response: Response,
    session: Annotated[AsyncSession, Depends(get_db_session)],
    idempotency_key: Annotated[str | None, Header(alias="Idempotency-Key")] = None,
) -> RunResponse:
    if idempotency_key is None:
        raise ApiError(
            "VALIDATION_ERROR",
            status=422,
            detail="Idempotency-Key header is required",
        )
    raw_body = await request.body()
    if raw_body.strip():
        try:
            RunCreate.model_validate_json(raw_body)
        except Exception as exc:
            raise ApiError(
                "VALIDATION_ERROR",
                status=422,
                detail="Request validation failed",
            ) from exc
    request_fingerprint = run_service.fingerprint_request_body(raw_body)
    result, created = await run_service.start_run(
        session,
        experiment_id=experiment_id,
        idempotency_key=idempotency_key,
        request_fingerprint=request_fingerprint,
    )
    response.status_code = status.HTTP_201_CREATED if created else status.HTTP_200_OK
    return result


@router.get("/runs/{run_id}", response_model=RunResponse)
async def get_run(
    run_id: str,
    session: Annotated[AsyncSession, Depends(get_db_session)],
) -> RunResponse:
    return await run_service.get_run(session, run_id)


@router.post("/runs/{run_id}/cancel", response_model=RunResponse)
async def cancel_run(
    run_id: str,
    session: Annotated[AsyncSession, Depends(get_db_session)],
) -> RunResponse:
    return await run_service.cancel_run(session, run_id)
