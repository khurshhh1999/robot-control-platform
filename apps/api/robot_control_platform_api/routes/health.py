"""Liveness and readiness health routes."""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, Request, status
from fastapi.responses import JSONResponse

from robot_control_platform_api.dependencies import get_runtime, request_id_from_request
from robot_control_platform_api.errors import PROBLEM_CONTENT_TYPE, ApiError, build_problem

router = APIRouter(tags=["health"])


@router.get("/health/live")
async def live() -> dict[str, str]:
    """Dependency-free liveness probe."""

    return {"status": "ok"}


@router.get("/health/ready")
async def ready(request: Request) -> JSONResponse:
    """Bounded readiness probe for database and artifact-store access."""

    runtime = get_runtime(request)
    request_id = request_id_from_request(request)
    try:
        await runtime.check_database()
        runtime.check_artifacts()
    except ApiError as exc:
        problem = build_problem(
            code=exc.code,
            status=exc.status,
            detail=exc.detail,
            title=exc.title,
        )
        headers: dict[str, str] = {}
        if request_id is not None:
            headers["X-Request-Id"] = request_id
        return JSONResponse(
            status_code=problem.status,
            content=problem.as_dict(),
            media_type=PROBLEM_CONTENT_TYPE,
            headers=headers,
        )

    payload: dict[str, Any] = {"status": "ok"}
    headers = {}
    if request_id is not None:
        headers["X-Request-Id"] = request_id
    return JSONResponse(status_code=status.HTTP_200_OK, content=payload, headers=headers)
