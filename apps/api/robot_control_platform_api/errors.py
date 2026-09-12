"""RFC 9457 problem-details mapping for stable API error codes."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Final

from fastapi import Request
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
from robot_control_platform_common.artifacts.base import (
    ArtifactIntegrityError,
    ArtifactNotFoundError,
    ArtifactStoreError,
)
from robot_control_platform_common.db.repositories.exceptions import (
    DuplicateEntityError,
    EntityNotFoundError,
    InvalidLeaseStateError,
    LeaseOwnershipError,
    RepositoryError,
)
from starlette.exceptions import HTTPException as StarletteHTTPException

PROBLEM_CONTENT_TYPE: Final[str] = "application/problem+json"

ERROR_CODES: Final[frozenset[str]] = frozenset(
    {
        "VALIDATION_ERROR",
        "RESOURCE_NOT_FOUND",
        "CONFLICT",
        "IDEMPOTENCY_CONFLICT",
        "INVALID_STATE_TRANSITION",
        "POLICY_IMMUTABLE",
        "ARTIFACT_NOT_FOUND",
        "ARTIFACT_INTEGRITY_FAILURE",
        "RUN_ALREADY_TERMINAL",
        "DEPENDENCY_UNAVAILABLE",
        "SIMULATION_ERROR",
        "INTERNAL_ERROR",
    }
)

_SAFE_TITLES: Final[dict[str, str]] = {
    "VALIDATION_ERROR": "Validation Error",
    "RESOURCE_NOT_FOUND": "Resource Not Found",
    "CONFLICT": "Conflict",
    "IDEMPOTENCY_CONFLICT": "Idempotency Conflict",
    "INVALID_STATE_TRANSITION": "Invalid State Transition",
    "POLICY_IMMUTABLE": "Policy Immutable",
    "ARTIFACT_NOT_FOUND": "Artifact Not Found",
    "ARTIFACT_INTEGRITY_FAILURE": "Artifact Integrity Failure",
    "RUN_ALREADY_TERMINAL": "Run Already Terminal",
    "DEPENDENCY_UNAVAILABLE": "Dependency Unavailable",
    "SIMULATION_ERROR": "Simulation Error",
    "INTERNAL_ERROR": "Internal Error",
}


@dataclass(frozen=True, slots=True)
class ProblemDetail:
    """RFC 9457 problem details with a stable platform error code."""

    type: str
    title: str
    status: int
    detail: str
    code: str
    instance: str | None = None

    def as_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "type": self.type,
            "title": self.title,
            "status": self.status,
            "detail": self.detail,
            "code": self.code,
        }
        if self.instance is not None:
            payload["instance"] = self.instance
        return payload


class ApiError(Exception):
    """Domain or application error that maps to a problem-details response."""

    def __init__(
        self,
        code: str,
        *,
        status: int,
        detail: str,
        title: str | None = None,
    ) -> None:
        if code not in ERROR_CODES:
            msg = f"unsupported API error code: {code}"
            raise ValueError(msg)
        self.code = code
        self.status = status
        self.detail = detail
        self.title = title or _SAFE_TITLES[code]
        super().__init__(detail)


def problem_type_for_code(code: str) -> str:
    """Return a stable type URI for ``code``."""

    if code not in ERROR_CODES:
        msg = f"unsupported API error code: {code}"
        raise ValueError(msg)
    return f"urn:robot-control-platform:problem:{code}"


def build_problem(
    *,
    code: str,
    status: int,
    detail: str,
    title: str | None = None,
    instance: str | None = None,
) -> ProblemDetail:
    """Build a problem-details object without leaking internals."""

    if code not in ERROR_CODES:
        msg = f"unsupported API error code: {code}"
        raise ValueError(msg)
    return ProblemDetail(
        type=problem_type_for_code(code),
        title=title or _SAFE_TITLES[code],
        status=status,
        detail=detail,
        code=code,
        instance=instance,
    )


def map_exception(exc: BaseException) -> ProblemDetail:
    """Map a domain or framework exception to problem details.

    Responses never include tracebacks, SQL, local paths, environment values,
    or raw simulator errors.
    """

    if isinstance(exc, ApiError):
        return build_problem(
            code=exc.code,
            status=exc.status,
            detail=exc.detail,
            title=exc.title,
        )
    if isinstance(exc, RequestValidationError):
        return build_problem(
            code="VALIDATION_ERROR",
            status=422,
            detail="Request validation failed",
        )
    if isinstance(exc, EntityNotFoundError):
        return build_problem(
            code="RESOURCE_NOT_FOUND",
            status=404,
            detail="The requested resource was not found",
        )
    if isinstance(exc, ArtifactNotFoundError):
        return build_problem(
            code="ARTIFACT_NOT_FOUND",
            status=404,
            detail="The requested artifact was not found",
        )
    if isinstance(exc, ArtifactIntegrityError):
        return build_problem(
            code="ARTIFACT_INTEGRITY_FAILURE",
            status=409,
            detail="Artifact integrity verification failed",
        )
    if isinstance(exc, DuplicateEntityError):
        return build_problem(
            code="CONFLICT",
            status=409,
            detail="The resource already exists",
        )
    if isinstance(exc, (InvalidLeaseStateError, LeaseOwnershipError)):
        return build_problem(
            code="INVALID_STATE_TRANSITION",
            status=409,
            detail="The requested state transition is not allowed",
        )
    if isinstance(exc, ArtifactStoreError):
        return build_problem(
            code="DEPENDENCY_UNAVAILABLE",
            status=503,
            detail="Artifact store is unavailable",
        )
    if isinstance(exc, RepositoryError):
        return build_problem(
            code="INTERNAL_ERROR",
            status=500,
            detail="A repository error occurred",
        )
    if isinstance(exc, StarletteHTTPException):
        if exc.status_code == 404:
            return build_problem(
                code="RESOURCE_NOT_FOUND",
                status=404,
                detail="The requested resource was not found",
            )
        if exc.status_code == 405:
            return build_problem(
                code="VALIDATION_ERROR",
                status=405,
                detail="Method not allowed",
            )
        return build_problem(
            code="INTERNAL_ERROR" if exc.status_code >= 500 else "VALIDATION_ERROR",
            status=exc.status_code,
            detail="Request could not be completed",
        )
    return build_problem(
        code="INTERNAL_ERROR",
        status=500,
        detail="An unexpected error occurred",
    )


def problem_response(problem: ProblemDetail, *, request_id: str | None = None) -> JSONResponse:
    """Return an ``application/problem+json`` response."""

    headers: dict[str, str] = {}
    if request_id is not None:
        headers["X-Request-Id"] = request_id
    return JSONResponse(
        status_code=problem.status,
        content=problem.as_dict(),
        media_type=PROBLEM_CONTENT_TYPE,
        headers=headers,
    )


def _request_id_from(request: Request) -> str | None:
    request_id = getattr(request.state, "request_id", None)
    if isinstance(request_id, str):
        return request_id
    return None


async def api_error_handler(request: Request, exc: Exception) -> JSONResponse:
    assert isinstance(exc, ApiError)
    return problem_response(map_exception(exc), request_id=_request_id_from(request))


async def validation_error_handler(request: Request, exc: Exception) -> JSONResponse:
    assert isinstance(exc, RequestValidationError)
    return problem_response(map_exception(exc), request_id=_request_id_from(request))


async def http_exception_handler(request: Request, exc: Exception) -> JSONResponse:
    assert isinstance(exc, StarletteHTTPException)
    return problem_response(map_exception(exc), request_id=_request_id_from(request))


async def unhandled_error_handler(request: Request, exc: Exception) -> JSONResponse:
    return problem_response(map_exception(exc), request_id=_request_id_from(request))


async def domain_error_handler(request: Request, exc: Exception) -> JSONResponse:
    """Handle known domain exceptions inside ExceptionMiddleware (no re-raise)."""

    return problem_response(map_exception(exc), request_id=_request_id_from(request))


DOMAIN_EXCEPTION_TYPES: Final[tuple[type[Exception], ...]] = (
    ApiError,
    EntityNotFoundError,
    DuplicateEntityError,
    InvalidLeaseStateError,
    LeaseOwnershipError,
    RepositoryError,
    ArtifactNotFoundError,
    ArtifactIntegrityError,
    ArtifactStoreError,
)
