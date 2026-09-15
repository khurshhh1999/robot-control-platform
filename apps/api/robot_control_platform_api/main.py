"""FastAPI application factory, lifespan, and process entrypoint."""

from __future__ import annotations

import os
import sys
from collections.abc import AsyncIterator, Sequence
from contextlib import asynccontextmanager
from typing import Final

import uvicorn
from fastapi import FastAPI
from fastapi.exceptions import RequestValidationError
from fastapi.middleware.cors import CORSMiddleware
from robot_control_platform_common.config import (
    ConfigurationError,
    RuntimeEnv,
    Settings,
    load_settings,
)
from robot_control_platform_common.logging import configure_logging, get_logger
from starlette.exceptions import HTTPException as StarletteHTTPException

from robot_control_platform_api.dependencies import (
    AppRuntime,
    close_runtime,
    open_runtime,
)
from robot_control_platform_api.errors import (
    DOMAIN_EXCEPTION_TYPES,
    ApiError,
    api_error_handler,
    domain_error_handler,
    http_exception_handler,
    unhandled_error_handler,
    validation_error_handler,
)
from robot_control_platform_api.middleware import RequestIdMiddleware
from robot_control_platform_api.routes import (
    annotations_router,
    artifacts_router,
    experiments_router,
    health_router,
    policies_router,
    runs_router,
    scenario_sets_router,
    trials_router,
)

# Explicit browser origins for the Compose-published web UI. Never combine a
# wildcard origin list with credentialed CORS.
DEFAULT_CORS_ORIGINS: Final[tuple[str, ...]] = (
    "http://localhost:3000",
    "http://127.0.0.1:3000",
)


def _env_flag(name: str) -> bool:
    raw = os.environ.get(name, "").strip().lower()
    return raw in {"1", "true", "yes", "on"}


def docs_enabled_for(settings: Settings) -> bool:
    """Interactive docs and OpenAPI are off in demo unless explicitly enabled."""

    if settings.env is RuntimeEnv.DEMO:
        return _env_flag("RCP_ENABLE_DOCS")
    return True


def create_app(
    settings: Settings,
    *,
    runtime: AppRuntime | None = None,
    cors_origins: Sequence[str] | None = None,
) -> FastAPI:
    """Create the API application.

    When ``runtime`` is provided, lifespan attaches it without opening new
    connections. Production entrypoints omit ``runtime`` so lifespan validates
    configuration, opens the database, and probes artifact readability.
    """

    origins = tuple(cors_origins) if cors_origins is not None else DEFAULT_CORS_ORIGINS
    show_docs = docs_enabled_for(settings)

    @asynccontextmanager
    async def lifespan(app: FastAPI) -> AsyncIterator[None]:
        active_runtime = runtime
        owns_runtime = active_runtime is None
        configure_logging("api", settings.log_level.value)
        logger = get_logger("api")
        try:
            if active_runtime is None:
                active_runtime = open_runtime(settings)
                await active_runtime.check_database()
                active_runtime.check_artifacts()
            app.state.runtime = active_runtime
            app.state.settings = settings
            logger.info("api_started", env=settings.env.value)
            yield
        finally:
            try:
                logger.info("api_stopping")
            except ValueError:
                # Test clients may close capture streams before lifespan ends.
                pass
            if owns_runtime and active_runtime is not None:
                await close_runtime(active_runtime)
            app.state.runtime = None

    app = FastAPI(
        title="Robot Control Platform API",
        version="0.1.0",
        lifespan=lifespan,
        docs_url="/docs" if show_docs else None,
        redoc_url="/redoc" if show_docs else None,
        openapi_url="/openapi.json" if show_docs else None,
    )
    app.add_middleware(
        CORSMiddleware,
        allow_origins=list(origins),
        allow_credentials=False,
        allow_methods=["GET", "POST", "PUT", "PATCH", "DELETE", "OPTIONS"],
        allow_headers=["Authorization", "Content-Type", "X-Request-Id", "Idempotency-Key"],
        expose_headers=["X-Request-Id"],
    )
    app.add_middleware(RequestIdMiddleware)
    app.add_exception_handler(ApiError, api_error_handler)
    for exc_type in DOMAIN_EXCEPTION_TYPES:
        if exc_type is not ApiError:
            app.add_exception_handler(exc_type, domain_error_handler)
    app.add_exception_handler(RequestValidationError, validation_error_handler)
    app.add_exception_handler(StarletteHTTPException, http_exception_handler)
    # Exception is handled by ServerErrorMiddleware and re-raised for servers/tests.
    app.add_exception_handler(Exception, unhandled_error_handler)
    app.include_router(health_router)
    app.include_router(policies_router)
    app.include_router(scenario_sets_router)
    app.include_router(experiments_router)
    app.include_router(runs_router)
    app.include_router(trials_router)
    app.include_router(artifacts_router)
    app.include_router(annotations_router)

    if settings.env is RuntimeEnv.TEST:

        @app.get("/__test__/errors/{code}", include_in_schema=False)
        async def _error_probe(code: str) -> None:
            """Test-only route that raises mapped domain errors."""

            from robot_control_platform_common.artifacts.base import (
                ArtifactIntegrityError,
                ArtifactNotFoundError,
            )
            from robot_control_platform_common.db.repositories.exceptions import (
                DuplicateEntityError,
                EntityNotFoundError,
            )

            mapping: dict[str, Exception] = {
                "RESOURCE_NOT_FOUND": EntityNotFoundError("missing"),
                "ARTIFACT_NOT_FOUND": ArtifactNotFoundError("missing"),
                "ARTIFACT_INTEGRITY_FAILURE": ArtifactIntegrityError("corrupt"),
                "CONFLICT": DuplicateEntityError("duplicate"),
                "VALIDATION_ERROR": ApiError(
                    "VALIDATION_ERROR",
                    status=422,
                    detail="Probe validation failure",
                ),
                "DEPENDENCY_UNAVAILABLE": ApiError(
                    "DEPENDENCY_UNAVAILABLE",
                    status=503,
                    detail="Probe dependency failure",
                ),
                "INTERNAL_ERROR": RuntimeError("probe boom"),
            }
            exc = mapping.get(code)
            if exc is None:
                raise ApiError(
                    "RESOURCE_NOT_FOUND",
                    status=404,
                    detail="Unknown probe code",
                )
            raise exc

    return app


def serve(host: str = "0.0.0.0", port: int = 8000) -> None:
    """Load settings and serve the API with Uvicorn until stopped."""

    settings = load_settings()
    configure_logging("api", settings.log_level.value)
    app = create_app(settings)
    uvicorn.run(app, host=host, port=port, log_config=None)


def main() -> None:
    """Process entrypoint. Configuration errors exit without secret values."""

    try:
        serve()
    except ConfigurationError as exc:
        print(f"startup failed: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
