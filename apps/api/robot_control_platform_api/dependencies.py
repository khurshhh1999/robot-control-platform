"""FastAPI dependency providers and readiness probes."""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator, Awaitable, Callable
from dataclasses import dataclass
from typing import Final

from fastapi import Request
from robot_control_platform_common.artifacts.filesystem import FilesystemArtifactStore
from robot_control_platform_common.config import Settings
from robot_control_platform_common.db.session import create_engine, create_session_factory
from sqlalchemy import text
from sqlalchemy.ext.asyncio import AsyncEngine, AsyncSession, async_sessionmaker

from robot_control_platform_api.errors import ApiError

READY_PROBE_TIMEOUT_SECONDS: Final[float] = 2.0

DatabaseProbe = Callable[[], Awaitable[None]]
ArtifactProbe = Callable[[], None]


@dataclass(slots=True)
class AppRuntime:
    """Process runtime resources created during application lifespan."""

    settings: Settings
    engine: AsyncEngine
    session_factory: async_sessionmaker[AsyncSession]
    artifact_store: FilesystemArtifactStore
    check_database: DatabaseProbe
    check_artifacts: ArtifactProbe


def get_runtime(request: Request) -> AppRuntime:
    """Return the application runtime attached during lifespan startup."""

    runtime = getattr(request.app.state, "runtime", None)
    if not isinstance(runtime, AppRuntime):
        raise ApiError(
            "DEPENDENCY_UNAVAILABLE",
            status=503,
            detail="Application runtime is not ready",
        )
    return runtime


def request_id_from_request(request: Request) -> str | None:
    value = getattr(request.state, "request_id", None)
    return value if isinstance(value, str) else None


def get_settings(request: Request) -> Settings:
    return get_runtime(request).settings


async def get_db_session(request: Request) -> AsyncIterator[AsyncSession]:
    runtime = get_runtime(request)
    session = runtime.session_factory()
    try:
        yield session
    finally:
        await session.close()


def build_default_database_probe(
    session_factory: async_sessionmaker[AsyncSession],
    *,
    timeout_seconds: float = READY_PROBE_TIMEOUT_SECONDS,
) -> DatabaseProbe:
    """Return a bounded readiness probe that executes ``SELECT 1``."""

    async def _probe() -> None:
        try:
            async with session_factory() as session:
                await asyncio.wait_for(session.execute(text("SELECT 1")), timeout=timeout_seconds)
        except Exception as exc:
            raise ApiError(
                "DEPENDENCY_UNAVAILABLE",
                status=503,
                detail="Database is unavailable",
            ) from exc

    return _probe


def build_default_artifact_probe(artifact_store: FilesystemArtifactStore) -> ArtifactProbe:
    """Return a probe that confirms the artifact root is readable."""

    def _probe() -> None:
        try:
            root = artifact_store.root
            if not root.exists() or not root.is_dir():
                raise ApiError(
                    "DEPENDENCY_UNAVAILABLE",
                    status=503,
                    detail="Artifact store is unavailable",
                )
            # Touch the directory listing path; ignore empty roots.
            next(root.iterdir(), None)
            artifact_store.list_storage_keys()
        except ApiError:
            raise
        except Exception as exc:
            raise ApiError(
                "DEPENDENCY_UNAVAILABLE",
                status=503,
                detail="Artifact store is unavailable",
            ) from exc

    return _probe


def open_runtime(settings: Settings) -> AppRuntime:
    """Open database and artifact dependencies for the API process."""

    engine = create_engine(settings)
    session_factory = create_session_factory(engine)
    artifact_store = FilesystemArtifactStore(settings.artifact_root)
    return AppRuntime(
        settings=settings,
        engine=engine,
        session_factory=session_factory,
        artifact_store=artifact_store,
        check_database=build_default_database_probe(session_factory),
        check_artifacts=build_default_artifact_probe(artifact_store),
    )


async def close_runtime(runtime: AppRuntime) -> None:
    """Dispose database resources. Artifact store needs no explicit close."""

    await runtime.engine.dispose()
