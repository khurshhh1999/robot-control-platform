"""Async SQLAlchemy engine and session helpers.

Application code must not call ``Base.metadata.create_all``. Schema changes go
through Alembic migrations only.
"""

from __future__ import annotations

from collections.abc import AsyncIterator
from contextlib import asynccontextmanager

from pydantic import SecretStr
from sqlalchemy.ext.asyncio import (
    AsyncEngine,
    AsyncSession,
    async_sessionmaker,
    create_async_engine,
)

from robot_control_platform_common.config import Settings


def database_url_from_dsn(dsn: SecretStr, *, driver: str = "psycopg") -> str:
    """Convert a ``postgresql://`` DSN into a SQLAlchemy URL for ``driver``."""

    raw = dsn.get_secret_value()
    prefix = "postgresql://"
    if not raw.startswith(prefix):
        msg = "database DSN must start with postgresql://"
        raise ValueError(msg)
    return f"postgresql+{driver}://{raw.removeprefix(prefix)}"


def create_engine(settings: Settings, *, echo: bool = False) -> AsyncEngine:
    """Create an async engine. Never log the constructed URL."""

    return create_async_engine(
        database_url_from_dsn(settings.database_dsn()),
        echo=echo,
        pool_pre_ping=True,
    )


def create_session_factory(engine: AsyncEngine) -> async_sessionmaker[AsyncSession]:
    """Return a session factory bound to ``engine``."""

    return async_sessionmaker(engine, expire_on_commit=False, class_=AsyncSession)


@asynccontextmanager
async def session_scope(
    session_factory: async_sessionmaker[AsyncSession],
) -> AsyncIterator[AsyncSession]:
    """Yield a session and commit on success; roll back on error."""

    session = session_factory()
    try:
        yield session
        await session.commit()
    except Exception:
        await session.rollback()
        raise
    finally:
        await session.close()
