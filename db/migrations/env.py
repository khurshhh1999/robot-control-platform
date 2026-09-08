"""Alembic environment. Schema changes apply only through reviewed migrations."""

from __future__ import annotations

import sys
from logging.config import fileConfig
from pathlib import Path

from alembic import context
from sqlalchemy import engine_from_config, pool

# Ensure workspace packages resolve when Alembic runs from ``db/``.
_REPO_ROOT = Path(__file__).resolve().parents[2]
_PY_COMMON = _REPO_ROOT / "packages" / "py-common"
for path in (_REPO_ROOT, _PY_COMMON):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)

from robot_control_platform_common.config import load_settings  # noqa: E402
from robot_control_platform_common.db import Base  # noqa: E402
from robot_control_platform_common.db import models as _models  # noqa: E402, F401
from robot_control_platform_common.db.session import database_url_from_dsn  # noqa: E402

config = context.config

if config.config_file_name is not None:
    fileConfig(config.config_file_name)

target_metadata = Base.metadata


def _sync_database_url() -> str:
    settings = load_settings()
    return database_url_from_dsn(settings.database_dsn())


def run_migrations_offline() -> None:
    """Run migrations in offline SQL emission mode."""

    url = _sync_database_url()
    context.configure(
        url=url,
        target_metadata=target_metadata,
        literal_binds=True,
        dialect_opts={"paramstyle": "named"},
        compare_type=True,
    )

    with context.begin_transaction():
        context.run_migrations()


def run_migrations_online() -> None:
    """Run migrations against a live database connection."""

    configuration = config.get_section(config.config_ini_section) or {}
    configuration["sqlalchemy.url"] = _sync_database_url()
    connectable = engine_from_config(
        configuration,
        prefix="sqlalchemy.",
        poolclass=pool.NullPool,
    )

    with connectable.connect() as connection:
        context.configure(
            connection=connection,
            target_metadata=target_metadata,
            compare_type=True,
        )

        with context.begin_transaction():
            context.run_migrations()


if context.is_offline_mode():
    run_migrations_offline()
else:
    run_migrations_online()
