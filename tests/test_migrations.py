"""Migration upgrade, introspection, downgrade, and re-upgrade tests.

Uses an ephemeral PostgreSQL container. Schema changes must never use
``create_all``; Alembic is the only schema authority.
"""

from __future__ import annotations

import shutil
import socket
import subprocess
import time
from collections.abc import Iterator
from pathlib import Path

import pytest
from alembic import command
from alembic.config import Config
from sqlalchemy import create_engine, inspect, text

REPO_ROOT = Path(__file__).resolve().parents[1]
DB_DIR = REPO_ROOT / "db"
ALEMBIC_INI = DB_DIR / "alembic.ini"
POSTGRES_IMAGE = (
    "postgres:16-bookworm@sha256:60f4761b9035e0b8d5218f701a8c3382f641bf12b1604822574cf5be3baeb537"
)
EXPECTED_TABLES = frozenset(
    {
        "policy_versions",
        "scenario_sets",
        "scenarios",
        "experiments",
        "experiment_policies",
        "runs",
        "trials",
        "trial_events",
        "artifacts",
        "annotations",
        "alembic_version",
    }
)
EXPECTED_UNIQUE_CONSTRAINTS = frozenset(
    {
        "uq_policy_versions_name_semver",
        "uq_scenarios_set_ordinal",
        "uq_scenarios_set_seed",
        "uq_experiment_policies_order",
        "uq_runs_experiment_idempotency_key",
        "uq_trials_experiment_policy_scenario",
        "uq_trial_events_trial_ordinal",
        "uq_artifacts_trial_kind",
    }
)
EXPECTED_FOREIGN_KEYS = frozenset(
    {
        "fk_scenarios_scenario_set_id",
        "fk_experiments_scenario_set_id",
        "fk_experiment_policies_experiment_id",
        "fk_experiment_policies_policy_version_id",
        "fk_runs_experiment_id",
        "fk_trials_experiment_id",
        "fk_trials_policy_version_id",
        "fk_trials_scenario_id",
        "fk_trial_events_trial_id",
        "fk_artifacts_trial_id",
        "fk_annotations_trial_id",
    }
)


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def _docker_available() -> bool:
    return shutil.which("docker") is not None


def _wait_for_postgres(url: str, *, timeout_seconds: float = 60.0) -> None:
    from sqlalchemy.exc import OperationalError

    deadline = time.monotonic() + timeout_seconds
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        engine = create_engine(url, pool_pre_ping=True)
        try:
            with engine.connect() as connection:
                connection.execute(text("SELECT 1"))
            engine.dispose()
            return
        except OperationalError as exc:
            last_error = exc
            engine.dispose()
            time.sleep(0.5)
    msg = f"PostgreSQL did not become ready: {last_error}"
    raise RuntimeError(msg)


@pytest.fixture(scope="module")
def postgres_url() -> Iterator[str]:
    if not _docker_available():
        pytest.fail("docker is required for migration checks")

    port = _free_port()
    password = "migration-check-password"
    user = "robot_app"
    database = "robot_platform"
    container = f"rcp-migration-check-{port}"
    url = f"postgresql+psycopg://{user}:{password}@127.0.0.1:{port}/{database}"

    run = subprocess.run(
        [
            "docker",
            "run",
            "--rm",
            "-d",
            "--name",
            container,
            "-e",
            f"POSTGRES_USER={user}",
            "-e",
            f"POSTGRES_PASSWORD={password}",
            "-e",
            f"POSTGRES_DB={database}",
            "-p",
            f"127.0.0.1:{port}:5432",
            POSTGRES_IMAGE,
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    if run.returncode != 0:
        pytest.fail(f"unable to start PostgreSQL container: {run.stderr.strip()}")

    try:
        _wait_for_postgres(url)
        yield url
    finally:
        subprocess.run(
            ["docker", "stop", container],
            check=False,
            capture_output=True,
            text=True,
        )


@pytest.fixture
def alembic_env(postgres_url: str, monkeypatch: pytest.MonkeyPatch) -> Config:
    without_scheme = postgres_url.split("://", 1)[1]
    credentials, host_part = without_scheme.split("@", 1)
    user, password = credentials.split(":", 1)
    host_port, database = host_part.split("/", 1)
    host, port_text = host_port.rsplit(":", 1)

    monkeypatch.setenv("RCP_ENV", "test")
    monkeypatch.setenv("RCP_LOG_LEVEL", "WARNING")
    monkeypatch.setenv("RCP_DATABASE_HOST", host)
    monkeypatch.setenv("RCP_DATABASE_PORT", port_text)
    monkeypatch.setenv("RCP_DATABASE_NAME", database)
    monkeypatch.setenv("RCP_DATABASE_USER", user)
    monkeypatch.setenv("RCP_DATABASE_PASSWORD", password)
    monkeypatch.setenv("RCP_ARTIFACT_ROOT", "/tmp/rcp-migration-artifacts")
    monkeypatch.setenv("RCP_API_BASE_URL", "http://127.0.0.1:8000")
    monkeypatch.setenv("RCP_WORKER_ID", "migration-check")
    monkeypatch.setenv("RCP_RUN_LEASE_SECONDS", "30")
    monkeypatch.setenv("RCP_RUN_HEARTBEAT_SECONDS", "10")
    monkeypatch.setenv("RCP_SIMULATION_GUI", "false")

    config = Config(str(ALEMBIC_INI))
    config.set_main_option("script_location", str(DB_DIR / "migrations"))
    config.set_main_option("path_separator", "os")
    config.set_main_option("prepend_sys_path", str(REPO_ROOT))
    return config


def _public_tables(url: str) -> set[str]:
    engine = create_engine(url)
    try:
        inspector = inspect(engine)
        return set(inspector.get_table_names(schema="public"))
    finally:
        engine.dispose()


def _named_uniques(url: str) -> set[str]:
    engine = create_engine(url)
    names: set[str] = set()
    try:
        inspector = inspect(engine)
        for table_name in inspector.get_table_names(schema="public"):
            for unique in inspector.get_unique_constraints(table_name, schema="public"):
                name = unique.get("name")
                if name:
                    names.add(name)
    finally:
        engine.dispose()
    return names


def _named_foreign_keys(url: str) -> set[str]:
    engine = create_engine(url)
    names: set[str] = set()
    try:
        inspector = inspect(engine)
        for table_name in inspector.get_table_names(schema="public"):
            for foreign_key in inspector.get_foreign_keys(table_name, schema="public"):
                name = foreign_key.get("name")
                if name:
                    names.add(name)
    finally:
        engine.dispose()
    return names


def _fk_delete_rules(url: str) -> dict[str, str]:
    engine = create_engine(url)
    rules: dict[str, str] = {}
    try:
        with engine.connect() as connection:
            rows = connection.execute(
                text(
                    """
                    SELECT conname, confdeltype
                    FROM pg_constraint
                    WHERE contype = 'f'
                    """
                )
            )
            # PostgreSQL: a=NO ACTION, r=RESTRICT, c=CASCADE, n=SET NULL, d=SET DEFAULT
            mapping = {"a": "NO ACTION", "r": "RESTRICT", "c": "CASCADE"}
            for name, delete_type in rows:
                rules[str(name)] = mapping.get(str(delete_type), str(delete_type))
    finally:
        engine.dispose()
    return rules


def _assert_schema(url: str) -> None:
    tables = _public_tables(url)
    assert EXPECTED_TABLES.issubset(tables)
    assert EXPECTED_UNIQUE_CONSTRAINTS.issubset(_named_uniques(url))
    assert EXPECTED_FOREIGN_KEYS.issubset(_named_foreign_keys(url))

    delete_rules = _fk_delete_rules(url)
    assert delete_rules["fk_scenarios_scenario_set_id"] == "CASCADE"
    assert delete_rules["fk_experiment_policies_experiment_id"] == "CASCADE"
    assert delete_rules["fk_trial_events_trial_id"] == "CASCADE"
    assert delete_rules["fk_experiments_scenario_set_id"] == "RESTRICT"
    assert delete_rules["fk_trials_experiment_id"] == "RESTRICT"
    assert delete_rules["fk_artifacts_trial_id"] == "RESTRICT"
    assert delete_rules["fk_annotations_trial_id"] == "RESTRICT"
    assert delete_rules["fk_runs_experiment_id"] == "RESTRICT"


def test_models_do_not_expose_create_all_helper() -> None:
    from robot_control_platform_common import db as db_package

    assert not hasattr(db_package, "create_all")
    assert "create_all" not in db_package.__all__


def test_empty_upgrade_introspection_downgrade_reupgrade(
    alembic_env: Config,
    postgres_url: str,
) -> None:
    assert _public_tables(postgres_url) == set()

    command.upgrade(alembic_env, "head")
    _assert_schema(postgres_url)

    engine = create_engine(postgres_url)
    try:
        with engine.connect() as connection:
            version = connection.execute(
                text("SELECT version_num FROM alembic_version")
            ).scalar_one()
        assert version == "20260915_0002"
    finally:
        engine.dispose()

    command.downgrade(alembic_env, "base")
    remaining = _public_tables(postgres_url)
    assert remaining == set() or remaining == {"alembic_version"}

    command.upgrade(alembic_env, "head")
    _assert_schema(postgres_url)


def test_run_request_fingerprint_column_present(
    alembic_env: Config,
    postgres_url: str,
) -> None:
    command.upgrade(alembic_env, "head")
    engine = create_engine(postgres_url)
    try:
        inspector = inspect(engine)
        columns = {column["name"] for column in inspector.get_columns("runs")}
        assert "request_fingerprint" in columns
    finally:
        engine.dispose()
    command.downgrade(alembic_env, "base")
