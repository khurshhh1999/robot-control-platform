"""Repository and run-lease queue tests.

Uses an ephemeral PostgreSQL container and Alembic migrations. Covers
two-worker exclusion, crash reclaim, duplicate trial delivery, and
preservation of completed trial evidence.
"""

from __future__ import annotations

import asyncio
import shutil
import socket
import subprocess
import time
from collections.abc import AsyncIterator, Iterator
from datetime import timedelta
from decimal import Decimal
from pathlib import Path

import pytest
from alembic import command
from alembic.config import Config
from robot_control_platform_common.db.models import (
    Artifact,
    Experiment,
    ExperimentPolicy,
    PolicyVersion,
    Run,
    Scenario,
    ScenarioSet,
    Trial,
    TrialEvent,
)
from robot_control_platform_common.db.repositories import (
    artifacts,
    experiments,
    policies,
    runs,
)
from robot_control_platform_common.db.repositories import (
    events as event_repo,
)
from robot_control_platform_common.db.repositories import scenarios as scenario_repo
from robot_control_platform_common.db.repositories import trials as trial_repo
from robot_control_platform_common.db.repositories.exceptions import (
    InvalidLeaseStateError,
    LeaseOwnershipError,
)
from robot_control_platform_common.db.repositories.experiments import derive_experiment_status
from robot_control_platform_common.db.session import create_session_factory, session_scope
from robot_control_platform_common.ids import new_id
from robot_control_platform_common.time import utc_now
from sqlalchemy import create_engine, func, select, text
from sqlalchemy.ext.asyncio import AsyncSession, async_sessionmaker, create_async_engine

REPO_ROOT = Path(__file__).resolve().parents[1]
DB_DIR = REPO_ROOT / "db"
ALEMBIC_INI = DB_DIR / "alembic.ini"
POSTGRES_IMAGE = (
    "postgres:16-bookworm@sha256:60f4761b9035e0b8d5218f701a8c3382f641bf12b1604822574cf5be3baeb537"
)
CHECKSUM = "a" * 64
MAX_ATTEMPTS = 3
LEASE_SECONDS = 30


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
        pytest.fail("docker is required for repository checks")

    port = _free_port()
    password = "repository-check-password"
    user = "robot_app"
    database = "robot_platform"
    container = f"rcp-repository-check-{port}"
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


@pytest.fixture(scope="module")
def monkeypatch_module() -> Iterator[pytest.MonkeyPatch]:
    with pytest.MonkeyPatch.context() as patcher:
        yield patcher


@pytest.fixture(scope="module")
def migrated_postgres_url(
    postgres_url: str,
    monkeypatch_module: pytest.MonkeyPatch,
) -> str:
    without_scheme = postgres_url.split("://", 1)[1]
    credentials, host_part = without_scheme.split("@", 1)
    user, password = credentials.split(":", 1)
    host_port, database = host_part.split("/", 1)
    host, port_text = host_port.rsplit(":", 1)

    monkeypatch_module.setenv("RCP_ENV", "test")
    monkeypatch_module.setenv("RCP_LOG_LEVEL", "WARNING")
    monkeypatch_module.setenv("RCP_DATABASE_HOST", host)
    monkeypatch_module.setenv("RCP_DATABASE_PORT", port_text)
    monkeypatch_module.setenv("RCP_DATABASE_NAME", database)
    monkeypatch_module.setenv("RCP_DATABASE_USER", user)
    monkeypatch_module.setenv("RCP_DATABASE_PASSWORD", password)
    monkeypatch_module.setenv("RCP_ARTIFACT_ROOT", "/tmp/rcp-repository-artifacts")
    monkeypatch_module.setenv("RCP_API_BASE_URL", "http://127.0.0.1:8000")
    monkeypatch_module.setenv("RCP_WORKER_ID", "repository-check")
    monkeypatch_module.setenv("RCP_RUN_LEASE_SECONDS", "30")
    monkeypatch_module.setenv("RCP_RUN_HEARTBEAT_SECONDS", "10")
    monkeypatch_module.setenv("RCP_SIMULATION_GUI", "false")

    config = Config(str(ALEMBIC_INI))
    config.set_main_option("script_location", str(DB_DIR / "migrations"))
    config.set_main_option("path_separator", "os")
    config.set_main_option("prepend_sys_path", str(REPO_ROOT))
    command.upgrade(config, "head")
    return postgres_url


@pytest.fixture
async def session_factory(
    migrated_postgres_url: str,
) -> AsyncIterator[async_sessionmaker[AsyncSession]]:
    engine = create_async_engine(migrated_postgres_url, pool_pre_ping=True)
    factory = create_session_factory(engine)
    try:
        yield factory
    finally:
        await engine.dispose()


async def _seed_experiment_graph(
    session: AsyncSession,
) -> tuple[Experiment, PolicyVersion, Scenario]:
    now = utc_now()
    policy = PolicyVersion(
        id=new_id(),
        name="fixed",
        semantic_version=str(new_id()),
        description="test policy",
        config={"name": "fixed"},
        config_sha256=CHECKSUM,
        source_revision="test-revision",
        container_image_digest="sha256:" + "b" * 64,
        created_at=now,
    )
    await policies.add_policy_version(session, policy)

    scenario_set = ScenarioSet(
        id=new_id(),
        name="fixture-set",
        generator_version="1.0.0",
        scenario_count=1,
        seed_manifest={"seeds": [1]},
        scene_config={"scene": "baseline"},
        checksum=CHECKSUM,
    )
    await scenario_repo.add_scenario_set(session, scenario_set)

    scenario = Scenario(
        id=new_id(),
        scenario_set_id=scenario_set.id,
        ordinal=0,
        seed=1,
        object_name="cube",
        object_category="parcel",
        target_bin="bin_a",
        initial_pose={"position_m": [0.0, 0.0, 0.1], "orientation_xyzw": [0, 0, 0, 1]},
        physical_properties={"mass_kg": 0.2, "friction": 0.5},
        checksum=CHECKSUM,
    )
    await scenario_repo.add_scenario(session, scenario)

    experiment = Experiment(
        id=new_id(),
        name="fixture-experiment",
        description=None,
        scenario_set_id=scenario_set.id,
        status="queued",
        requested_by="tester",
        created_at=now,
        started_at=None,
        completed_at=None,
        source_revision="test-revision",
        simulator_image_digest="sha256:" + "c" * 64,
    )
    await experiments.add_experiment(session, experiment)
    await experiments.add_experiment_policy(
        session,
        ExperimentPolicy(
            experiment_id=experiment.id,
            policy_version_id=policy.id,
            execution_order=0,
        ),
    )
    return experiment, policy, scenario


@pytest.mark.asyncio
async def test_two_workers_cannot_claim_same_run(
    session_factory: async_sessionmaker[AsyncSession],
) -> None:
    async with session_scope(session_factory) as session:
        experiment, _policy, _scenario = await _seed_experiment_graph(session)
        await runs.create_queued_run(
            session,
            experiment_id=experiment.id,
            idempotency_key="two-worker-key",
        )

    async def claim(worker_id: str) -> str | None:
        async with session_scope(session_factory) as session:
            claimed = await runs.claim_next_run(
                session,
                worker_id=worker_id,
                lease_seconds=LEASE_SECONDS,
                max_attempts=MAX_ATTEMPTS,
            )
            return None if claimed is None else str(claimed.id)

    first, second = await asyncio.gather(claim("worker-a"), claim("worker-b"))
    winners = [run_id for run_id in (first, second) if run_id is not None]
    assert len(winners) == 1

    async with session_scope(session_factory) as session:
        leftover = await runs.claim_next_run(
            session,
            worker_id="worker-c",
            lease_seconds=LEASE_SECONDS,
            max_attempts=MAX_ATTEMPTS,
        )
        assert leftover is None


@pytest.mark.asyncio
async def test_expired_lease_is_reclaimed_without_duplicating_completed_trials(
    session_factory: async_sessionmaker[AsyncSession],
) -> None:
    async with session_scope(session_factory) as session:
        experiment, policy, scenario = await _seed_experiment_graph(session)
        queued = await runs.create_queued_run(
            session,
            experiment_id=experiment.id,
            idempotency_key="reclaim-key",
        )
        claimed = await runs.claim_next_run(
            session,
            worker_id="worker-crash",
            lease_seconds=LEASE_SECONDS,
            max_attempts=MAX_ATTEMPTS,
        )
        assert claimed is not None
        assert claimed.id == queued.id
        assert claimed.attempt == 1

        trial, created = await trial_repo.create_trial_if_absent(
            session,
            experiment_id=experiment.id,
            policy_version_id=policy.id,
            scenario_id=scenario.id,
        )
        assert created is True
        await trial_repo.mark_trial_running(session, trial.id)
        await trial_repo.mark_trial_completed(
            session,
            trial_id=trial.id,
            terminal_outcome="success",
            success=True,
            collision_count=0,
            collision_max_force_newtons=Decimal("0"),
            duration_seconds=Decimal("1.250000"),
            action_count=4,
            simulator_metadata={"note": "completed-before-crash"},
        )
        await artifacts.add_artifact(
            session,
            Artifact(
                id=new_id(),
                trial_id=trial.id,
                kind="terminal_rgb",
                storage_key=f"{experiment.id}/{trial.id}/terminal_rgb.png",
                media_type="image/png",
                width_px=640,
                height_px=480,
                byte_size=128,
                sha256=CHECKSUM,
                created_at=utc_now(),
            ),
        )
        await session.execute(
            text(
                """
                UPDATE runs
                SET lease_expires_at = :expired
                WHERE id = :run_id
                """
            ),
            {
                "expired": utc_now() - timedelta(seconds=5),
                "run_id": claimed.id,
            },
        )
        completed_trial_id = trial.id
        run_id = claimed.id
        experiment_id = experiment.id
        policy_id = policy.id
        scenario_id = scenario.id

    async with session_scope(session_factory) as session:
        reclaimed = await runs.claim_next_run(
            session,
            worker_id="worker-recovery",
            lease_seconds=LEASE_SECONDS,
            max_attempts=MAX_ATTEMPTS,
        )
        assert reclaimed is not None
        assert reclaimed.id == run_id
        assert reclaimed.lease_owner == "worker-recovery"
        assert reclaimed.attempt == 2
        assert reclaimed.status == "claimed"

        duplicate, created_again = await trial_repo.create_trial_if_absent(
            session,
            experiment_id=experiment_id,
            policy_version_id=policy_id,
            scenario_id=scenario_id,
        )
        assert created_again is False
        assert duplicate.id == completed_trial_id
        assert duplicate.status == "completed"
        assert duplicate.terminal_outcome == "success"
        assert duplicate.simulator_metadata == {"note": "completed-before-crash"}

        preserved = await artifacts.list_artifacts_for_trial(session, completed_trial_id)
        assert len(preserved) == 1
        assert preserved[0].kind == "terminal_rgb"
        assert preserved[0].sha256 == CHECKSUM

        trial_count = await session.scalar(
            select(func.count()).select_from(Trial).where(Trial.experiment_id == experiment_id)
        )
        assert trial_count == 1


@pytest.mark.asyncio
async def test_duplicate_trial_insert_is_defeated_by_unique_key(
    session_factory: async_sessionmaker[AsyncSession],
) -> None:
    async with session_scope(session_factory) as session:
        experiment, policy, scenario = await _seed_experiment_graph(session)
        experiment_id = experiment.id
        policy_id = policy.id
        scenario_id = scenario.id

    async def insert_once() -> str:
        async with session_scope(session_factory) as session:
            trial, _created = await trial_repo.create_trial_if_absent(
                session,
                experiment_id=experiment_id,
                policy_version_id=policy_id,
                scenario_id=scenario_id,
            )
            return str(trial.id)

    first, second = await asyncio.gather(insert_once(), insert_once())
    assert first == second

    async with session_scope(session_factory) as session:
        count = await session.scalar(
            select(func.count()).select_from(Trial).where(Trial.experiment_id == experiment_id)
        )
        assert count == 1


@pytest.mark.asyncio
async def test_heartbeat_requires_owner_and_nonterminal_lease(
    session_factory: async_sessionmaker[AsyncSession],
) -> None:
    async with session_scope(session_factory) as session:
        experiment, _policy, _scenario = await _seed_experiment_graph(session)
        await runs.create_queued_run(
            session,
            experiment_id=experiment.id,
            idempotency_key="heartbeat-key",
        )
        claimed = await runs.claim_next_run(
            session,
            worker_id="owner",
            lease_seconds=LEASE_SECONDS,
            max_attempts=MAX_ATTEMPTS,
        )
        assert claimed is not None
        run_id = claimed.id
        original_expiry = claimed.lease_expires_at

    async with session_scope(session_factory) as session:
        with pytest.raises(LeaseOwnershipError):
            await runs.heartbeat_run(
                session,
                run_id=run_id,
                worker_id="intruder",
                lease_seconds=LEASE_SECONDS,
            )

    async with session_scope(session_factory) as session:
        renewed = await runs.heartbeat_run(
            session,
            run_id=run_id,
            worker_id="owner",
            lease_seconds=LEASE_SECONDS,
            now=(original_expiry or utc_now()) + timedelta(seconds=1),
        )
        assert renewed.lease_expires_at is not None
        assert original_expiry is not None
        assert renewed.lease_expires_at > original_expiry

    async with session_scope(session_factory) as session:
        await runs.complete_run(session, run_id=run_id, status="completed")
        with pytest.raises(InvalidLeaseStateError):
            await runs.heartbeat_run(
                session,
                run_id=run_id,
                worker_id="owner",
                lease_seconds=LEASE_SECONDS,
            )


@pytest.mark.asyncio
async def test_requeue_fails_when_attempt_limit_reached(
    session_factory: async_sessionmaker[AsyncSession],
) -> None:
    async with session_scope(session_factory) as session:
        experiment, _policy, _scenario = await _seed_experiment_graph(session)
        await runs.create_queued_run(
            session,
            experiment_id=experiment.id,
            idempotency_key="attempt-limit-key",
        )
        claimed = await runs.claim_next_run(
            session,
            worker_id="worker-limit",
            lease_seconds=LEASE_SECONDS,
            max_attempts=1,
        )
        assert claimed is not None
        assert claimed.attempt == 1
        await session.execute(
            text(
                """
                UPDATE runs
                SET lease_expires_at = :expired
                WHERE id = :run_id
                """
            ),
            {
                "expired": utc_now() - timedelta(seconds=1),
                "run_id": claimed.id,
            },
        )
        run_id = claimed.id

    async with session_scope(session_factory) as session:
        changed = await runs.requeue_expired_leases(session, max_attempts=1)
        assert len(changed) == 1
        assert changed[0].id == run_id
        assert changed[0].status == "failed"
        assert changed[0].error_detail == "run lease retries exhausted"

        again = await runs.claim_next_run(
            session,
            worker_id="worker-limit-2",
            lease_seconds=LEASE_SECONDS,
            max_attempts=1,
        )
        assert again is None


@pytest.mark.asyncio
async def test_sync_experiment_status_from_runs_and_trials(
    session_factory: async_sessionmaker[AsyncSession],
) -> None:
    async with session_scope(session_factory) as session:
        experiment, policy, scenario = await _seed_experiment_graph(session)
        await runs.create_queued_run(
            session,
            experiment_id=experiment.id,
            idempotency_key="status-key",
        )
        synced = await experiments.sync_experiment_status(session, experiment.id)
        assert synced.status == "queued"

        claimed = await runs.claim_next_run(
            session,
            worker_id="status-worker",
            lease_seconds=LEASE_SECONDS,
            max_attempts=MAX_ATTEMPTS,
        )
        assert claimed is not None
        await runs.mark_run_running(session, claimed.id)
        synced = await experiments.sync_experiment_status(session, experiment.id)
        assert synced.status == "running"

        trial, _ = await trial_repo.create_trial_if_absent(
            session,
            experiment_id=experiment.id,
            policy_version_id=policy.id,
            scenario_id=scenario.id,
        )
        await trial_repo.mark_trial_running(session, trial.id)
        await trial_repo.mark_trial_completed(
            session,
            trial_id=trial.id,
            terminal_outcome="success",
            success=True,
            collision_count=0,
            collision_max_force_newtons=Decimal("0"),
            duration_seconds=Decimal("0.500000"),
            action_count=2,
        )
        await runs.complete_run(session, run_id=claimed.id, status="completed")
        synced = await experiments.sync_experiment_status(session, experiment.id)
        assert synced.status == "completed"
        assert synced.completed_at is not None


def test_derive_experiment_status_pure_rules() -> None:
    now = utc_now()
    experiment_id = new_id()

    queued = Run(
        id=new_id(),
        experiment_id=experiment_id,
        status="queued",
        idempotency_key="a",
        lease_owner=None,
        lease_expires_at=None,
        attempt=0,
        error_detail=None,
        created_at=now,
        started_at=None,
        completed_at=None,
    )
    assert derive_experiment_status(current_status="draft", runs=[queued], trials=[]) == "queued"

    running = Run(
        id=new_id(),
        experiment_id=experiment_id,
        status="running",
        idempotency_key="b",
        lease_owner="w",
        lease_expires_at=now + timedelta(seconds=10),
        attempt=1,
        error_detail=None,
        created_at=now,
        started_at=now,
        completed_at=None,
    )
    assert derive_experiment_status(current_status="queued", runs=[running], trials=[]) == "running"

    completed_run = Run(
        id=new_id(),
        experiment_id=experiment_id,
        status="completed",
        idempotency_key="c",
        lease_owner=None,
        lease_expires_at=None,
        attempt=1,
        error_detail=None,
        created_at=now,
        started_at=now,
        completed_at=now,
    )
    failed_trial = Trial(
        id=new_id(),
        experiment_id=experiment_id,
        policy_version_id=new_id(),
        scenario_id=new_id(),
        status="failed",
        terminal_outcome="system_error",
        success=None,
        collision_count=None,
        collision_max_force_newtons=None,
        duration_seconds=None,
        action_count=None,
        started_at=now,
        completed_at=now,
        simulator_metadata=None,
    )
    assert (
        derive_experiment_status(
            current_status="running",
            runs=[completed_run],
            trials=[failed_trial],
        )
        == "completed_with_errors"
    )


@pytest.mark.asyncio
async def test_trial_events_remain_after_reclaim(
    session_factory: async_sessionmaker[AsyncSession],
) -> None:
    async with session_scope(session_factory) as session:
        experiment, policy, scenario = await _seed_experiment_graph(session)
        await runs.create_queued_run(
            session,
            experiment_id=experiment.id,
            idempotency_key="event-preserve-key",
        )
        claimed = await runs.claim_next_run(
            session,
            worker_id="event-worker",
            lease_seconds=LEASE_SECONDS,
            max_attempts=MAX_ATTEMPTS,
        )
        assert claimed is not None
        trial, _ = await trial_repo.create_trial_if_absent(
            session,
            experiment_id=experiment.id,
            policy_version_id=policy.id,
            scenario_id=scenario.id,
        )
        await event_repo.add_trial_event(
            session,
            TrialEvent(
                trial_id=trial.id,
                ordinal=0,
                timestamp_offset_seconds=Decimal("0"),
                event_type="state_start",
                controller_state="reset",
                action=None,
                observation={"phase": "initial"},
            ),
        )
        await session.execute(
            text(
                """
                UPDATE runs
                SET lease_expires_at = :expired
                WHERE id = :run_id
                """
            ),
            {
                "expired": utc_now() - timedelta(seconds=2),
                "run_id": claimed.id,
            },
        )
        trial_id = trial.id

    async with session_scope(session_factory) as session:
        reclaimed = await runs.claim_next_run(
            session,
            worker_id="event-recovery",
            lease_seconds=LEASE_SECONDS,
            max_attempts=MAX_ATTEMPTS,
        )
        assert reclaimed is not None
        preserved_events = await event_repo.list_trial_events(session, trial_id)
        assert len(preserved_events) == 1
        assert preserved_events[0].observation == {"phase": "initial"}
