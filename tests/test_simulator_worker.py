"""Simulator worker lifecycle tests.

Covers a normal sequential run, cancellation between controller states,
physics failures, database loss, corrupt artifacts, and termination recovery.
Physics is injected so these checks do not require a PyBullet process.
"""

from __future__ import annotations

import asyncio
import shutil
import signal
import socket
import subprocess
import threading
import time
from collections.abc import AsyncIterator, Awaitable, Callable, Iterator
from pathlib import Path
from typing import cast
from uuid import UUID

import pytest
from alembic import command
from alembic.config import Config
from PIL import Image
from robot_control_platform_common.artifacts.base import ArtifactStore, artifact_storage_key
from robot_control_platform_common.artifacts.filesystem import FilesystemArtifactStore
from robot_control_platform_common.config import load_settings
from robot_control_platform_common.db.models import (
    Experiment,
    ExperimentPolicy,
    PolicyVersion,
    Run,
    Scenario,
    ScenarioSet,
    Trial,
)
from robot_control_platform_common.db.repositories import artifacts as artifact_repo
from robot_control_platform_common.db.repositories import events as event_repo
from robot_control_platform_common.db.repositories import experiments, policies, runs
from robot_control_platform_common.db.repositories import scenarios as scenario_repo
from robot_control_platform_common.db.session import create_session_factory, session_scope
from robot_control_platform_common.ids import new_id
from robot_control_platform_common.logging import configure_logging
from robot_control_platform_common.time import utc_now
from robot_control_platform_simulator.policies.base import default_fixed_policy_config
from robot_control_platform_simulator.trial_execution import (
    ControlProbe,
    ExecutionSignal,
    PhysicsTrialExecutor,
    RecordedEvent,
    StoredArtifact,
    TrialCancelled,
    TrialContext,
    TrialExecution,
    TrialExecutionFailure,
    TrialInterrupted,
)
from robot_control_platform_simulator.worker import (
    RUN_ERROR_BUDGET_EXCEEDED,
    DatabaseUnavailable,
    SimulatorWorker,
    idle_backoff_seconds,
    install_worker_signals,
)
from sqlalchemy import create_engine, func, select, text
from sqlalchemy.ext.asyncio import AsyncSession, async_sessionmaker, create_async_engine

REPO_ROOT = Path(__file__).resolve().parents[1]
DB_DIR = REPO_ROOT / "db"
ALEMBIC_INI = DB_DIR / "alembic.ini"
POSTGRES_IMAGE = (
    "postgres:16-bookworm@sha256:60f4761b9035e0b8d5218f701a8c3382f641bf12b1604822574cf5be3baeb537"
)
CHECKSUM = "a" * 64
SCENARIO_CHECKSUM = "c" * 64
FINGERPRINT = "d" * 64
_KINDS = (
    "initial_rgb",
    "pre_grasp_rgb",
    "post_grasp_rgb",
    "pre_release_rgb",
    "terminal_rgb",
    "trajectory",
    "trial_manifest",
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


def _png_bytes() -> bytes:
    from io import BytesIO

    buffer = BytesIO()
    Image.new("RGB", (8, 8), (12, 80, 40)).save(buffer, format="PNG")
    return buffer.getvalue()


class ScriptedTrialExecutor:
    """Walk controller states and optionally fail before artifact finalization."""

    def __init__(
        self,
        *,
        behavior: str = "success",
        states: tuple[str, ...] = ("reset", "observe"),
        pause_seconds: float = 0.0,
        on_after_state: Callable[[TrialContext, str], Awaitable[None]] | None = None,
    ) -> None:
        self.behavior = behavior
        self.states = states
        self.pause_seconds = pause_seconds
        self.on_after_state = on_after_state
        self.entered_finalize = False
        self.executed: list[tuple[int, int]] = []
        self.probe_calls_during_finalize = 0

    async def execute(
        self,
        context: TrialContext,
        store: ArtifactStore,
        probe: ControlProbe,
    ) -> TrialExecution:
        events: list[RecordedEvent] = []
        self.executed.append((context.execution_order, context.scenario_ordinal))
        for state in self.states:
            signal = await probe.before_state(state)
            if signal is ExecutionSignal.STOP:
                raise TrialInterrupted(tuple(events))
            if signal is ExecutionSignal.CANCEL:
                raise TrialCancelled(tuple(events))
            events.append(
                RecordedEvent(
                    ordinal=len(events),
                    event_type="observation",
                    controller_state=state if state != "reset" else "reset",
                    simulation_time_seconds=float(len(events)),
                    detail="initial_state" if not events else "state_advanced",
                )
            )
            if self.on_after_state is not None:
                await self.on_after_state(context, state)
            if self.pause_seconds > 0:
                await asyncio.sleep(self.pause_seconds)
        if self.behavior == "physics":
            raise TrialExecutionFailure(
                "/tmp/secret.urdf traceback pybullet",
                partial_events=tuple(events),
            )
        if self.behavior == "corrupt":
            key = artifact_storage_key(context.experiment_id, context.trial_id, "initial_rgb")
            store.write(key, _png_bytes())
            raise TrialExecutionFailure(
                "simulation_error",
                partial_events=tuple(events),
                storage_keys=(key,),
            )
        self.entered_finalize = True
        payload = _png_bytes()
        artifacts: list[StoredArtifact] = []
        for kind in _KINDS:
            metadata = store.write(
                artifact_storage_key(context.experiment_id, context.trial_id, kind),
                payload,
            )
            width = 8 if kind.endswith("_rgb") else None
            height = 8 if kind.endswith("_rgb") else None
            artifacts.append(StoredArtifact(metadata=metadata, width_px=width, height_px=height))
        return TrialExecution(
            outcome="success",
            collision_count=0,
            collision_max_force_newtons=0.0,
            duration_seconds=0.4,
            action_count=len(self.states),
            events=tuple(events),
            artifacts=tuple(artifacts),
        )


@pytest.fixture(autouse=True)
def _restore_simulator_logging() -> Iterator[None]:
    """Rebind structlog after pytest closes a captured stdout stream."""

    yield
    configure_logging("simulator", "INFO")


@pytest.fixture(scope="module")
def postgres_url() -> Iterator[str]:
    if not _docker_available():
        pytest.fail("docker is required for simulator worker checks")

    port = _free_port()
    password = "worker-check-password"
    user = "robot_app"
    database = "robot_platform"
    container = f"rcp-worker-check-{port}"
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
        subprocess.run(["docker", "stop", container], check=False, capture_output=True, text=True)


@pytest.fixture(scope="module")
def monkeypatch_module() -> Iterator[pytest.MonkeyPatch]:
    with pytest.MonkeyPatch.context() as patcher:
        yield patcher


@pytest.fixture(scope="module")
def migrated_postgres_url(postgres_url: str, monkeypatch_module: pytest.MonkeyPatch) -> str:
    without_scheme = postgres_url.split("://", 1)[1]
    credentials, host_part = without_scheme.split("@", 1)
    user, password = credentials.split(":", 1)
    host_port, database = host_part.split("/", 1)
    host, port_text = host_port.rsplit(":", 1)
    monkeypatch_module.setenv("RCP_ENV", "test")
    monkeypatch_module.setenv("RCP_LOG_LEVEL", "INFO")
    monkeypatch_module.setenv("RCP_DATABASE_HOST", host)
    monkeypatch_module.setenv("RCP_DATABASE_PORT", port_text)
    monkeypatch_module.setenv("RCP_DATABASE_NAME", database)
    monkeypatch_module.setenv("RCP_DATABASE_USER", user)
    monkeypatch_module.setenv("RCP_DATABASE_PASSWORD", password)
    monkeypatch_module.setenv("RCP_ARTIFACT_ROOT", "/tmp/rcp-worker-artifacts")
    monkeypatch_module.setenv("RCP_API_BASE_URL", "http://127.0.0.1:8000")
    monkeypatch_module.setenv("RCP_WORKER_ID", "simulator-worker-test")
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


def _worker(
    factory: async_sessionmaker[AsyncSession],
    store: FilesystemArtifactStore,
    executor: ScriptedTrialExecutor,
    *,
    stop_event: threading.Event | None = None,
    liveness_path: Path,
    system_error_budget: int | None = None,
    heartbeat_interval_seconds: float = 10.0,
    phase_probe: Callable[[str], None] | None = None,
    idle_sleep: Callable[[float], object] | None = None,
) -> SimulatorWorker:
    async def _idle(seconds: float) -> None:
        if idle_sleep is not None:
            result = idle_sleep(seconds)
            if asyncio.iscoroutine(result):
                await result

    return SimulatorWorker(
        settings=load_settings(),
        session_factory=factory,
        store=store,
        executor=executor,
        stop_event=stop_event,
        liveness_path=liveness_path,
        system_error_budget=system_error_budget,
        heartbeat_interval_seconds=heartbeat_interval_seconds,
        idle_sleep=_idle if idle_sleep is not None else None,
        phase_probe=phase_probe,
    )


async def _seed(
    factory: async_sessionmaker[AsyncSession],
    *,
    policy_count: int = 1,
    scenario_count: int = 1,
) -> UUID:
    now = utc_now()
    names = ("v1_fixed", "v2_pose_aware", "v3_collision_aware")
    async with session_scope(factory) as session:
        leftover = await runs.list_claimable_or_leased_runs(session)
        for leftover_run in leftover:
            if leftover_run.status == "queued":
                await runs.request_run_cancellation(session, leftover_run.id)
    async with session_scope(factory) as session:
        scenario_set = ScenarioSet(
            id=new_id(),
            name=f"worker-set-{new_id()}",
            generator_version="1",
            scenario_count=scenario_count,
            seed_manifest={"seeds": list(range(scenario_count))},
            scene_config={"scene": "baseline"},
            checksum=CHECKSUM,
        )
        await scenario_repo.add_scenario_set(session, scenario_set)
        policy_rows: list[PolicyVersion] = []
        for index in range(policy_count):
            policy = PolicyVersion(
                id=new_id(),
                name=names[index],
                semantic_version=str(new_id()),
                description="worker test policy",
                config={"name": names[index]},
                config_sha256=CHECKSUM,
                source_revision="test-revision",
                container_image_digest="sha256:" + "b" * 64,
                created_at=now,
            )
            await policies.add_policy_version(session, policy)
            policy_rows.append(policy)
        bins = ("bin_red", "bin_green", "bin_blue", "bin_yellow")
        for ordinal in range(scenario_count):
            scenario = Scenario(
                id=new_id(),
                scenario_set_id=scenario_set.id,
                ordinal=ordinal,
                seed=1000 + ordinal,
                object_name=f"parcel-{ordinal}",
                object_category="parcel",
                target_bin=bins[ordinal % len(bins)],
                initial_pose={
                    "position_m": [0.5, 0.0, 0.05],
                    "orientation_xyzw": [0, 0, 0, 1],
                },
                physical_properties={"mass_kg": 0.1, "friction": 0.8, "shape": "box"},
                checksum=SCENARIO_CHECKSUM,
            )
            await scenario_repo.add_scenario(session, scenario)
        experiment = Experiment(
            id=new_id(),
            name=f"worker-experiment-{new_id()}",
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
        for index, policy in enumerate(policy_rows):
            await experiments.add_experiment_policy(
                session,
                ExperimentPolicy(
                    experiment_id=experiment.id,
                    policy_version_id=policy.id,
                    execution_order=index,
                ),
            )
        await runs.create_queued_run(
            session,
            experiment_id=experiment.id,
            idempotency_key=f"worker-{new_id()}",
            request_fingerprint=FINGERPRINT,
        )
        return experiment.id


async def _trials(factory: async_sessionmaker[AsyncSession], experiment_id: UUID) -> list[Trial]:
    async with session_scope(factory) as session:
        return await experiments.list_trials_for_experiment(session, experiment_id)


async def _run_for(factory: async_sessionmaker[AsyncSession], experiment_id: UUID) -> Run:
    async with session_scope(factory) as session:
        found = await experiments.list_runs_for_experiment(session, experiment_id)
        assert len(found) == 1
        return found[0]


def test_idle_backoff_is_bounded() -> None:
    assert idle_backoff_seconds(0) == pytest.approx(0.25)
    assert idle_backoff_seconds(1) == pytest.approx(0.5)
    assert idle_backoff_seconds(2) == pytest.approx(1.0)
    assert idle_backoff_seconds(20) == pytest.approx(5.0)


def test_signal_handler_requests_stop() -> None:
    previous_term = signal.getsignal(signal.SIGTERM)
    previous_int = signal.getsignal(signal.SIGINT)
    stop = threading.Event()
    try:
        install_worker_signals(stop)
        handler = signal.getsignal(signal.SIGTERM)
        assert callable(handler)
        cast(Callable[[int, object | None], object], handler)(signal.SIGTERM, None)
        assert stop.is_set()
    finally:
        signal.signal(signal.SIGTERM, previous_term)
        signal.signal(signal.SIGINT, previous_int)


@pytest.mark.asyncio
async def test_physics_executor_sanitizes_missing_engine(tmp_path: Path) -> None:
    try:
        import pybullet  # noqa: F401
    except ImportError:
        pybullet = None
    if pybullet is not None:
        pytest.skip("PyBullet is installed; this check covers the missing-engine path")
    config = default_fixed_policy_config()
    context = TrialContext(
        experiment_id=new_id(),
        run_id=new_id(),
        trial_id=new_id(),
        policy_version_id=new_id(),
        scenario_id=new_id(),
        execution_order=0,
        scenario_ordinal=0,
        scenario_seed=1,
        policy_name="v1_fixed",
        policy_config=dict(config.to_checksum_payload()),
        policy_config_sha256=config.sha256_hex(),
        source_revision="test-revision",
        scenario_checksum=SCENARIO_CHECKSUM,
        generator_version="1",
        object_name="parcel",
        object_category="parcel",
        target_bin="bin_red",
        initial_pose={"position_m": [0.5, 0.0, 0.05], "orientation_xyzw": [0, 0, 0, 1]},
        physical_properties={"mass_kg": 0.1, "friction": 0.8, "shape": "box"},
    )

    class _Probe:
        async def before_state(self, controller_state: str) -> ExecutionSignal:
            _ = controller_state
            return ExecutionSignal.CONTINUE

    store = FilesystemArtifactStore(tmp_path)
    with pytest.raises(TrialExecutionFailure) as raised:
        await PhysicsTrialExecutor(gui=False).execute(context, store, _Probe())
    assert raised.value.code == "simulation_error"
    assert "pybullet" not in str(raised.value).lower()
    assert "/" not in str(raised.value)


@pytest.mark.asyncio
async def test_worker_resets_idle_backoff_after_claim(
    session_factory: async_sessionmaker[AsyncSession],
    tmp_path: Path,
) -> None:
    delays: list[float] = []
    created = False
    stop = threading.Event()

    async def idle(seconds: float) -> None:
        nonlocal created
        delays.append(seconds)
        if len(delays) == 2 and not created:
            created = True
            await _seed(session_factory, policy_count=1, scenario_count=1)
        if len(delays) >= 4:
            stop.set()

    store = FilesystemArtifactStore(tmp_path / "artifacts")
    worker = _worker(
        session_factory,
        store,
        ScriptedTrialExecutor(),
        stop_event=stop,
        liveness_path=tmp_path / "healthy",
        idle_sleep=idle,
    )
    result = await worker.run()
    assert delays[:3] == pytest.approx([0.25, 0.5, 0.25])
    assert result.runs_processed == 1
    assert not (tmp_path / "healthy").exists()


@pytest.mark.asyncio
async def test_worker_executes_trials_in_policy_then_scenario_order(
    session_factory: async_sessionmaker[AsyncSession],
    tmp_path: Path,
) -> None:
    configure_logging("simulator", "INFO")
    experiment_id = await _seed(session_factory, policy_count=2, scenario_count=2)
    executor = ScriptedTrialExecutor(pause_seconds=0.05)
    liveness = tmp_path / "healthy"
    store = FilesystemArtifactStore(tmp_path / "artifacts")
    worker = _worker(
        session_factory,
        store,
        executor,
        liveness_path=liveness,
        heartbeat_interval_seconds=0.01,
    )
    result = await worker.run(max_runs=1)
    assert result.database_lost is False
    assert executor.executed == [(0, 0), (0, 1), (1, 0), (1, 1)]
    assert executor.entered_finalize is True
    assert worker.heartbeat_count >= 1
    assert worker.max_concurrent_sessions == 1
    assert not liveness.exists()
    trials = await _trials(session_factory, experiment_id)
    assert len(trials) == 4
    assert {trial.status for trial in trials} == {"completed"}
    assert {trial.terminal_outcome for trial in trials} == {"success"}
    run = await _run_for(session_factory, experiment_id)
    assert run.status == "completed"
    async with session_scope(session_factory) as session:
        experiment = await experiments.get_experiment(session, experiment_id)
        assert experiment.status == "completed"
        for trial in trials:
            rows = await artifact_repo.list_artifacts_for_trial(session, trial.id)
            assert len(rows) == len(_KINDS)
            events = await event_repo.list_trial_events(session, trial.id)
            assert events
            assert events[0].event_type == "observation"


@pytest.mark.asyncio
async def test_cancellation_between_states_skips_finalization(
    session_factory: async_sessionmaker[AsyncSession],
    tmp_path: Path,
) -> None:
    experiment_id = await _seed(session_factory, scenario_count=2)
    run = await _run_for(session_factory, experiment_id)

    async def cancel_after_first(_context: TrialContext, state: str) -> None:
        if state != "reset":
            return
        async with session_scope(session_factory) as session:
            await runs.request_run_cancellation(session, run.id)

    executor = ScriptedTrialExecutor(states=("reset", "observe"), on_after_state=cancel_after_first)
    worker = _worker(
        session_factory,
        FilesystemArtifactStore(tmp_path / "artifacts"),
        executor,
        liveness_path=tmp_path / "healthy",
    )
    await worker.run(max_runs=1)
    assert executor.entered_finalize is False
    trials = await _trials(session_factory, experiment_id)
    assert len(trials) == 2
    assert {trial.status for trial in trials} == {"cancelled"}
    assert {trial.terminal_outcome for trial in trials} == {None}
    finished = await _run_for(session_factory, experiment_id)
    assert finished.status == "cancelled"


@pytest.mark.asyncio
async def test_finalization_completes_after_cancellation_is_requested(
    session_factory: async_sessionmaker[AsyncSession],
    tmp_path: Path,
) -> None:
    experiment_id = await _seed(session_factory, scenario_count=1)
    run = await _run_for(session_factory, experiment_id)

    async def cancel_before_finalize(_context: TrialContext, state: str) -> None:
        if state != "reset":
            return
        async with session_scope(session_factory) as session:
            await runs.request_run_cancellation(session, run.id)

    executor = ScriptedTrialExecutor(states=("reset",), on_after_state=cancel_before_finalize)
    worker = _worker(
        session_factory,
        FilesystemArtifactStore(tmp_path / "artifacts"),
        executor,
        liveness_path=tmp_path / "healthy",
    )
    await worker.run(max_runs=1)
    assert executor.entered_finalize is True
    assert executor.probe_calls_during_finalize == 0
    trials = await _trials(session_factory, experiment_id)
    assert len(trials) == 1
    assert trials[0].status == "completed"
    assert trials[0].terminal_outcome == "success"
    finished = await _run_for(session_factory, experiment_id)
    assert finished.status == "cancelled"


@pytest.mark.asyncio
async def test_physics_error_is_sanitized_and_stops_at_error_budget(
    session_factory: async_sessionmaker[AsyncSession],
    tmp_path: Path,
) -> None:
    experiment_id = await _seed(session_factory, scenario_count=2)
    executor = ScriptedTrialExecutor(behavior="physics")
    worker = _worker(
        session_factory,
        FilesystemArtifactStore(tmp_path / "artifacts"),
        executor,
        liveness_path=tmp_path / "healthy",
        system_error_budget=1,
    )
    await worker.run(max_runs=1)
    trials = await _trials(session_factory, experiment_id)
    by_outcome = {trial.status: trial for trial in trials}
    assert set(by_outcome) == {"failed", "pending"}
    failed = by_outcome["failed"]
    assert failed.terminal_outcome == "system_error"
    metadata = failed.simulator_metadata
    assert metadata == {
        "schema_version": "1",
        "system_error_code": "simulation_error",
    }
    rendered = str(metadata)
    assert "/tmp" not in rendered
    assert "pybullet" not in rendered
    assert "traceback" not in rendered
    finished = await _run_for(session_factory, experiment_id)
    assert finished.status == "failed"
    assert finished.error_detail == RUN_ERROR_BUDGET_EXCEEDED
    async with session_scope(session_factory) as session:
        events = await event_repo.list_trial_events(session, failed.id)
        assert events
        assert "urdf" not in str(events[0].observation)


@pytest.mark.asyncio
async def test_database_loss_does_not_mark_trial_successful(
    session_factory: async_sessionmaker[AsyncSession],
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    configure_logging("simulator", "INFO")
    experiment_id = await _seed(session_factory, scenario_count=1)

    def probe(phase: str) -> None:
        if phase == "persist_terminal":
            raise DatabaseUnavailable()

    store = FilesystemArtifactStore(tmp_path / "artifacts")
    worker = _worker(
        session_factory,
        store,
        ScriptedTrialExecutor(),
        liveness_path=tmp_path / "healthy",
        phase_probe=probe,
    )
    result = await worker.run(max_runs=1)
    assert result.database_lost is True
    trials = await _trials(session_factory, experiment_id)
    assert len(trials) == 1
    assert trials[0].status == "running"
    assert trials[0].terminal_outcome is None
    async with session_scope(session_factory) as session:
        rows = await artifact_repo.list_artifacts_for_trial(session, trials[0].id)
        assert rows == []
    assert store.list_storage_keys()
    finished = await _run_for(session_factory, experiment_id)
    assert finished.status == "queued"
    captured = capsys.readouterr()
    combined = captured.out + captured.err
    assert "worker-check-password" not in combined
    assert "postgresql://" not in combined


@pytest.mark.asyncio
async def test_corrupt_artifact_is_quarantined_and_trial_fails(
    session_factory: async_sessionmaker[AsyncSession],
    tmp_path: Path,
) -> None:
    experiment_id = await _seed(session_factory, scenario_count=1)
    store = FilesystemArtifactStore(tmp_path / "artifacts")
    worker = _worker(
        session_factory,
        store,
        ScriptedTrialExecutor(behavior="corrupt"),
        liveness_path=tmp_path / "healthy",
    )
    await worker.run(max_runs=1)
    trials = await _trials(session_factory, experiment_id)
    assert len(trials) == 1
    assert trials[0].status == "failed"
    assert trials[0].terminal_outcome == "system_error"
    async with session_scope(session_factory) as session:
        rows = await artifact_repo.list_artifacts_for_trial(session, trials[0].id)
        assert rows == []
    quarantined = [
        path
        for path in store.root.rglob("*")
        if path.is_file() and "_quarantine" in path.as_posix()
    ]
    assert quarantined
    assert store.list_storage_keys() == ()


@pytest.mark.asyncio
async def test_termination_releases_lease_and_recovery_does_not_duplicate(
    session_factory: async_sessionmaker[AsyncSession],
    tmp_path: Path,
) -> None:
    experiment_id = await _seed(session_factory, scenario_count=2)
    stop = threading.Event()

    async def stop_second_trial(context: TrialContext, state: str) -> None:
        if context.scenario_ordinal == 1 and state == "reset":
            stop.set()

    first = ScriptedTrialExecutor(states=("reset", "observe"), on_after_state=stop_second_trial)
    started = time.monotonic()
    worker = _worker(
        session_factory,
        FilesystemArtifactStore(tmp_path / "artifacts"),
        first,
        stop_event=stop,
        liveness_path=tmp_path / "healthy",
    )
    await worker.run()
    elapsed = time.monotonic() - started
    assert elapsed < 2.0
    trials = await _trials(session_factory, experiment_id)
    assert len(trials) == 2
    completed = [trial for trial in trials if trial.status == "completed"]
    interrupted = [trial for trial in trials if trial.status == "running"]
    assert len(completed) == 1
    assert len(interrupted) == 1
    completed_id = completed[0].id
    released = await _run_for(session_factory, experiment_id)
    assert released.status == "queued"
    assert released.lease_owner is None
    second = ScriptedTrialExecutor()
    recovered = _worker(
        session_factory,
        FilesystemArtifactStore(tmp_path / "artifacts"),
        second,
        liveness_path=tmp_path / "healthy-2",
    )
    await recovered.run(max_runs=1)
    again = await _trials(session_factory, experiment_id)
    assert len(again) == 2
    assert {trial.id for trial in again} == {trial.id for trial in trials}
    assert all(trial.status == "completed" for trial in again)
    assert any(trial.id == completed_id and trial.terminal_outcome == "success" for trial in again)
    finished = await _run_for(session_factory, experiment_id)
    assert finished.status == "completed"
    assert finished.attempt == 2
    async with session_scope(session_factory) as session:
        count = await session.scalar(
            select(func.count()).select_from(Trial).where(Trial.experiment_id == experiment_id)
        )
    assert count == 2
