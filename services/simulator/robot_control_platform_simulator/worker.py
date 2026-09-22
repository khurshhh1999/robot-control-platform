"""Simulator worker: claim one run, execute its trials, and preserve evidence."""

from __future__ import annotations

import asyncio
import signal
import threading
from collections.abc import AsyncIterator, Awaitable, Callable
from contextlib import asynccontextmanager
from dataclasses import dataclass
from decimal import Decimal
from pathlib import Path
from typing import Any, Final
from uuid import UUID

from robot_control_platform_common.artifacts.base import ArtifactStore
from robot_control_platform_common.config import Settings
from robot_control_platform_common.db.models import Artifact, TrialEvent
from robot_control_platform_common.db.repositories import artifacts as artifact_repo
from robot_control_platform_common.db.repositories import events as event_repo
from robot_control_platform_common.db.repositories import experiments as experiment_repo
from robot_control_platform_common.db.repositories import policies as policy_repo
from robot_control_platform_common.db.repositories import runs as run_repo
from robot_control_platform_common.db.repositories import scenarios as scenario_repo
from robot_control_platform_common.db.repositories import trials as trial_repo
from robot_control_platform_common.db.repositories.exceptions import (
    InvalidLeaseStateError,
    LeaseOwnershipError,
)
from robot_control_platform_common.db.session import session_scope
from robot_control_platform_common.ids import new_id
from robot_control_platform_common.logging import bind_log_context, clear_log_context, get_logger
from sqlalchemy import text
from sqlalchemy.exc import DBAPIError
from sqlalchemy.ext.asyncio import AsyncSession, async_sessionmaker

from robot_control_platform_simulator.domain.models import DOMAIN_SCHEMA_VERSION
from robot_control_platform_simulator.trial_execution import (
    ExecutionSignal,
    RecordedEvent,
    StoredArtifact,
    TrialCancelled,
    TrialContext,
    TrialExecution,
    TrialExecutionFailure,
    TrialExecutor,
    TrialInterrupted,
)

HEARTBEAT_PATH: Final[Path] = Path("/tmp/robot-platform-healthy")
IDLE_BACKOFF_INITIAL_SECONDS: Final[float] = 0.25
IDLE_BACKOFF_MAX_SECONDS: Final[float] = 5.0
DEFAULT_LEASE_ATTEMPT_LIMIT: Final[int] = 3
RUN_ERROR_BUDGET_EXCEEDED: Final[str] = "system_error_budget_exceeded"
_TERMINAL_TRIAL_STATUSES: Final[frozenset[str]] = frozenset({"completed", "failed", "cancelled"})
_TASK_OUTCOMES: Final[frozenset[str]] = frozenset(
    {"success", "missed_grasp", "dropped_object", "collision", "wrong_bin"}
)

PhaseProbe = Callable[[str], None]
IdleSleep = Callable[[float], Awaitable[None]]


class DatabaseUnavailable(Exception):
    """The database could not be used. The message stays free of DSNs and paths."""

    def __init__(self) -> None:
        super().__init__("database unavailable")


@dataclass(frozen=True)
class WorkerResult:
    """Summary of one worker process loop."""

    runs_processed: int
    database_lost: bool
    stop_requested: bool


@dataclass(frozen=True)
class _TrialSlot:
    trial_id: UUID
    execution_order: int
    scenario_ordinal: int
    scenario_seed: int
    policy_version_id: UUID
    scenario_id: UUID
    policy_name: str
    policy_config: dict[str, Any]
    policy_config_sha256: str
    scenario_checksum: str
    generator_version: str
    object_name: str
    object_category: str
    target_bin: str
    initial_pose: dict[str, Any]
    physical_properties: dict[str, Any]
    source_revision: str


def idle_backoff_seconds(idle_polls: int) -> float:
    """Return a bounded exponential delay. ``idle_polls`` starts at zero."""

    if isinstance(idle_polls, bool) or not isinstance(idle_polls, int) or idle_polls < 0:
        msg = "idle_polls must be a nonnegative integer"
        raise ValueError(msg)
    delay = IDLE_BACKOFF_INITIAL_SECONDS * (2**idle_polls)
    return float(min(delay, IDLE_BACKOFF_MAX_SECONDS))


def write_liveness(path: Path) -> None:
    """Record that the worker loop is alive for the compose health check."""

    path.write_text("ok\n", encoding="utf-8")


def clear_liveness(path: Path) -> None:
    """Remove the liveness file during shutdown."""

    path.unlink(missing_ok=True)


def install_worker_signals(stop_event: threading.Event) -> None:
    """Request a graceful stop on SIGTERM or SIGINT."""

    def _handle(_signum: int, _frame: object | None) -> None:
        stop_event.set()

    signal.signal(signal.SIGTERM, _handle)
    signal.signal(signal.SIGINT, _handle)


class SimulatorWorker:
    """Poll the run queue, execute trials sequentially, and heartbeat the lease."""

    def __init__(
        self,
        *,
        settings: Settings,
        session_factory: async_sessionmaker[AsyncSession],
        store: ArtifactStore,
        executor: TrialExecutor,
        stop_event: threading.Event | None = None,
        liveness_path: Path = HEARTBEAT_PATH,
        system_error_budget: int | None = None,
        lease_attempt_limit: int = DEFAULT_LEASE_ATTEMPT_LIMIT,
        idle_sleep: IdleSleep | None = None,
        phase_probe: PhaseProbe | None = None,
        heartbeat_interval_seconds: float | None = None,
        lease_seconds: int | None = None,
    ) -> None:
        if system_error_budget is not None and system_error_budget < 1:
            msg = "system_error_budget must be at least 1"
            raise ValueError(msg)
        if lease_attempt_limit < 1:
            msg = "lease_attempt_limit must be at least 1"
            raise ValueError(msg)
        self._settings = settings
        self._session_factory = session_factory
        self._store = store
        self._executor = executor
        self._stop = stop_event if stop_event is not None else threading.Event()
        self._liveness_path = liveness_path
        self._system_error_budget = system_error_budget
        self._lease_attempt_limit = lease_attempt_limit
        self._idle_sleep = idle_sleep
        self._phase_probe = phase_probe
        self._heartbeat_interval = (
            heartbeat_interval_seconds
            if heartbeat_interval_seconds is not None
            else float(settings.run_heartbeat_seconds)
        )
        self._lease_seconds = (
            lease_seconds if lease_seconds is not None else settings.run_lease_seconds
        )
        if self._heartbeat_interval <= 0 or self._lease_seconds < 1:
            msg = "lease timing must be positive"
            raise ValueError(msg)
        self._db_lock = asyncio.Lock()
        self._heartbeat_stop = asyncio.Event()
        self._database_lost = False
        self.heartbeat_count = 0
        self.max_concurrent_sessions = 0
        self._inflight_sessions = 0
        self._logger = get_logger("simulator")

    def request_stop(self) -> None:
        """Ask the loop to finish the current atomic step and release its lease."""

        self._stop.set()

    def stop_requested(self) -> bool:
        """Return whether SIGTERM, SIGINT, or the caller asked the loop to stop."""

        return self._stop.is_set()

    async def run(
        self,
        *,
        max_runs: int | None = None,
        stop_when_idle: bool = False,
    ) -> WorkerResult:
        """Claim runs until stopped. Idle polls use bounded exponential backoff."""

        write_liveness(self._liveness_path)
        runs_processed = 0
        idle_polls = 0
        try:
            await self._ping_database()
            while not self._stop.is_set():
                write_liveness(self._liveness_path)
                claimed = await self._claim_run()
                if claimed is None:
                    if stop_when_idle and runs_processed > 0:
                        break
                    if self._stop.is_set():
                        break
                    await self._sleep_idle(idle_backoff_seconds(idle_polls))
                    idle_polls += 1
                    continue
                idle_polls = 0
                await self._process_run(claimed)
                runs_processed += 1
                if self._database_lost or (max_runs is not None and runs_processed >= max_runs):
                    break
        except DatabaseUnavailable:
            self._database_lost = True
        finally:
            clear_liveness(self._liveness_path)
            clear_log_context()
        return WorkerResult(
            runs_processed=runs_processed,
            database_lost=self._database_lost,
            stop_requested=self._stop.is_set(),
        )

    async def _process_run(self, run_id: UUID) -> None:
        self._heartbeat_stop = asyncio.Event()
        heartbeat = asyncio.create_task(self._heartbeat_loop(run_id))
        release_lease = True
        try:
            experiment_id = await self._mark_run_running(run_id)
            bind_log_context(experiment_id=str(experiment_id), run_id=str(run_id))
            outcome = await self._execute_slots(run_id, experiment_id)
            if outcome == "cancelled":
                await self._complete_run(run_id, experiment_id, "cancelled", None)
                release_lease = False
            elif outcome == "budget":
                await self._complete_run(
                    run_id,
                    experiment_id,
                    "failed",
                    RUN_ERROR_BUDGET_EXCEEDED,
                )
                release_lease = False
            elif outcome == "finished":
                await self._complete_from_trials(run_id, experiment_id)
                release_lease = False
            elif outcome == "database":
                self._database_lost = True
                release_lease = True
        except DatabaseUnavailable:
            self._database_lost = True
            release_lease = True
        finally:
            self._heartbeat_stop.set()
            await heartbeat
            if release_lease:
                await self._release_lease(run_id)

    async def _execute_slots(self, run_id: UUID, experiment_id: UUID) -> str:
        slots = await self._ensure_trials(experiment_id)
        failed_count = await self._failed_trial_count(experiment_id)
        for slot in slots:
            if self._stop.is_set():
                return "stopped"
            status = await self._trial_status(slot.trial_id)
            if status in _TERMINAL_TRIAL_STATUSES:
                continue
            if await self._is_cancelling(run_id):
                await self._cancel_open_trials(experiment_id)
                return "cancelled"
            if self._system_error_budget is not None and failed_count >= self._system_error_budget:
                return "budget"
            bind_log_context(
                experiment_id=str(experiment_id),
                run_id=str(run_id),
                trial_id=str(slot.trial_id),
            )
            try:
                await self._mark_trial_running(slot.trial_id)
                result = await self._executor.execute(
                    _context_for_slot(slot, experiment_id, run_id),
                    self._store,
                    _RunProbe(self, run_id),
                )
            except TrialInterrupted:
                self._logger.info("trial_interrupted", trial_id=str(slot.trial_id))
                return "stopped"
            except TrialCancelled as exc:
                await self._cancel_trial(slot.trial_id, exc.partial_events)
                await self._cancel_open_trials(experiment_id)
                return "cancelled"
            except TrialExecutionFailure as exc:
                await self._quarantine(exc.storage_keys)
                await self._fail_trial(slot.trial_id, exc.code, exc.partial_events)
                failed_count += 1
                if (
                    self._system_error_budget is not None
                    and failed_count >= self._system_error_budget
                ):
                    return "budget"
                continue
            except DatabaseUnavailable:
                return "database"
            if result.outcome == "system_error" or result.system_error_code is not None:
                code = result.system_error_code or "simulation_error"
                await self._fail_trial(slot.trial_id, code, result.events, result.artifacts)
                failed_count += 1
                if (
                    self._system_error_budget is not None
                    and failed_count >= self._system_error_budget
                ):
                    return "budget"
                continue
            await self._commit_success(slot.trial_id, result)
        if await self._is_cancelling(run_id):
            await self._cancel_open_trials(experiment_id)
            return "cancelled"
        return "finished"

    async def _heartbeat_loop(self, run_id: UUID) -> None:
        while not self._heartbeat_stop.is_set():
            try:
                await asyncio.wait_for(
                    self._heartbeat_stop.wait(),
                    timeout=self._heartbeat_interval,
                )
            except TimeoutError:
                pass
            if self._heartbeat_stop.is_set():
                return
            try:
                async with self._db("heartbeat") as session:
                    await run_repo.heartbeat_run(
                        session,
                        run_id=run_id,
                        worker_id=self._settings.worker_id,
                        lease_seconds=self._lease_seconds,
                    )
                self.heartbeat_count += 1
            except (InvalidLeaseStateError, LeaseOwnershipError):
                return
            except DatabaseUnavailable:
                self._database_lost = True
                self.request_stop()
                return
            except Exception as exc:
                self._logger.error("heartbeat_failed", error_type=type(exc).__name__)
                self.request_stop()
                return

    async def _claim_run(self) -> UUID | None:
        async with self._db("claim") as session:
            run = await run_repo.claim_next_run(
                session,
                worker_id=self._settings.worker_id,
                lease_seconds=self._lease_seconds,
                max_attempts=self._lease_attempt_limit,
            )
            if run is None:
                return None
            await experiment_repo.sync_experiment_status(session, run.experiment_id)
            self._logger.info(
                "run_claimed", run_id=str(run.id), experiment_id=str(run.experiment_id)
            )
            return run.id

    async def _mark_run_running(self, run_id: UUID) -> UUID:
        async with self._db("mark_running") as session:
            run = await run_repo.mark_run_running(session, run_id)
            await experiment_repo.sync_experiment_status(session, run.experiment_id)
            return run.experiment_id

    async def _ensure_trials(self, experiment_id: UUID) -> list[_TrialSlot]:
        async with self._db("create_trials") as session:
            experiment = await experiment_repo.get_experiment(session, experiment_id)
            scenario_set = await scenario_repo.get_scenario_set(session, experiment.scenario_set_id)
            memberships = await experiment_repo.list_policies_for_experiment(session, experiment_id)
            scenarios = await scenario_repo.list_scenarios_for_set(
                session, experiment.scenario_set_id
            )
            slots: list[_TrialSlot] = []
            for membership in memberships:
                policy = await policy_repo.get_policy_version(session, membership.policy_version_id)
                for scenario in scenarios:
                    trial, _created = await trial_repo.create_trial_if_absent(
                        session,
                        experiment_id=experiment_id,
                        policy_version_id=policy.id,
                        scenario_id=scenario.id,
                    )
                    slots.append(
                        _TrialSlot(
                            trial_id=trial.id,
                            execution_order=membership.execution_order,
                            scenario_ordinal=scenario.ordinal,
                            scenario_seed=scenario.seed,
                            policy_version_id=policy.id,
                            scenario_id=scenario.id,
                            policy_name=policy.name,
                            policy_config=dict(policy.config),
                            policy_config_sha256=policy.config_sha256,
                            scenario_checksum=scenario.checksum,
                            generator_version=scenario_set.generator_version,
                            object_name=scenario.object_name,
                            object_category=scenario.object_category,
                            target_bin=scenario.target_bin,
                            initial_pose=dict(scenario.initial_pose),
                            physical_properties=dict(scenario.physical_properties),
                            source_revision=experiment.source_revision,
                        )
                    )
            return slots

    async def _trial_status(self, trial_id: UUID) -> str:
        async with self._db("read_trial") as session:
            trial = await trial_repo.get_trial(session, trial_id)
            return trial.status

    async def _failed_trial_count(self, experiment_id: UUID) -> int:
        async with self._db("read_trials") as session:
            trials = await experiment_repo.list_trials_for_experiment(session, experiment_id)
            return sum(1 for trial in trials if trial.status == "failed")

    async def _is_cancelling(self, run_id: UUID) -> bool:
        async with self._db("read_run") as session:
            run = await run_repo.get_run(session, run_id)
            return run.status == "cancelling"

    async def _mark_trial_running(self, trial_id: UUID) -> None:
        async with self._db("mark_trial_running") as session:
            trial = await trial_repo.get_trial(session, trial_id)
            if trial.status == "pending":
                await trial_repo.mark_trial_running(session, trial_id)
            self._logger.info("trial_started", trial_id=str(trial_id))

    async def _commit_success(self, trial_id: UUID, result: TrialExecution) -> None:
        outcome = result.outcome if result.outcome in _TASK_OUTCOMES else "system_error"
        if outcome == "system_error":
            await self._fail_trial(
                trial_id, "evaluation_unavailable", result.events, result.artifacts
            )
            return
        async with self._db("persist_terminal") as session:
            await _insert_events(session, trial_id, result.events)
            await _insert_artifacts(session, trial_id, result)
            await trial_repo.mark_trial_completed(
                session,
                trial_id=trial_id,
                terminal_outcome=outcome,
                success=outcome == "success",
                collision_count=result.collision_count,
                collision_max_force_newtons=_decimal(result.collision_max_force_newtons),
                duration_seconds=_decimal(result.duration_seconds),
                action_count=result.action_count,
                simulator_metadata=_success_metadata(result),
            )
        self._logger.info("trial_completed", trial_id=str(trial_id), terminal_outcome=outcome)

    async def _fail_trial(
        self,
        trial_id: UUID,
        code: str,
        events: tuple[RecordedEvent, ...] = (),
        artifacts: tuple[StoredArtifact, ...] = (),
    ) -> None:
        allowed = {"simulation_error", "timeout_unevaluable", "evaluation_unavailable"}
        safe_code = code if code in allowed else "simulation_error"
        async with self._db("persist_failure") as session:
            await _insert_events(session, trial_id, events)
            if artifacts:
                await _insert_artifacts(
                    session,
                    trial_id,
                    TrialExecution(
                        outcome="system_error",
                        collision_count=0,
                        collision_max_force_newtons=0.0,
                        duration_seconds=0.0,
                        action_count=0,
                        events=events,
                        artifacts=artifacts,
                        system_error_code=safe_code,
                    ),
                )
            await trial_repo.mark_trial_failed(
                session,
                trial_id=trial_id,
                simulator_metadata={
                    "schema_version": DOMAIN_SCHEMA_VERSION,
                    "system_error_code": safe_code,
                },
            )
        self._logger.error("trial_failed", trial_id=str(trial_id), system_error_code=safe_code)

    async def _cancel_trial(self, trial_id: UUID, events: tuple[RecordedEvent, ...]) -> None:
        async with self._db("persist_cancel") as session:
            await _insert_events(session, trial_id, events)
            await trial_repo.mark_trial_cancelled(session, trial_id)

    async def _cancel_open_trials(self, experiment_id: UUID) -> None:
        async with self._db("cancel_trials") as session:
            trials = await experiment_repo.list_trials_for_experiment(session, experiment_id)
            for trial in trials:
                if trial.status in {"pending", "running"}:
                    await trial_repo.mark_trial_cancelled(session, trial.id)

    async def _complete_run(
        self,
        run_id: UUID,
        experiment_id: UUID,
        status: str,
        error_detail: str | None,
    ) -> None:
        async with self._db("complete_run") as session:
            await run_repo.complete_run(
                session,
                run_id=run_id,
                status=status,
                error_detail=error_detail,
            )
            await experiment_repo.sync_experiment_status(session, experiment_id)
        self._logger.info("run_completed", run_id=str(run_id), status=status)

    async def _complete_from_trials(self, run_id: UUID, experiment_id: UUID) -> None:
        async with self._db("complete_run") as session:
            trials = await experiment_repo.list_trials_for_experiment(session, experiment_id)
            if any(trial.status == "failed" for trial in trials):
                status = "completed_with_errors"
            else:
                status = "completed"
            await run_repo.complete_run(session, run_id=run_id, status=status, error_detail=None)
            await experiment_repo.sync_experiment_status(session, experiment_id)
        self._logger.info("run_completed", run_id=str(run_id), status=status)

    async def _release_lease(self, run_id: UUID) -> None:
        try:
            async with self._db("release") as session:
                run = await run_repo.get_run(session, run_id)
                if run.lease_owner != self._settings.worker_id:
                    return
                if run.status == "cancelling":
                    trials = await experiment_repo.list_trials_for_experiment(
                        session, run.experiment_id
                    )
                    for trial in trials:
                        if trial.status in {"pending", "running"}:
                            await trial_repo.mark_trial_cancelled(session, trial.id)
                released = await run_repo.release_owned_lease(
                    session,
                    run_id=run_id,
                    worker_id=self._settings.worker_id,
                )
                await experiment_repo.sync_experiment_status(session, released.experiment_id)
        except (DatabaseUnavailable, LeaseOwnershipError, InvalidLeaseStateError):
            self._logger.error("lease_release_failed", run_id=str(run_id))

    async def _ping_database(self) -> None:
        async with self._db("ping") as session:
            await session.execute(text("SELECT 1"))

    async def _quarantine(self, storage_keys: tuple[str, ...]) -> None:
        for key in storage_keys:
            try:
                self._store.quarantine(key, reason="checksum_mismatch")
            except Exception as exc:
                self._logger.error(
                    "artifact_quarantine_failed",
                    error_type=type(exc).__name__,
                )

    async def _sleep_idle(self, seconds: float) -> None:
        if self._idle_sleep is not None:
            await self._idle_sleep(seconds)
            return
        remaining = seconds
        while remaining > 0 and not self._stop.is_set():
            chunk = min(0.05, remaining)
            await asyncio.sleep(chunk)
            remaining -= chunk

    @asynccontextmanager
    async def _db(self, phase: str) -> AsyncIterator[AsyncSession]:
        if self._phase_probe is not None:
            try:
                self._phase_probe(phase)
            except DatabaseUnavailable:
                self._database_lost = True
                raise
        async with self._db_lock:
            self._inflight_sessions += 1
            self.max_concurrent_sessions = max(
                self.max_concurrent_sessions, self._inflight_sessions
            )
            try:
                try:
                    async with session_scope(self._session_factory) as session:
                        yield session
                except DBAPIError:
                    self._database_lost = True
                    raise DatabaseUnavailable() from None
            finally:
                self._inflight_sessions -= 1


class _RunProbe:
    """Read run cancellation between controller states using its own session."""

    def __init__(self, worker: SimulatorWorker, run_id: UUID) -> None:
        self._worker = worker
        self._run_id = run_id

    async def before_state(self, controller_state: str) -> ExecutionSignal:
        _ = controller_state
        if await self._worker._is_cancelling(self._run_id):
            return ExecutionSignal.CANCEL
        if self._worker.stop_requested():
            return ExecutionSignal.STOP
        return ExecutionSignal.CONTINUE


def _context_for_slot(slot: _TrialSlot, experiment_id: UUID, run_id: UUID) -> TrialContext:
    return TrialContext(
        experiment_id=experiment_id,
        run_id=run_id,
        trial_id=slot.trial_id,
        policy_version_id=slot.policy_version_id,
        scenario_id=slot.scenario_id,
        execution_order=slot.execution_order,
        scenario_ordinal=slot.scenario_ordinal,
        scenario_seed=slot.scenario_seed,
        policy_name=slot.policy_name,
        policy_config=dict(slot.policy_config),
        policy_config_sha256=slot.policy_config_sha256,
        source_revision=slot.source_revision,
        scenario_checksum=slot.scenario_checksum,
        generator_version=slot.generator_version,
        object_name=slot.object_name,
        object_category=slot.object_category,
        target_bin=slot.target_bin,
        initial_pose=dict(slot.initial_pose),
        physical_properties=dict(slot.physical_properties),
    )


def _success_metadata(result: TrialExecution) -> dict[str, object]:
    return {
        "schema_version": DOMAIN_SCHEMA_VERSION,
        "terminal_outcome": result.outcome,
        "system_error_code": result.system_error_code,
    }


def _decimal(value: float) -> Decimal:
    return Decimal(f"{value:.6f}")


async def _insert_events(
    session: AsyncSession,
    trial_id: UUID,
    events: tuple[RecordedEvent, ...],
) -> None:
    existing = await event_repo.list_trial_events(session, trial_id)
    used = {event.ordinal for event in existing}
    for event in events:
        if event.ordinal in used:
            continue
        observation = {"detail": event.detail} if event.detail else None
        await event_repo.add_trial_event(
            session,
            TrialEvent(
                trial_id=trial_id,
                ordinal=event.ordinal,
                timestamp_offset_seconds=_decimal(event.simulation_time_seconds),
                event_type=event.event_type,
                controller_state=event.controller_state,
                action=None,
                observation=observation,
            ),
        )
        used.add(event.ordinal)


async def _insert_artifacts(session: AsyncSession, trial_id: UUID, result: TrialExecution) -> None:
    for stored in result.artifacts:
        metadata = stored.metadata
        await artifact_repo.create_artifact_if_absent(
            session,
            Artifact(
                id=new_id(),
                trial_id=trial_id,
                kind=metadata.kind,
                storage_key=metadata.storage_key,
                media_type=metadata.media_type,
                width_px=stored.width_px,
                height_px=stored.height_px,
                byte_size=metadata.byte_size,
                sha256=metadata.sha256,
                created_at=metadata.created_at,
            ),
        )
