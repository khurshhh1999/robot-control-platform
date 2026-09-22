"""Simulator process entrypoint."""

from __future__ import annotations

import asyncio
import sys
import threading

from robot_control_platform_common.artifacts.base import ArtifactStoreError
from robot_control_platform_common.artifacts.filesystem import FilesystemArtifactStore
from robot_control_platform_common.config import ConfigurationError, load_settings
from robot_control_platform_common.db.session import create_engine, create_session_factory
from robot_control_platform_common.logging import configure_logging, get_logger

from robot_control_platform_simulator.trial_execution import PhysicsTrialExecutor
from robot_control_platform_simulator.worker import (
    HEARTBEAT_PATH,
    DatabaseUnavailable,
    SimulatorWorker,
    clear_liveness,
    install_worker_signals,
    write_liveness,
)


def main() -> None:
    """Validate configuration, then run the simulator worker until signaled."""

    try:
        settings = load_settings()
    except ConfigurationError as exc:
        print(f"startup failed: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc

    configure_logging("simulator", settings.log_level.value)
    logger = get_logger("simulator")
    stop_event = threading.Event()
    install_worker_signals(stop_event)
    write_liveness(HEARTBEAT_PATH)
    try:
        store = FilesystemArtifactStore(settings.artifact_root)
    except (ArtifactStoreError, OSError):
        logger.error("artifact_store_unavailable")
        clear_liveness(HEARTBEAT_PATH)
        raise SystemExit(1) from None

    engine = create_engine(settings)
    worker = SimulatorWorker(
        settings=settings,
        session_factory=create_session_factory(engine),
        store=store,
        executor=PhysicsTrialExecutor(gui=settings.simulation_gui),
        stop_event=stop_event,
        liveness_path=HEARTBEAT_PATH,
    )

    async def _run() -> None:
        try:
            await worker.run()
        finally:
            clear_liveness(HEARTBEAT_PATH)
            await engine.dispose()

    try:
        asyncio.run(_run())
    except DatabaseUnavailable:
        logger.error("database_unavailable")
        raise SystemExit(1) from None
