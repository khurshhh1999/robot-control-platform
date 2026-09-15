"""API resource tests for T16 routes, schemas, and service rules."""

from __future__ import annotations

import shutil
import socket
import subprocess
import time
from collections.abc import AsyncIterator, Iterator
from decimal import Decimal
from pathlib import Path
from typing import Any
from uuid import UUID

import pytest
from alembic import command
from alembic.config import Config
from fastapi.testclient import TestClient
from pydantic import SecretStr
from robot_control_platform_api.dependencies import AppRuntime
from robot_control_platform_api.errors import PROBLEM_CONTENT_TYPE, ApiError
from robot_control_platform_api.main import create_app
from robot_control_platform_api.pagination import decode_cursor, encode_cursor
from robot_control_platform_common.artifacts.base import artifact_storage_key, sha256_hex_bytes
from robot_control_platform_common.artifacts.filesystem import FilesystemArtifactStore
from robot_control_platform_common.config import LogLevel, RuntimeEnv, Settings
from robot_control_platform_common.db.models import Artifact, PolicyVersion, TrialEvent
from robot_control_platform_common.db.repositories import artifacts as artifact_repo
from robot_control_platform_common.db.repositories import events as event_repo
from robot_control_platform_common.db.repositories import policies as policy_repo
from robot_control_platform_common.db.repositories import trials as trial_repo
from robot_control_platform_common.db.session import create_session_factory, session_scope
from robot_control_platform_common.ids import new_id
from robot_control_platform_common.time import utc_now
from sqlalchemy import create_engine, text
from sqlalchemy.ext.asyncio import (
    AsyncEngine,
    AsyncSession,
    async_sessionmaker,
    create_async_engine,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
DB_DIR = REPO_ROOT / "db"
ALEMBIC_INI = DB_DIR / "alembic.ini"
POSTGRES_IMAGE = (
    "postgres:16-bookworm@sha256:60f4761b9035e0b8d5218f701a8c3382f641bf12b1604822574cf5be3baeb537"
)
CHECKSUM = "a" * 64


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
        pytest.fail("docker is required for API resource checks")

    port = _free_port()
    password = "api-resource-check-password"
    user = "robot_app"
    database = "robot_platform"
    container = f"rcp-api-resource-check-{port}"
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
    monkeypatch_module.setenv("RCP_ARTIFACT_ROOT", "/tmp/rcp-api-resource-artifacts")
    monkeypatch_module.setenv("RCP_API_BASE_URL", "http://127.0.0.1:8000")
    monkeypatch_module.setenv("RCP_WORKER_ID", "api-resource-check")
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
def artifact_root(tmp_path: Path) -> Path:
    root = tmp_path / "artifacts"
    root.mkdir(parents=True, exist_ok=True)
    return root


@pytest.fixture
async def db_engine(migrated_postgres_url: str) -> AsyncIterator[AsyncEngine]:
    engine = create_async_engine(migrated_postgres_url, pool_pre_ping=True)
    try:
        yield engine
    finally:
        await engine.dispose()


@pytest.fixture
def session_factory(db_engine: AsyncEngine) -> async_sessionmaker[AsyncSession]:
    return create_session_factory(db_engine)


@pytest.fixture
def client(
    migrated_postgres_url: str,
    artifact_root: Path,
    db_engine: AsyncEngine,
    session_factory: async_sessionmaker[AsyncSession],
) -> Iterator[TestClient]:
    without_scheme = migrated_postgres_url.split("://", 1)[1]
    credentials, host_part = without_scheme.split("@", 1)
    user, password = credentials.split(":", 1)
    host_port, database = host_part.split("/", 1)
    host, port_text = host_port.rsplit(":", 1)

    settings = Settings(
        env=RuntimeEnv.TEST,
        log_level=LogLevel.WARNING,
        database_host=host,
        database_port=int(port_text),
        database_name=database,
        database_user=user,
        database_password=SecretStr(password),
        artifact_root=artifact_root,
        api_base_url="http://127.0.0.1:8000",
        worker_id="api-resource-test",
        run_lease_seconds=30,
        run_heartbeat_seconds=10,
        simulation_gui=False,
    )
    store = FilesystemArtifactStore(artifact_root)

    async def check_database() -> None:
        async with session_factory() as session:
            await session.execute(text("SELECT 1"))

    runtime = AppRuntime(
        settings=settings,
        engine=db_engine,
        session_factory=session_factory,
        artifact_store=store,
        check_database=check_database,
        check_artifacts=lambda: None,
    )
    app = create_app(settings, runtime=runtime)
    with TestClient(app) as test_client:
        yield test_client


def _scenario_payload(
    *, seed: int = 1, ordinal: int = 0, name: str | None = None
) -> dict[str, Any]:
    return {
        "name": name or f"set-{seed}-{ordinal}-{new_id()}",
        "generator_version": "1.0.0",
        "seed_manifest": {"seeds": [seed]},
        "scene_config": {"scene": "baseline"},
        "checksum": CHECKSUM,
        "scenarios": [
            {
                "ordinal": ordinal,
                "seed": seed,
                "object_name": "cube",
                "object_category": "parcel",
                "target_bin": "bin_a",
                "initial_pose": {
                    "position_m": [0.0, 0.0, 0.1],
                    "orientation_xyzw": [0, 0, 0, 1],
                },
                "physical_properties": {"mass_kg": 0.2, "friction": 0.5},
                "checksum": CHECKSUM,
            }
        ],
    }


async def _seed_policy(session: AsyncSession, *, name: str = "fixed") -> PolicyVersion:
    policy = PolicyVersion(
        id=new_id(),
        name=name,
        semantic_version=str(new_id()),
        description="test policy",
        config={"name": name},
        config_sha256=CHECKSUM,
        source_revision="test-revision",
        container_image_digest="sha256:" + "b" * 64,
        created_at=utc_now(),
    )
    await policy_repo.add_policy_version(session, policy)
    return policy


def test_cursor_round_trip() -> None:
    encoded = encode_cursor(kind="trials", sort_values=["0193f1a2-b3c4-7d8e-9f01-23456789abcd"])
    assert decode_cursor(encoded, expected_kind="trials") == [
        "0193f1a2-b3c4-7d8e-9f01-23456789abcd"
    ]


@pytest.mark.asyncio
async def test_policies_list_success(
    client: TestClient,
    session_factory: async_sessionmaker[AsyncSession],
) -> None:
    async with session_scope(session_factory) as session:
        policy = await _seed_policy(session)
        policy_id = str(policy.id)

    response = client.get("/api/v1/policies")
    assert response.status_code == 200
    items = response.json()["items"]
    assert any(item["id"] == policy_id for item in items)


def test_policies_dependency_unavailable(artifact_root: Path, migrated_postgres_url: str) -> None:
    without_scheme = migrated_postgres_url.split("://", 1)[1]
    credentials, host_part = without_scheme.split("@", 1)
    user, password = credentials.split(":", 1)
    host_port, database = host_part.split("/", 1)
    host, port_text = host_port.rsplit(":", 1)
    settings = Settings(
        env=RuntimeEnv.TEST,
        log_level=LogLevel.WARNING,
        database_host=host,
        database_port=int(port_text),
        database_name=database,
        database_user=user,
        database_password=SecretStr(password),
        artifact_root=artifact_root,
        api_base_url="http://127.0.0.1:8000",
        worker_id="api-resource-test",
        run_lease_seconds=30,
        run_heartbeat_seconds=10,
        simulation_gui=False,
    )

    class BrokenSessionFactory:
        def __call__(self) -> Any:
            raise ApiError(
                "DEPENDENCY_UNAVAILABLE",
                status=503,
                detail="Database is unavailable",
            )

    runtime = AppRuntime(
        settings=settings,
        engine=create_async_engine(migrated_postgres_url),  # unused; closed below
        session_factory=BrokenSessionFactory(),  # type: ignore[arg-type]
        artifact_store=FilesystemArtifactStore(artifact_root),
        check_database=lambda: (_ for _ in ()).throw(
            ApiError("DEPENDENCY_UNAVAILABLE", status=503, detail="Database is unavailable")
        ),
        check_artifacts=lambda: None,
    )
    app = create_app(settings, runtime=runtime)
    with TestClient(app) as test_client:
        response = test_client.get("/api/v1/policies")
    assert response.status_code == 503
    assert response.headers["content-type"].startswith(PROBLEM_CONTENT_TYPE)
    assert response.json()["code"] == "DEPENDENCY_UNAVAILABLE"


def test_scenario_set_create_get_and_validation(client: TestClient) -> None:
    created = client.post("/api/v1/scenario-sets", json=_scenario_payload())
    assert created.status_code == 201
    body = created.json()
    assert body["scenario_count"] == 1
    assert body["scenarios"][0]["object_name"] == "cube"

    fetched = client.get(f"/api/v1/scenario-sets/{body['id']}")
    assert fetched.status_code == 200
    assert fetched.json()["id"] == body["id"]

    missing = client.get(f"/api/v1/scenario-sets/{new_id()}")
    assert missing.status_code == 404
    assert missing.json()["code"] == "RESOURCE_NOT_FOUND"

    invalid = client.post(
        "/api/v1/scenario-sets",
        json={**_scenario_payload(), "checksum": "not-a-checksum"},
    )
    assert invalid.status_code == 422
    assert invalid.json()["code"] == "VALIDATION_ERROR"

    conflict_payload = _scenario_payload(seed=42)
    conflict_payload["scenarios"].append(
        {
            **conflict_payload["scenarios"][0],
            "ordinal": 0,
            "seed": 43,
        }
    )
    conflict = client.post("/api/v1/scenario-sets", json=conflict_payload)
    assert conflict.status_code == 422


@pytest.mark.asyncio
async def test_experiments_crud_and_not_found(
    client: TestClient,
    session_factory: async_sessionmaker[AsyncSession],
) -> None:
    async with session_scope(session_factory) as session:
        policy = await _seed_policy(session, name="pose_aware")
        policy_id = str(policy.id)

    scenario_set = client.post("/api/v1/scenario-sets", json=_scenario_payload(seed=7)).json()
    created = client.post(
        "/api/v1/experiments",
        json={
            "name": "demo-experiment",
            "description": "neutral",
            "scenario_set_id": scenario_set["id"],
            "policy_version_ids": [policy_id],
            "requested_by": "reviewer",
            "source_revision": "deadbeef",
            "simulator_image_digest": "sha256:" + "c" * 64,
        },
    )
    assert created.status_code == 201
    experiment = created.json()
    assert experiment["status"] == "draft"
    assert experiment["policies"][0]["policy_version_id"] == policy_id

    listed = client.get("/api/v1/experiments")
    assert listed.status_code == 200
    assert any(item["id"] == experiment["id"] for item in listed.json()["items"])

    fetched = client.get(f"/api/v1/experiments/{experiment['id']}")
    assert fetched.status_code == 200

    missing = client.get(f"/api/v1/experiments/{new_id()}")
    assert missing.status_code == 404

    invalid = client.post(
        "/api/v1/experiments",
        json={
            "name": "bad",
            "scenario_set_id": scenario_set["id"],
            "policy_version_ids": [str(new_id())],
            "source_revision": "deadbeef",
            "simulator_image_digest": "sha256:" + "c" * 64,
        },
    )
    assert invalid.status_code == 404


@pytest.mark.asyncio
async def test_runs_idempotency_cancel_and_conflict(
    client: TestClient,
    session_factory: async_sessionmaker[AsyncSession],
) -> None:
    async with session_scope(session_factory) as session:
        policy = await _seed_policy(session, name="collision_aware")
        policy_id = str(policy.id)

    scenario_set = client.post("/api/v1/scenario-sets", json=_scenario_payload(seed=9)).json()
    experiment = client.post(
        "/api/v1/experiments",
        json={
            "name": "run-experiment",
            "scenario_set_id": scenario_set["id"],
            "policy_version_ids": [policy_id],
            "source_revision": "deadbeef",
            "simulator_image_digest": "sha256:" + "c" * 64,
        },
    ).json()

    missing_key = client.post(f"/api/v1/experiments/{experiment['id']}/runs")
    assert missing_key.status_code == 422
    assert missing_key.json()["code"] == "VALIDATION_ERROR"

    first = client.post(
        f"/api/v1/experiments/{experiment['id']}/runs",
        headers={"Idempotency-Key": "run-key-1"},
        content=b"",
    )
    assert first.status_code == 201
    run_id = first.json()["id"]

    replay = client.post(
        f"/api/v1/experiments/{experiment['id']}/runs",
        headers={"Idempotency-Key": "run-key-1"},
        content=b"",
    )
    assert replay.status_code == 200
    assert replay.json()["id"] == run_id

    conflict = client.post(
        f"/api/v1/experiments/{experiment['id']}/runs",
        headers={"Idempotency-Key": "run-key-1"},
        content=b'{"note":"changed"}',
    )
    assert conflict.status_code == 409
    assert conflict.json()["code"] == "IDEMPOTENCY_CONFLICT"

    fetched = client.get(f"/api/v1/runs/{run_id}")
    assert fetched.status_code == 200
    assert fetched.json()["status"] == "queued"

    cancelled = client.post(f"/api/v1/runs/{run_id}/cancel")
    assert cancelled.status_code == 200
    assert cancelled.json()["status"] == "cancelled"

    again = client.post(f"/api/v1/runs/{run_id}/cancel")
    assert again.status_code == 409
    assert again.json()["code"] == "RUN_ALREADY_TERMINAL"

    missing = client.get(f"/api/v1/runs/{new_id()}")
    assert missing.status_code == 404


@pytest.mark.asyncio
async def test_trials_events_artifacts_annotations(
    client: TestClient,
    session_factory: async_sessionmaker[AsyncSession],
    artifact_root: Path,
) -> None:
    async with session_scope(session_factory) as session:
        policy = await _seed_policy(session, name="fixed")
        policy_id = str(policy.id)

    scenario_set = client.post("/api/v1/scenario-sets", json=_scenario_payload(seed=11)).json()
    experiment = client.post(
        "/api/v1/experiments",
        json={
            "name": "trial-experiment",
            "scenario_set_id": scenario_set["id"],
            "policy_version_ids": [policy_id],
            "source_revision": "deadbeef",
            "simulator_image_digest": "sha256:" + "c" * 64,
        },
    ).json()

    async with session_scope(session_factory) as session:
        trial, _created = await trial_repo.create_trial_if_absent(
            session,
            experiment_id=UUID(experiment["id"]),
            policy_version_id=UUID(policy_id),
            scenario_id=UUID(scenario_set["scenarios"][0]["id"]),
        )
        await trial_repo.mark_trial_completed(
            session,
            trial_id=trial.id,
            terminal_outcome="success",
            success=True,
            collision_count=0,
            collision_max_force_newtons=Decimal("0"),
            duration_seconds=Decimal("1.5"),
            action_count=3,
        )
        await event_repo.add_trial_event(
            session,
            TrialEvent(
                trial_id=trial.id,
                ordinal=0,
                timestamp_offset_seconds=Decimal("0"),
                event_type="state_start",
                controller_state="observe",
                action=None,
                observation={"ok": True},
            ),
        )
        await event_repo.add_trial_event(
            session,
            TrialEvent(
                trial_id=trial.id,
                ordinal=1,
                timestamp_offset_seconds=Decimal("0.1"),
                event_type="state_end",
                controller_state="observe",
                action=None,
                observation=None,
            ),
        )
        storage_key = artifact_storage_key(experiment["id"], trial.id, "initial_rgb")
        payload = b"\x89PNG\r\n\x1a\n" + b"rgb-fixture"
        store = FilesystemArtifactStore(artifact_root)
        meta = store.write(storage_key, payload)
        artifact = Artifact(
            id=new_id(),
            trial_id=trial.id,
            kind="initial_rgb",
            storage_key=storage_key,
            media_type=meta.media_type,
            width_px=640,
            height_px=480,
            byte_size=meta.byte_size,
            sha256=meta.sha256,
            created_at=utc_now(),
        )
        await artifact_repo.add_artifact(session, artifact)
        trial_id = str(trial.id)
        artifact_id = str(artifact.id)
        digest = meta.sha256

    listed = client.get(
        "/api/v1/trials",
        params={"experiment_id": experiment["id"], "limit": 1},
    )
    assert listed.status_code == 200
    page = listed.json()
    assert len(page["items"]) == 1
    assert page["items"][0]["terminal_outcome"] == "success"

    detail = client.get(f"/api/v1/trials/{trial_id}")
    assert detail.status_code == 200

    events = client.get(
        f"/api/v1/trials/{trial_id}/events",
        params={"limit": 1},
    )
    assert events.status_code == 200
    assert len(events.json()["items"]) == 1
    assert events.json()["page"]["next_cursor"] is not None
    next_page = client.get(
        f"/api/v1/trials/{trial_id}/events",
        params={"cursor": events.json()["page"]["next_cursor"]},
    )
    assert next_page.status_code == 200
    assert len(next_page.json()["items"]) == 1
    assert next_page.json()["page"]["next_cursor"] is None

    artifacts = client.get(f"/api/v1/trials/{trial_id}/artifacts")
    assert artifacts.status_code == 200
    assert artifacts.json()["items"][0]["kind"] == "initial_rgb"

    content = client.get(f"/api/v1/artifacts/{artifact_id}/content")
    assert content.status_code == 200
    assert content.headers["content-type"].startswith("image/png")
    assert content.headers["x-content-type-options"] == "nosniff"
    assert content.headers["etag"] == f'"{digest}"'
    assert content.headers["x-checksum-sha256"] == digest
    assert "attachment;" in content.headers["content-disposition"]
    assert content.headers["content-length"] == str(len(content.content))
    assert sha256_hex_bytes(content.content) == digest

    missing_artifact = client.get(f"/api/v1/artifacts/{new_id()}/content")
    assert missing_artifact.status_code == 404

    created = client.post(
        f"/api/v1/trials/{trial_id}/annotations",
        json={"label": "needs_review", "note": "check grasp", "reviewer": "alice"},
    )
    assert created.status_code == 201
    annotation = created.json()
    assert annotation["revision"] == 1

    updated = client.patch(
        f"/api/v1/annotations/{annotation['id']}",
        json={"label": "confirmed", "note": "ok", "revision": 1},
    )
    assert updated.status_code == 200
    assert updated.json()["revision"] == 2

    conflict = client.patch(
        f"/api/v1/annotations/{annotation['id']}",
        json={"label": "stale", "note": None, "revision": 1},
    )
    assert conflict.status_code == 409
    assert conflict.json()["code"] == "CONFLICT"

    missing_trial = client.get(f"/api/v1/trials/{new_id()}")
    assert missing_trial.status_code == 404

    invalid_filter = client.get("/api/v1/trials", params={"terminal_outcome": "nope"})
    assert invalid_filter.status_code == 422
