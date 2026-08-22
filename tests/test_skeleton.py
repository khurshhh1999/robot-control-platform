from pathlib import Path

import robot_control_platform_api
import robot_control_platform_common
import robot_control_platform_simulator

REPO_ROOT = Path(__file__).resolve().parents[1]

REQUIRED_ENV_KEYS = (
    "RCP_ENV",
    "RCP_LOG_LEVEL",
    "RCP_DATABASE_HOST",
    "RCP_DATABASE_PORT",
    "RCP_DATABASE_NAME",
    "RCP_DATABASE_USER",
    "RCP_DATABASE_PASSWORD",
    "RCP_ARTIFACT_ROOT",
    "RCP_API_BASE_URL",
    "RCP_WORKER_ID",
    "RCP_RUN_LEASE_SECONDS",
    "RCP_RUN_HEARTBEAT_SECONDS",
    "RCP_SIMULATION_GUI",
)

DOCKERIGNORE_PATHS = (
    REPO_ROOT / ".dockerignore",
    REPO_ROOT / "apps" / "api" / ".dockerignore",
    REPO_ROOT / "apps" / "web" / ".dockerignore",
    REPO_ROOT / "services" / "simulator" / ".dockerignore",
)

REQUIRED_DOCKERIGNORE_PATTERNS = (
    ".git",
    ".env",
    ".env.*",
    "AGENTS.md",
    ".project-private",
    "artifacts",
    "datasets",
    "volumes",
    "node_modules",
)


def test_workspace_packages_are_importable() -> None:
    assert robot_control_platform_common.__doc__
    assert robot_control_platform_api.__doc__
    assert robot_control_platform_simulator.__doc__


def test_env_example_declares_required_settings() -> None:
    env_example = (REPO_ROOT / ".env.example").read_text(encoding="utf-8")
    for key in REQUIRED_ENV_KEYS:
        assert f"{key}=" in env_example


def test_dockerignore_files_exclude_private_and_generated_paths() -> None:
    for dockerignore_path in DOCKERIGNORE_PATHS:
        contents = dockerignore_path.read_text(encoding="utf-8")
        for pattern in REQUIRED_DOCKERIGNORE_PATTERNS:
            assert pattern in contents, f"{pattern} missing from {dockerignore_path}"


def test_toolchain_version_pins() -> None:
    python_version = (REPO_ROOT / ".python-version").read_text(encoding="utf-8").strip()
    node_version = (REPO_ROOT / ".nvmrc").read_text(encoding="utf-8").strip()
    assert python_version == "3.12"
    assert node_version == "22.23.2"
