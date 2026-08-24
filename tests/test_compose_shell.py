from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
COMPOSE_PATH = REPO_ROOT / "compose.yaml"
DOCKERFILE_PATHS = (
    REPO_ROOT / "apps" / "api" / "Dockerfile",
    REPO_ROOT / "services" / "simulator" / "Dockerfile",
    REPO_ROOT / "apps" / "web" / "Dockerfile",
)


def _compose_text() -> str:
    return COMPOSE_PATH.read_text(encoding="utf-8")


def _service_block(text: str, name: str) -> str:
    lines = text.splitlines()
    heading = f"  {name}:"
    start = next(index for index, line in enumerate(lines) if line == heading)
    collected: list[str] = []
    for line in lines[start + 1 :]:
        if line.startswith("  ") and not line.startswith("    ") and line.endswith(":"):
            break
        if line and not line.startswith(" ") and not line.startswith("#"):
            break
        collected.append(line)
    return "\n".join(collected)


def _top_keys(text: str, heading: str) -> list[str]:
    lines = text.splitlines()
    start = next(index for index, line in enumerate(lines) if line == heading)
    keys: list[str] = []
    for line in lines[start + 1 :]:
        if line and not line.startswith(" ") and not line.startswith("#"):
            break
        if line.startswith("  ") and not line.startswith("    ") and line.endswith(":"):
            keys.append(line.strip()[:-1])
    return keys


def test_compose_defines_exactly_four_named_services() -> None:
    text = _compose_text()
    assert text.splitlines()[0] == "name: robot-control-platform"
    assert _top_keys(text, "services:") == ["db", "api", "simulator", "web"]
    assert _top_keys(text, "networks:") == ["backend", "frontend"]
    assert _top_keys(text, "volumes:") == ["postgres_data", "robot_artifacts"]
    assert "internal: true" in _service_block(text, "backend") or "internal: true" in text


def test_network_attachments_match_the_four_service_shell() -> None:
    text = _compose_text()
    db = _service_block(text, "db")
    api = _service_block(text, "api")
    simulator = _service_block(text, "simulator")
    web = _service_block(text, "web")
    assert "backend" in db and "frontend" not in db
    assert "backend" in api and "frontend" in api
    assert "backend" in simulator and "frontend" not in simulator
    assert "frontend" in web and "backend" not in web


def test_only_web_publishes_a_host_port() -> None:
    text = _compose_text()
    assert '"3000:8080"' in _service_block(text, "web")
    assert "ports:" not in _service_block(text, "api")
    assert "ports:" not in _service_block(text, "simulator")
    assert "ports:" not in _service_block(text, "db")
    assert "5432:5432" not in text
    assert "8000:8000" not in text


def test_artifact_volume_is_read_only_for_api_and_read_write_for_simulator() -> None:
    text = _compose_text()
    api = _service_block(text, "api")
    simulator = _service_block(text, "simulator")
    assert "robot_artifacts:/var/lib/robot-platform/artifacts:ro" in api
    assert "robot_artifacts:/var/lib/robot-platform/artifacts" in simulator
    assert "robot_artifacts:/var/lib/robot-platform/artifacts:ro" not in simulator


def test_application_services_use_locked_down_runtime_options() -> None:
    text = _compose_text()
    for name in ("api", "simulator", "web"):
        block = _service_block(text, name)
        assert "read_only: true" in block
        assert "init: true" in block
        assert "no-new-privileges:true" in block
        assert "cap_drop:" in block
        assert "ALL" in block
        assert "tmpfs:" in block
        assert "stop_grace_period:" in block
        assert "mem_limit:" in block
        assert "cpus:" in block
    assert 'user: "10001:10001"' in _service_block(text, "api")
    assert 'user: "10001:10001"' in _service_block(text, "simulator")
    assert 'user: "101:101"' in _service_block(text, "web")


def test_healthchecks_and_start_order_are_declared() -> None:
    text = _compose_text()
    for name in ("db", "api", "simulator", "web"):
        assert "healthcheck:" in _service_block(text, name)
    assert "service_healthy" in _service_block(text, "api")
    assert "service_healthy" in _service_block(text, "simulator")
    assert "service_healthy" in _service_block(text, "web")


def test_release_dockerfiles_pin_base_image_digests() -> None:
    for dockerfile in DOCKERFILE_PATHS:
        contents = dockerfile.read_text(encoding="utf-8")
        assert "@sha256:" in contents
        assert "latest" not in contents
        assert "AGENTS.md" not in contents
        assert ".project-private" not in contents


def test_nginx_proxies_api_prefix_to_the_api_service() -> None:
    nginx = (REPO_ROOT / "apps" / "web" / "nginx.conf").read_text(encoding="utf-8")
    assert "listen 8080;" in nginx
    assert "location /api/" in nginx
    assert "proxy_pass http://api:8000/api/;" in nginx
    assert "listen 80;" not in nginx
