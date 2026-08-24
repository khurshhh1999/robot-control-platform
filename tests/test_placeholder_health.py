from __future__ import annotations

import json
import threading
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

import pytest
from robot_control_platform_api.health import HealthHandler


def _serve() -> tuple[ThreadingHTTPServer, threading.Thread, int]:
    server = ThreadingHTTPServer(("127.0.0.1", 0), HealthHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    _host, port = server.server_address[:2]
    assert isinstance(port, int)
    return server, thread, port


def _stop(server: ThreadingHTTPServer, thread: threading.Thread) -> None:
    server.shutdown()
    server.server_close()
    thread.join(timeout=2)


def test_health_paths_return_ok_json() -> None:
    server, thread, port = _serve()
    try:
        for path in ("/health/live", "/health/ready"):
            with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=2) as response:
                assert response.status == 200
                payload = json.loads(response.read().decode("utf-8"))
            assert payload == {"status": "ok"}
    finally:
        _stop(server, thread)


def test_unknown_path_returns_not_found() -> None:
    server, thread, port = _serve()
    try:
        with pytest.raises(urllib.error.HTTPError) as error:
            urllib.request.urlopen(f"http://127.0.0.1:{port}/no-such-route", timeout=2)
        assert error.value.code == 404
    finally:
        _stop(server, thread)
