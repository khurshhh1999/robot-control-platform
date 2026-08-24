"""Placeholder HTTP health endpoints for the Compose API shell."""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

LIVE_PATH = "/health/live"
READY_PATH = "/health/ready"
_OK_BODY = b'{"status":"ok"}'


class HealthHandler(BaseHTTPRequestHandler):
    """Serve liveness and readiness JSON without application routes."""

    protocol_version = "HTTP/1.1"

    def log_message(self, format: str, *args: object) -> None:
        return

    def do_GET(self) -> None:
        path = self.path.split("?", 1)[0]
        if path in {LIVE_PATH, READY_PATH}:
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(_OK_BODY)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(_OK_BODY)
            return
        self.send_error(404)


def make_server(host: str, port: int) -> ThreadingHTTPServer:
    """Return a threaded HTTP server bound to ``host`` and ``port``."""

    return ThreadingHTTPServer((host, port), HealthHandler)
