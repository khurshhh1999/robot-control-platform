"""HTTP middleware for request correlation identifiers."""

from __future__ import annotations

import re
from typing import Final

from robot_control_platform_common.ids import new_id
from robot_control_platform_common.logging import bind_log_context, clear_log_context
from starlette.datastructures import MutableHeaders
from starlette.types import ASGIApp, Message, Receive, Scope, Send

REQUEST_ID_HEADER: Final[str] = "X-Request-Id"
_REQUEST_ID_PATTERN: Final[re.Pattern[str]] = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")


def normalize_request_id(raw: str | None) -> str:
    """Return a caller-supplied request ID when valid; otherwise generate one."""

    if raw is None:
        return str(new_id())
    candidate = raw.strip()
    if _REQUEST_ID_PATTERN.fullmatch(candidate) is None:
        return str(new_id())
    return candidate


class RequestIdMiddleware:
    """Bind and return ``X-Request-Id`` for every HTTP request.

    Implemented as pure ASGI middleware so FastAPI exception handlers remain
    effective (unlike ``BaseHTTPMiddleware``).
    """

    def __init__(self, app: ASGIApp) -> None:
        self.app = app

    async def __call__(self, scope: Scope, receive: Receive, send: Send) -> None:
        if scope["type"] != "http":
            await self.app(scope, receive, send)
            return

        headers = {
            key.decode("latin-1").lower(): value.decode("latin-1")
            for key, value in scope["headers"]
        }
        request_id = normalize_request_id(headers.get(REQUEST_ID_HEADER.lower()))
        scope.setdefault("state", {})
        scope["state"]["request_id"] = request_id
        clear_log_context()
        bind_log_context(request_id=request_id)

        async def send_with_request_id(message: Message) -> None:
            if message["type"] == "http.response.start":
                mutable = MutableHeaders(scope=message)
                mutable[REQUEST_ID_HEADER] = request_id
            await send(message)

        try:
            await self.app(scope, receive, send_with_request_id)
        finally:
            clear_log_context()
