"""Time-sortable identifiers for platform entities."""

from __future__ import annotations

import secrets
import threading
import time
from uuid import RFC_4122, UUID

_RFC9562_VERSION = 7
_UNIX_TS_MS_BITS = 48
_RAND_A_BITS = 12
_RAND_B_BITS = 62
_MAX_UNIX_TS_MS = (1 << _UNIX_TS_MS_BITS) - 1
_MAX_RAND_A = (1 << _RAND_A_BITS) - 1
_MAX_RAND_B = (1 << _RAND_B_BITS) - 1

_lock = threading.Lock()
_last_unix_ts_ms = -1
_rand_a_counter = 0


def new_id() -> UUID:
    """Return a new RFC 9562 UUIDv7.

    This is the only UUIDv7 entry point. Callers must not construct entity
    identifiers through other UUID versions or helpers.
    """

    global _last_unix_ts_ms, _rand_a_counter

    with _lock:
        unix_ts_ms = min(time.time_ns() // 1_000_000, _MAX_UNIX_TS_MS)
        if unix_ts_ms > _last_unix_ts_ms:
            _last_unix_ts_ms = unix_ts_ms
            _rand_a_counter = 0
        else:
            unix_ts_ms = _last_unix_ts_ms
            _rand_a_counter += 1
            if _rand_a_counter > _MAX_RAND_A:
                unix_ts_ms = min(unix_ts_ms + 1, _MAX_UNIX_TS_MS)
                _last_unix_ts_ms = unix_ts_ms
                _rand_a_counter = 0

        rand_a = _rand_a_counter
        rand_b = secrets.randbits(_RAND_B_BITS) & _MAX_RAND_B

    uuid_int = unix_ts_ms << 80
    uuid_int |= _RFC9562_VERSION << 76
    uuid_int |= rand_a << 64
    uuid_int |= 0b10 << 62
    uuid_int |= rand_b
    identifier = UUID(int=uuid_int)
    if identifier.version != _RFC9562_VERSION or identifier.variant != RFC_4122:
        msg = "new_id() produced a UUID that is not RFC 9562 version 7"
        raise RuntimeError(msg)
    return identifier
