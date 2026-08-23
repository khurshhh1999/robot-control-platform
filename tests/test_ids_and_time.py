from datetime import UTC, datetime, timedelta, timezone
from uuid import RFC_4122, UUID

import pytest
from robot_control_platform_common.ids import new_id
from robot_control_platform_common.time import to_iso8601_z, utc_now


def test_new_id_values_are_unique_uuidv7() -> None:
    generated = [new_id() for _ in range(256)]
    assert len(generated) == len(set(generated))
    for identifier in generated:
        assert isinstance(identifier, UUID)
        assert identifier.version == 7
        assert identifier.variant == RFC_4122


def test_new_id_values_are_time_sortable() -> None:
    generated = [new_id() for _ in range(256)]
    assert generated == sorted(generated)


def test_utc_now_is_timezone_aware_utc() -> None:
    now = utc_now()
    assert now.tzinfo is not None
    assert now.utcoffset() == timedelta(0)
    assert now.tzinfo is UTC or now.tzinfo == UTC


def test_to_iso8601_z_uses_zulu_suffix() -> None:
    value = datetime(2026, 8, 21, 15, 30, 45, 123456, tzinfo=UTC)
    serialized = to_iso8601_z(value)
    assert serialized == "2026-08-21T15:30:45.123456Z"
    assert serialized.endswith("Z")
    assert "+" not in serialized


def test_to_iso8601_z_converts_offsets_to_utc() -> None:
    eastern = timezone(timedelta(hours=-4))
    value = datetime(2026, 8, 21, 11, 30, 45, 123456, tzinfo=eastern)
    assert to_iso8601_z(value) == "2026-08-21T15:30:45.123456Z"


def test_to_iso8601_z_rejects_naive_datetime() -> None:
    with pytest.raises(ValueError, match="timezone-aware"):
        to_iso8601_z(datetime(2026, 8, 21, 15, 30, 45))
