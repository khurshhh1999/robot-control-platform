"""Trial and trial-event service rules."""

from __future__ import annotations

from uuid import UUID

from robot_control_platform_common.db.models import TERMINAL_OUTCOMES
from robot_control_platform_common.db.repositories import events as event_repo
from robot_control_platform_common.db.repositories import trials as trial_repo
from robot_control_platform_common.db.repositories.trials import TrialListFilters
from sqlalchemy.ext.asyncio import AsyncSession

from robot_control_platform_api.errors import ApiError
from robot_control_platform_api.pagination import (
    clamp_page_size,
    decode_cursor,
    encode_cursor,
    parse_int_cursor_value,
    parse_uuid_cursor_value,
)
from robot_control_platform_api.schemas.trials import (
    TrialEventListResponse,
    TrialListResponse,
    TrialResponse,
)
from robot_control_platform_api.services.serializers import (
    page_meta,
    trial_event_response,
    trial_response,
)


def _parse_uuid(value: str, *, field_name: str) -> UUID:
    try:
        return UUID(value)
    except ValueError as exc:
        raise ApiError(
            "VALIDATION_ERROR",
            status=422,
            detail=f"{field_name} must be a valid UUID",
        ) from exc


async def get_trial(session: AsyncSession, trial_id: str) -> TrialResponse:
    identifier = _parse_uuid(trial_id, field_name="trial_id")
    trial = await trial_repo.get_trial(session, identifier)
    return trial_response(trial)


async def list_trials(
    session: AsyncSession,
    *,
    experiment_id: str | None,
    policy_version_id: str | None,
    terminal_outcome: str | None,
    object_name: str | None,
    reviewed: bool | None,
    limit: int | None,
    cursor: str | None,
) -> TrialListResponse:
    page_size = clamp_page_size(limit)
    if terminal_outcome is not None and terminal_outcome not in TERMINAL_OUTCOMES:
        raise ApiError(
            "VALIDATION_ERROR",
            status=422,
            detail="terminal_outcome filter is invalid",
        )

    after_id: UUID | None = None
    if cursor is not None:
        sort_values = decode_cursor(cursor, expected_kind="trials")
        if len(sort_values) != 1:
            raise ApiError(
                "VALIDATION_ERROR",
                status=422,
                detail="Cursor sort values are invalid",
            )
        after_id = parse_uuid_cursor_value(sort_values[0])

    filters = TrialListFilters(
        experiment_id=(
            None
            if experiment_id is None
            else _parse_uuid(experiment_id, field_name="experiment_id")
        ),
        policy_version_id=(
            None
            if policy_version_id is None
            else _parse_uuid(policy_version_id, field_name="policy_version_id")
        ),
        terminal_outcome=terminal_outcome,
        object_name=object_name,
        reviewed=reviewed,
    )
    page = await trial_repo.list_trials(
        session,
        filters=filters,
        limit=page_size,
        after_id=after_id,
    )
    next_cursor = None
    if page.next_sort_values is not None:
        next_cursor = encode_cursor(kind="trials", sort_values=list(page.next_sort_values))
    return TrialListResponse(
        items=[trial_response(item) for item in page.items],
        page=page_meta(next_cursor=next_cursor, page_size=page_size),
    )


async def list_trial_events(
    session: AsyncSession,
    trial_id: str,
    *,
    limit: int | None,
    cursor: str | None,
) -> TrialEventListResponse:
    identifier = _parse_uuid(trial_id, field_name="trial_id")
    await trial_repo.get_trial(session, identifier)
    page_size = clamp_page_size(limit)

    after_ordinal: int | None = None
    if cursor is not None:
        sort_values = decode_cursor(cursor, expected_kind="trial_events")
        if len(sort_values) != 1:
            raise ApiError(
                "VALIDATION_ERROR",
                status=422,
                detail="Cursor sort values are invalid",
            )
        after_ordinal = parse_int_cursor_value(sort_values[0])

    page = await event_repo.list_trial_events_page(
        session,
        identifier,
        limit=page_size,
        after_ordinal=after_ordinal,
    )
    next_cursor = None
    if page.next_sort_values is not None:
        next_cursor = encode_cursor(
            kind="trial_events",
            sort_values=list(page.next_sort_values),
        )
    return TrialEventListResponse(
        items=[trial_event_response(item) for item in page.items],
        page=page_meta(next_cursor=next_cursor, page_size=page_size),
    )
