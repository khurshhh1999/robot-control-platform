"""Scenario set service rules."""

from __future__ import annotations

from uuid import UUID

from robot_control_platform_common.db.models import Scenario, ScenarioSet
from robot_control_platform_common.db.repositories import scenarios as scenario_repo
from robot_control_platform_common.ids import new_id
from sqlalchemy.exc import IntegrityError
from sqlalchemy.ext.asyncio import AsyncSession

from robot_control_platform_api.errors import ApiError
from robot_control_platform_api.schemas.scenarios import ScenarioSetCreate, ScenarioSetResponse
from robot_control_platform_api.services.serializers import scenario_set_response


def _parse_uuid(value: str, *, field_name: str) -> UUID:
    try:
        return UUID(value)
    except ValueError as exc:
        raise ApiError(
            "VALIDATION_ERROR",
            status=422,
            detail=f"{field_name} must be a valid UUID",
        ) from exc


async def create_scenario_set(
    session: AsyncSession,
    payload: ScenarioSetCreate,
) -> ScenarioSetResponse:
    ordinals = [item.ordinal for item in payload.scenarios]
    seeds = [item.seed for item in payload.scenarios]
    if len(ordinals) != len(set(ordinals)):
        raise ApiError(
            "VALIDATION_ERROR",
            status=422,
            detail="Scenario ordinals must be unique within a set",
        )
    if len(seeds) != len(set(seeds)):
        raise ApiError(
            "VALIDATION_ERROR",
            status=422,
            detail="Scenario seeds must be unique within a set",
        )

    scenario_set = ScenarioSet(
        id=new_id(),
        name=payload.name,
        generator_version=payload.generator_version,
        scenario_count=len(payload.scenarios),
        seed_manifest=dict(payload.seed_manifest),
        scene_config=dict(payload.scene_config),
        checksum=payload.checksum,
    )
    try:
        async with session.begin_nested():
            await scenario_repo.add_scenario_set(session, scenario_set)
            created_scenarios: list[Scenario] = []
            for item in sorted(payload.scenarios, key=lambda row: row.ordinal):
                scenario = Scenario(
                    id=new_id(),
                    scenario_set_id=scenario_set.id,
                    ordinal=item.ordinal,
                    seed=item.seed,
                    object_name=item.object_name,
                    object_category=item.object_category,
                    target_bin=item.target_bin,
                    initial_pose=dict(item.initial_pose),
                    physical_properties=dict(item.physical_properties),
                    checksum=item.checksum,
                )
                await scenario_repo.add_scenario(session, scenario)
                created_scenarios.append(scenario)
    except IntegrityError as exc:
        raise ApiError(
            "CONFLICT",
            status=409,
            detail="Scenario set conflicts with an existing record",
        ) from exc

    return scenario_set_response(scenario_set, created_scenarios)


async def get_scenario_set(session: AsyncSession, scenario_set_id: str) -> ScenarioSetResponse:
    identifier = _parse_uuid(scenario_set_id, field_name="scenario_set_id")
    scenario_set = await scenario_repo.get_scenario_set(session, identifier)
    scenarios = await scenario_repo.list_scenarios_for_set(session, identifier)
    return scenario_set_response(scenario_set, scenarios)
