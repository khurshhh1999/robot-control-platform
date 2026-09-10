"""Scenario set and scenario persistence queries."""

from __future__ import annotations

from uuid import UUID

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from robot_control_platform_common.db.models import Scenario, ScenarioSet
from robot_control_platform_common.db.repositories.exceptions import EntityNotFoundError


async def add_scenario_set(session: AsyncSession, scenario_set: ScenarioSet) -> ScenarioSet:
    """Persist a new scenario set."""

    session.add(scenario_set)
    await session.flush()
    return scenario_set


async def get_scenario_set(session: AsyncSession, scenario_set_id: UUID) -> ScenarioSet:
    """Return a scenario set by id or raise ``EntityNotFoundError``."""

    scenario_set = await session.get(ScenarioSet, scenario_set_id)
    if scenario_set is None:
        msg = f"scenario set {scenario_set_id} not found"
        raise EntityNotFoundError(msg)
    return scenario_set


async def add_scenario(session: AsyncSession, scenario: Scenario) -> Scenario:
    """Persist a new scenario row."""

    session.add(scenario)
    await session.flush()
    return scenario


async def get_scenario(session: AsyncSession, scenario_id: UUID) -> Scenario:
    """Return a scenario by id or raise ``EntityNotFoundError``."""

    scenario = await session.get(Scenario, scenario_id)
    if scenario is None:
        msg = f"scenario {scenario_id} not found"
        raise EntityNotFoundError(msg)
    return scenario


async def list_scenarios_for_set(
    session: AsyncSession,
    scenario_set_id: UUID,
) -> list[Scenario]:
    """Return scenarios for a set ordered by ordinal."""

    result = await session.execute(
        select(Scenario)
        .where(Scenario.scenario_set_id == scenario_set_id)
        .order_by(Scenario.ordinal)
    )
    return list(result.scalars().all())
