"""Experiment service rules."""

from __future__ import annotations

from uuid import UUID

from robot_control_platform_common.db.models import Experiment, ExperimentPolicy
from robot_control_platform_common.db.repositories import experiments as experiment_repo
from robot_control_platform_common.db.repositories import policies as policy_repo
from robot_control_platform_common.db.repositories import scenarios as scenario_repo
from robot_control_platform_common.ids import new_id
from robot_control_platform_common.time import utc_now
from sqlalchemy.ext.asyncio import AsyncSession

from robot_control_platform_api.errors import ApiError
from robot_control_platform_api.schemas.experiments import (
    ExperimentCreate,
    ExperimentListResponse,
    ExperimentResponse,
)
from robot_control_platform_api.services.serializers import experiment_response


def _parse_uuid(value: str, *, field_name: str) -> UUID:
    try:
        return UUID(value)
    except ValueError as exc:
        raise ApiError(
            "VALIDATION_ERROR",
            status=422,
            detail=f"{field_name} must be a valid UUID",
        ) from exc


async def create_experiment(
    session: AsyncSession,
    payload: ExperimentCreate,
) -> ExperimentResponse:
    scenario_set_id = _parse_uuid(payload.scenario_set_id, field_name="scenario_set_id")
    await scenario_repo.get_scenario_set(session, scenario_set_id)

    policy_ids = [
        _parse_uuid(item, field_name="policy_version_ids") for item in payload.policy_version_ids
    ]
    for policy_id in policy_ids:
        await policy_repo.get_policy_version(session, policy_id)

    experiment = Experiment(
        id=new_id(),
        name=payload.name,
        description=payload.description,
        scenario_set_id=scenario_set_id,
        status="draft",
        requested_by=payload.requested_by,
        created_at=utc_now(),
        started_at=None,
        completed_at=None,
        source_revision=payload.source_revision,
        simulator_image_digest=payload.simulator_image_digest,
    )
    await experiment_repo.add_experiment(session, experiment)

    memberships: list[ExperimentPolicy] = []
    for order, policy_id in enumerate(policy_ids):
        membership = ExperimentPolicy(
            experiment_id=experiment.id,
            policy_version_id=policy_id,
            execution_order=order,
        )
        await experiment_repo.add_experiment_policy(session, membership)
        memberships.append(membership)

    return experiment_response(experiment, memberships)


async def list_experiments(session: AsyncSession) -> ExperimentListResponse:
    experiments = await experiment_repo.list_experiments(session)
    items: list[ExperimentResponse] = []
    for experiment in experiments:
        policies = await experiment_repo.list_policies_for_experiment(session, experiment.id)
        items.append(experiment_response(experiment, policies))
    return ExperimentListResponse(items=items)


async def get_experiment(session: AsyncSession, experiment_id: str) -> ExperimentResponse:
    identifier = _parse_uuid(experiment_id, field_name="experiment_id")
    experiment = await experiment_repo.get_experiment(session, identifier)
    policies = await experiment_repo.list_policies_for_experiment(session, identifier)
    return experiment_response(experiment, policies)
