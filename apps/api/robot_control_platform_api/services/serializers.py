"""Map SQLAlchemy models to API response schemas."""

from __future__ import annotations

from decimal import Decimal

from robot_control_platform_common.db.models import (
    Annotation,
    Artifact,
    Experiment,
    ExperimentPolicy,
    PolicyVersion,
    Run,
    Scenario,
    ScenarioSet,
    Trial,
    TrialEvent,
)

from robot_control_platform_api.schemas import PageMeta, serialize_datetime, serialize_uuid
from robot_control_platform_api.schemas.annotations import AnnotationResponse
from robot_control_platform_api.schemas.artifacts import ArtifactResponse
from robot_control_platform_api.schemas.experiments import (
    ExperimentPolicyResponse,
    ExperimentResponse,
)
from robot_control_platform_api.schemas.policies import PolicyVersionResponse
from robot_control_platform_api.schemas.runs import RunResponse
from robot_control_platform_api.schemas.scenarios import ScenarioResponse, ScenarioSetResponse
from robot_control_platform_api.schemas.trials import TrialEventResponse, TrialResponse


def _decimal_str(value: Decimal | None) -> str | None:
    if value is None:
        return None
    return format(value, "f")


def policy_response(policy: PolicyVersion) -> PolicyVersionResponse:
    return PolicyVersionResponse(
        id=serialize_uuid(policy.id),
        name=policy.name,
        semantic_version=policy.semantic_version,
        description=policy.description,
        config=dict(policy.config),
        config_sha256=policy.config_sha256,
        source_revision=policy.source_revision,
        container_image_digest=policy.container_image_digest,
        created_at=serialize_datetime(policy.created_at) or "",
    )


def scenario_response(scenario: Scenario) -> ScenarioResponse:
    return ScenarioResponse(
        id=serialize_uuid(scenario.id),
        ordinal=scenario.ordinal,
        seed=scenario.seed,
        object_name=scenario.object_name,
        object_category=scenario.object_category,
        target_bin=scenario.target_bin,
        initial_pose=dict(scenario.initial_pose),
        physical_properties=dict(scenario.physical_properties),
        checksum=scenario.checksum,
    )


def scenario_set_response(
    scenario_set: ScenarioSet,
    scenarios: list[Scenario],
) -> ScenarioSetResponse:
    return ScenarioSetResponse(
        id=serialize_uuid(scenario_set.id),
        name=scenario_set.name,
        generator_version=scenario_set.generator_version,
        scenario_count=scenario_set.scenario_count,
        seed_manifest=dict(scenario_set.seed_manifest),
        scene_config=dict(scenario_set.scene_config),
        checksum=scenario_set.checksum,
        scenarios=[scenario_response(item) for item in scenarios],
    )


def experiment_response(
    experiment: Experiment,
    policies: list[ExperimentPolicy],
) -> ExperimentResponse:
    return ExperimentResponse(
        id=serialize_uuid(experiment.id),
        name=experiment.name,
        description=experiment.description,
        scenario_set_id=serialize_uuid(experiment.scenario_set_id),
        status=experiment.status,
        requested_by=experiment.requested_by,
        created_at=serialize_datetime(experiment.created_at) or "",
        started_at=serialize_datetime(experiment.started_at),
        completed_at=serialize_datetime(experiment.completed_at),
        source_revision=experiment.source_revision,
        simulator_image_digest=experiment.simulator_image_digest,
        policies=[
            ExperimentPolicyResponse(
                policy_version_id=serialize_uuid(item.policy_version_id),
                execution_order=item.execution_order,
            )
            for item in policies
        ],
    )


def run_response(run: Run) -> RunResponse:
    return RunResponse(
        id=serialize_uuid(run.id),
        experiment_id=serialize_uuid(run.experiment_id),
        status=run.status,
        idempotency_key=run.idempotency_key,
        attempt=run.attempt,
        error_detail=run.error_detail,
        created_at=serialize_datetime(run.created_at) or "",
        started_at=serialize_datetime(run.started_at),
        completed_at=serialize_datetime(run.completed_at),
    )


def trial_response(trial: Trial) -> TrialResponse:
    return TrialResponse(
        id=serialize_uuid(trial.id),
        experiment_id=serialize_uuid(trial.experiment_id),
        policy_version_id=serialize_uuid(trial.policy_version_id),
        scenario_id=serialize_uuid(trial.scenario_id),
        status=trial.status,
        terminal_outcome=trial.terminal_outcome,
        success=trial.success,
        collision_count=trial.collision_count,
        collision_max_force_newtons=_decimal_str(trial.collision_max_force_newtons),
        duration_seconds=_decimal_str(trial.duration_seconds),
        action_count=trial.action_count,
        started_at=serialize_datetime(trial.started_at),
        completed_at=serialize_datetime(trial.completed_at),
        simulator_metadata=(
            None if trial.simulator_metadata is None else dict(trial.simulator_metadata)
        ),
    )


def trial_event_response(event: TrialEvent) -> TrialEventResponse:
    return TrialEventResponse(
        id=event.id,
        trial_id=serialize_uuid(event.trial_id),
        ordinal=event.ordinal,
        timestamp_offset_seconds=_decimal_str(event.timestamp_offset_seconds) or "0",
        event_type=event.event_type,
        controller_state=event.controller_state,
        action=None if event.action is None else dict(event.action),
        observation=None if event.observation is None else dict(event.observation),
    )


def artifact_response(artifact: Artifact) -> ArtifactResponse:
    return ArtifactResponse(
        id=serialize_uuid(artifact.id),
        trial_id=serialize_uuid(artifact.trial_id),
        kind=artifact.kind,
        media_type=artifact.media_type,
        width_px=artifact.width_px,
        height_px=artifact.height_px,
        byte_size=artifact.byte_size,
        sha256=artifact.sha256,
        created_at=serialize_datetime(artifact.created_at) or "",
    )


def annotation_response(annotation: Annotation) -> AnnotationResponse:
    return AnnotationResponse(
        id=serialize_uuid(annotation.id),
        trial_id=serialize_uuid(annotation.trial_id),
        label=annotation.label,
        note=annotation.note,
        reviewer=annotation.reviewer,
        created_at=serialize_datetime(annotation.created_at) or "",
        updated_at=serialize_datetime(annotation.updated_at) or "",
        revision=annotation.revision,
    )


def page_meta(*, next_cursor: str | None, page_size: int) -> PageMeta:
    return PageMeta(next_cursor=next_cursor, page_size=page_size)
