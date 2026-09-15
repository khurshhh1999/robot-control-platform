"""SQLAlchemy models for section-6 persistence tables.

Query-critical identities, lifecycle, outcomes, timestamps, and metric fields use
typed columns. JSONB is reserved for versioned configuration and detail payloads.
Large artifact bytes are never stored here.
"""

from __future__ import annotations

from datetime import datetime
from decimal import Decimal
from typing import Any
from uuid import UUID

from sqlalchemy import (
    BigInteger,
    Boolean,
    CheckConstraint,
    DateTime,
    ForeignKey,
    Index,
    Integer,
    Numeric,
    String,
    Text,
    UniqueConstraint,
    text,
)
from sqlalchemy.dialects.postgresql import JSONB
from sqlalchemy.orm import Mapped, mapped_column

from robot_control_platform_common.db.base import Base

# Lifecycle and outcome values match simulator domain string enums.
EXPERIMENT_STATUSES: tuple[str, ...] = (
    "draft",
    "queued",
    "running",
    "completed",
    "completed_with_errors",
    "cancelled",
    "failed",
)
RUN_STATUSES: tuple[str, ...] = (
    "queued",
    "claimed",
    "running",
    "completed",
    "completed_with_errors",
    "cancelling",
    "cancelled",
    "lease_expired",
    "failed",
)
TRIAL_STATUSES: tuple[str, ...] = (
    "pending",
    "running",
    "completed",
    "failed",
    "cancelled",
)
TERMINAL_OUTCOMES: tuple[str, ...] = (
    "system_error",
    "collision",
    "missed_grasp",
    "dropped_object",
    "wrong_bin",
    "success",
)
ARTIFACT_KINDS: tuple[str, ...] = (
    "initial_rgb",
    "pre_grasp_rgb",
    "post_grasp_rgb",
    "pre_release_rgb",
    "terminal_rgb",
    "trajectory",
    "trial_manifest",
)
CONTROLLER_STATES: tuple[str, ...] = (
    "reset",
    "observe",
    "plan",
    "approach",
    "grasp",
    "verify_grasp",
    "lift",
    "transfer",
    "release",
    "verify_place",
    "retract",
    "terminal",
)
EVENT_TYPES: tuple[str, ...] = (
    "state_start",
    "state_end",
    "state_failure",
    "action",
    "observation",
    "contact",
    "timeout",
)

SHA256_HEX_LENGTH = 64


def _in_clause(values: tuple[str, ...]) -> str:
    return ", ".join(f"'{value}'" for value in values)


class PolicyVersion(Base):
    """Immutable policy implementation selected from the allowlist."""

    __tablename__ = "policy_versions"
    __table_args__ = (
        CheckConstraint(
            f"char_length(config_sha256) = {SHA256_HEX_LENGTH}",
            name="config_sha256_length",
        ),
        UniqueConstraint("name", "semantic_version", name="uq_policy_versions_name_semver"),
    )

    id: Mapped[UUID] = mapped_column(primary_key=True)
    name: Mapped[str] = mapped_column(Text, nullable=False)
    semantic_version: Mapped[str] = mapped_column(Text, nullable=False)
    description: Mapped[str | None] = mapped_column(Text, nullable=True)
    config: Mapped[dict[str, Any]] = mapped_column(JSONB, nullable=False)
    config_sha256: Mapped[str] = mapped_column(String(SHA256_HEX_LENGTH), nullable=False)
    source_revision: Mapped[str] = mapped_column(Text, nullable=False)
    container_image_digest: Mapped[str] = mapped_column(Text, nullable=False)
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), nullable=False)


class ScenarioSet(Base):
    """Frozen set of scenarios reused across matched policy comparisons."""

    __tablename__ = "scenario_sets"
    __table_args__ = (
        CheckConstraint("scenario_count >= 0", name="scenario_count_nonnegative"),
        CheckConstraint(
            f"char_length(checksum) = {SHA256_HEX_LENGTH}",
            name="checksum_length",
        ),
    )

    id: Mapped[UUID] = mapped_column(primary_key=True)
    name: Mapped[str] = mapped_column(Text, nullable=False)
    generator_version: Mapped[str] = mapped_column(Text, nullable=False)
    scenario_count: Mapped[int] = mapped_column(Integer, nullable=False)
    seed_manifest: Mapped[dict[str, Any]] = mapped_column(JSONB, nullable=False)
    scene_config: Mapped[dict[str, Any]] = mapped_column(JSONB, nullable=False)
    checksum: Mapped[str] = mapped_column(String(SHA256_HEX_LENGTH), nullable=False)


class Scenario(Base):
    """One seeded scenario belonging to a scenario set."""

    __tablename__ = "scenarios"
    __table_args__ = (
        UniqueConstraint("scenario_set_id", "ordinal", name="uq_scenarios_set_ordinal"),
        UniqueConstraint("scenario_set_id", "seed", name="uq_scenarios_set_seed"),
        CheckConstraint("ordinal >= 0", name="ordinal_nonnegative"),
        CheckConstraint("seed >= 0", name="seed_nonnegative"),
        CheckConstraint(
            f"char_length(checksum) = {SHA256_HEX_LENGTH}",
            name="checksum_length",
        ),
        Index("ix_scenarios_scenario_set_id", "scenario_set_id"),
    )

    id: Mapped[UUID] = mapped_column(primary_key=True)
    scenario_set_id: Mapped[UUID] = mapped_column(
        ForeignKey("scenario_sets.id", ondelete="CASCADE", name="fk_scenarios_scenario_set_id"),
        nullable=False,
    )
    ordinal: Mapped[int] = mapped_column(Integer, nullable=False)
    seed: Mapped[int] = mapped_column(Integer, nullable=False)
    object_name: Mapped[str] = mapped_column(Text, nullable=False)
    object_category: Mapped[str] = mapped_column(Text, nullable=False)
    target_bin: Mapped[str] = mapped_column(Text, nullable=False)
    initial_pose: Mapped[dict[str, Any]] = mapped_column(JSONB, nullable=False)
    physical_properties: Mapped[dict[str, Any]] = mapped_column(JSONB, nullable=False)
    checksum: Mapped[str] = mapped_column(String(SHA256_HEX_LENGTH), nullable=False)


class Experiment(Base):
    """Evaluation of one scenario set by one or more policy versions."""

    __tablename__ = "experiments"
    __table_args__ = (
        CheckConstraint(
            f"status IN ({_in_clause(EXPERIMENT_STATUSES)})",
            name="status",
        ),
        Index("ix_experiments_scenario_set_id", "scenario_set_id"),
        Index("ix_experiments_status", "status"),
    )

    id: Mapped[UUID] = mapped_column(primary_key=True)
    name: Mapped[str] = mapped_column(Text, nullable=False)
    description: Mapped[str | None] = mapped_column(Text, nullable=True)
    scenario_set_id: Mapped[UUID] = mapped_column(
        ForeignKey(
            "scenario_sets.id",
            ondelete="RESTRICT",
            name="fk_experiments_scenario_set_id",
        ),
        nullable=False,
    )
    status: Mapped[str] = mapped_column(Text, nullable=False)
    requested_by: Mapped[str | None] = mapped_column(Text, nullable=True)
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), nullable=False)
    started_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)
    completed_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)
    source_revision: Mapped[str] = mapped_column(Text, nullable=False)
    simulator_image_digest: Mapped[str] = mapped_column(Text, nullable=False)


class ExperimentPolicy(Base):
    """Ordered policy membership for an experiment."""

    __tablename__ = "experiment_policies"
    __table_args__ = (
        # Composite primary key enforces the unique experiment/policy pair.
        UniqueConstraint(
            "experiment_id",
            "execution_order",
            name="uq_experiment_policies_order",
        ),
        CheckConstraint("execution_order >= 0", name="execution_order_nonnegative"),
        Index("ix_experiment_policies_policy_version_id", "policy_version_id"),
    )

    experiment_id: Mapped[UUID] = mapped_column(
        ForeignKey(
            "experiments.id",
            ondelete="CASCADE",
            name="fk_experiment_policies_experiment_id",
        ),
        primary_key=True,
    )
    policy_version_id: Mapped[UUID] = mapped_column(
        ForeignKey(
            "policy_versions.id",
            ondelete="RESTRICT",
            name="fk_experiment_policies_policy_version_id",
        ),
        primary_key=True,
    )
    execution_order: Mapped[int] = mapped_column(Integer, nullable=False)


class Run(Base):
    """Asynchronous experiment execution claim with lease metadata."""

    __tablename__ = "runs"
    __table_args__ = (
        CheckConstraint(
            f"status IN ({_in_clause(RUN_STATUSES)})",
            name="status",
        ),
        CheckConstraint("attempt >= 0", name="attempt_nonnegative"),
        CheckConstraint(
            f"char_length(request_fingerprint) = {SHA256_HEX_LENGTH}",
            name="request_fingerprint_length",
        ),
        UniqueConstraint(
            "experiment_id",
            "idempotency_key",
            name="uq_runs_experiment_idempotency_key",
        ),
        Index("ix_runs_experiment_id", "experiment_id"),
        Index("ix_runs_status", "status"),
        Index("ix_runs_lease_expires_at", "lease_expires_at"),
    )

    id: Mapped[UUID] = mapped_column(primary_key=True)
    experiment_id: Mapped[UUID] = mapped_column(
        ForeignKey("experiments.id", ondelete="RESTRICT", name="fk_runs_experiment_id"),
        nullable=False,
    )
    status: Mapped[str] = mapped_column(Text, nullable=False)
    idempotency_key: Mapped[str] = mapped_column(Text, nullable=False)
    request_fingerprint: Mapped[str] = mapped_column(String(SHA256_HEX_LENGTH), nullable=False)
    lease_owner: Mapped[str | None] = mapped_column(Text, nullable=True)
    lease_expires_at: Mapped[datetime | None] = mapped_column(
        DateTime(timezone=True),
        nullable=True,
    )
    attempt: Mapped[int] = mapped_column(Integer, nullable=False, server_default=text("0"))
    error_detail: Mapped[str | None] = mapped_column(Text, nullable=True)
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), nullable=False)
    started_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)
    completed_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)


class Trial(Base):
    """One scenario executed by one policy within an experiment."""

    __tablename__ = "trials"
    __table_args__ = (
        CheckConstraint(
            f"status IN ({_in_clause(TRIAL_STATUSES)})",
            name="status",
        ),
        CheckConstraint(
            f"terminal_outcome IS NULL OR terminal_outcome IN ({_in_clause(TERMINAL_OUTCOMES)})",
            name="terminal_outcome",
        ),
        CheckConstraint(
            """
            (
                status IN ('completed', 'failed')
                AND terminal_outcome IS NOT NULL
            )
            OR (
                status NOT IN ('completed', 'failed')
                AND terminal_outcome IS NULL
            )
            """.strip(),
            name="terminal_outcome_lifecycle",
        ),
        CheckConstraint(
            """
            (
                status = 'failed'
                AND terminal_outcome = 'system_error'
            )
            OR status <> 'failed'
            """.strip(),
            name="failed_is_system_error",
        ),
        CheckConstraint(
            """
            (
                status = 'completed'
                AND success IS NOT NULL
            )
            OR (
                status <> 'completed'
                AND success IS NULL
            )
            """.strip(),
            name="success_lifecycle",
        ),
        CheckConstraint("collision_count >= 0", name="collision_count_nonnegative"),
        CheckConstraint("collision_max_force_newtons >= 0", name="collision_force_nonnegative"),
        CheckConstraint("duration_seconds >= 0", name="duration_nonnegative"),
        CheckConstraint("action_count >= 0", name="action_count_nonnegative"),
        UniqueConstraint(
            "experiment_id",
            "policy_version_id",
            "scenario_id",
            name="uq_trials_experiment_policy_scenario",
        ),
        Index("ix_trials_experiment_id", "experiment_id"),
        Index("ix_trials_policy_version_id", "policy_version_id"),
        Index("ix_trials_scenario_id", "scenario_id"),
        Index("ix_trials_status", "status"),
        Index("ix_trials_terminal_outcome", "terminal_outcome"),
    )

    id: Mapped[UUID] = mapped_column(primary_key=True)
    experiment_id: Mapped[UUID] = mapped_column(
        ForeignKey("experiments.id", ondelete="RESTRICT", name="fk_trials_experiment_id"),
        nullable=False,
    )
    policy_version_id: Mapped[UUID] = mapped_column(
        ForeignKey(
            "policy_versions.id",
            ondelete="RESTRICT",
            name="fk_trials_policy_version_id",
        ),
        nullable=False,
    )
    scenario_id: Mapped[UUID] = mapped_column(
        ForeignKey("scenarios.id", ondelete="RESTRICT", name="fk_trials_scenario_id"),
        nullable=False,
    )
    status: Mapped[str] = mapped_column(Text, nullable=False)
    terminal_outcome: Mapped[str | None] = mapped_column(Text, nullable=True)
    success: Mapped[bool | None] = mapped_column(Boolean, nullable=True)
    collision_count: Mapped[int | None] = mapped_column(Integer, nullable=True)
    collision_max_force_newtons: Mapped[Decimal | None] = mapped_column(
        Numeric(18, 6),
        nullable=True,
    )
    duration_seconds: Mapped[Decimal | None] = mapped_column(Numeric(18, 6), nullable=True)
    action_count: Mapped[int | None] = mapped_column(Integer, nullable=True)
    started_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)
    completed_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)
    simulator_metadata: Mapped[dict[str, Any] | None] = mapped_column(JSONB, nullable=True)


class TrialEvent(Base):
    """Ordered controller, action, observation, or contact event for a trial."""

    __tablename__ = "trial_events"
    __table_args__ = (
        UniqueConstraint("trial_id", "ordinal", name="uq_trial_events_trial_ordinal"),
        CheckConstraint("ordinal >= 0", name="ordinal_nonnegative"),
        CheckConstraint("timestamp_offset_seconds >= 0", name="timestamp_offset_nonnegative"),
        CheckConstraint(
            f"event_type IN ({_in_clause(EVENT_TYPES)})",
            name="event_type",
        ),
        CheckConstraint(
            f"controller_state IS NULL OR controller_state IN ({_in_clause(CONTROLLER_STATES)})",
            name="controller_state",
        ),
        Index("ix_trial_events_trial_id", "trial_id"),
    )

    id: Mapped[int] = mapped_column(BigInteger, primary_key=True, autoincrement=True)
    trial_id: Mapped[UUID] = mapped_column(
        ForeignKey("trials.id", ondelete="CASCADE", name="fk_trial_events_trial_id"),
        nullable=False,
    )
    ordinal: Mapped[int] = mapped_column(Integer, nullable=False)
    timestamp_offset_seconds: Mapped[Decimal] = mapped_column(Numeric(18, 6), nullable=False)
    event_type: Mapped[str] = mapped_column(Text, nullable=False)
    controller_state: Mapped[str | None] = mapped_column(Text, nullable=True)
    action: Mapped[dict[str, Any] | None] = mapped_column(JSONB, nullable=True)
    observation: Mapped[dict[str, Any] | None] = mapped_column(JSONB, nullable=True)


class Artifact(Base):
    """Checksummed artifact metadata; bytes live in the artifact store."""

    __tablename__ = "artifacts"
    __table_args__ = (
        UniqueConstraint("trial_id", "kind", name="uq_artifacts_trial_kind"),
        CheckConstraint(
            f"kind IN ({_in_clause(ARTIFACT_KINDS)})",
            name="kind",
        ),
        CheckConstraint("byte_size >= 0", name="byte_size_nonnegative"),
        CheckConstraint(
            f"char_length(sha256) = {SHA256_HEX_LENGTH}",
            name="sha256_length",
        ),
        CheckConstraint(
            "(width_px IS NULL AND height_px IS NULL) OR (width_px > 0 AND height_px > 0)",
            name="dimensions_pair",
        ),
        Index("ix_artifacts_trial_id", "trial_id"),
        Index("ix_artifacts_kind", "kind"),
    )

    id: Mapped[UUID] = mapped_column(primary_key=True)
    trial_id: Mapped[UUID] = mapped_column(
        ForeignKey("trials.id", ondelete="RESTRICT", name="fk_artifacts_trial_id"),
        nullable=False,
    )
    kind: Mapped[str] = mapped_column(Text, nullable=False)
    storage_key: Mapped[str] = mapped_column(Text, nullable=False)
    media_type: Mapped[str] = mapped_column(Text, nullable=False)
    width_px: Mapped[int | None] = mapped_column(Integer, nullable=True)
    height_px: Mapped[int | None] = mapped_column(Integer, nullable=True)
    byte_size: Mapped[int] = mapped_column(BigInteger, nullable=False)
    sha256: Mapped[str] = mapped_column(String(SHA256_HEX_LENGTH), nullable=False)
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), nullable=False)


class Annotation(Base):
    """Reviewer label and note attached to a trial."""

    __tablename__ = "annotations"
    __table_args__ = (
        CheckConstraint("revision >= 1", name="revision_positive"),
        Index("ix_annotations_trial_id", "trial_id"),
    )

    id: Mapped[UUID] = mapped_column(primary_key=True)
    trial_id: Mapped[UUID] = mapped_column(
        ForeignKey("trials.id", ondelete="RESTRICT", name="fk_annotations_trial_id"),
        nullable=False,
    )
    label: Mapped[str] = mapped_column(Text, nullable=False)
    note: Mapped[str | None] = mapped_column(Text, nullable=True)
    reviewer: Mapped[str] = mapped_column(Text, nullable=False)
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), nullable=False)
    updated_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), nullable=False)
    revision: Mapped[int] = mapped_column(Integer, nullable=False, server_default=text("1"))
