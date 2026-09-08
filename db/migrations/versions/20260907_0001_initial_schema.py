"""Initial relational schema for experiment evidence and run metadata.

Revision ID: 20260907_0001
Revises:
Create Date: 2026-09-07

"""

from __future__ import annotations

from collections.abc import Sequence

import sqlalchemy as sa
from alembic import op
from sqlalchemy.dialects import postgresql

# revision identifiers, used by Alembic.
revision: str = "20260907_0001"
down_revision: str | None = None
branch_labels: str | Sequence[str] | None = None
depends_on: str | Sequence[str] | None = None


def upgrade() -> None:
    op.create_table(
        "policy_versions",
        sa.Column("id", sa.Uuid(), nullable=False),
        sa.Column("name", sa.Text(), nullable=False),
        sa.Column("semantic_version", sa.Text(), nullable=False),
        sa.Column("description", sa.Text(), nullable=True),
        sa.Column("config", postgresql.JSONB(astext_type=sa.Text()), nullable=False),
        sa.Column("config_sha256", sa.String(length=64), nullable=False),
        sa.Column("source_revision", sa.Text(), nullable=False),
        sa.Column("container_image_digest", sa.Text(), nullable=False),
        sa.Column("created_at", sa.DateTime(timezone=True), nullable=False),
        sa.CheckConstraint(
            "char_length(config_sha256) = 64",
            name=op.f("ck_policy_versions_config_sha256_length"),
        ),
        sa.PrimaryKeyConstraint("id", name=op.f("pk_policy_versions")),
        sa.UniqueConstraint(
            "name",
            "semantic_version",
            name="uq_policy_versions_name_semver",
        ),
    )

    op.create_table(
        "scenario_sets",
        sa.Column("id", sa.Uuid(), nullable=False),
        sa.Column("name", sa.Text(), nullable=False),
        sa.Column("generator_version", sa.Text(), nullable=False),
        sa.Column("scenario_count", sa.Integer(), nullable=False),
        sa.Column("seed_manifest", postgresql.JSONB(astext_type=sa.Text()), nullable=False),
        sa.Column("scene_config", postgresql.JSONB(astext_type=sa.Text()), nullable=False),
        sa.Column("checksum", sa.String(length=64), nullable=False),
        sa.CheckConstraint(
            "scenario_count >= 0",
            name=op.f("ck_scenario_sets_scenario_count_nonnegative"),
        ),
        sa.CheckConstraint(
            "char_length(checksum) = 64",
            name=op.f("ck_scenario_sets_checksum_length"),
        ),
        sa.PrimaryKeyConstraint("id", name=op.f("pk_scenario_sets")),
    )

    op.create_table(
        "scenarios",
        sa.Column("id", sa.Uuid(), nullable=False),
        sa.Column("scenario_set_id", sa.Uuid(), nullable=False),
        sa.Column("ordinal", sa.Integer(), nullable=False),
        sa.Column("seed", sa.Integer(), nullable=False),
        sa.Column("object_name", sa.Text(), nullable=False),
        sa.Column("object_category", sa.Text(), nullable=False),
        sa.Column("target_bin", sa.Text(), nullable=False),
        sa.Column("initial_pose", postgresql.JSONB(astext_type=sa.Text()), nullable=False),
        sa.Column(
            "physical_properties",
            postgresql.JSONB(astext_type=sa.Text()),
            nullable=False,
        ),
        sa.Column("checksum", sa.String(length=64), nullable=False),
        sa.CheckConstraint(
            "ordinal >= 0",
            name=op.f("ck_scenarios_ordinal_nonnegative"),
        ),
        sa.CheckConstraint(
            "seed >= 0",
            name=op.f("ck_scenarios_seed_nonnegative"),
        ),
        sa.CheckConstraint(
            "char_length(checksum) = 64",
            name=op.f("ck_scenarios_checksum_length"),
        ),
        sa.ForeignKeyConstraint(
            ["scenario_set_id"],
            ["scenario_sets.id"],
            name="fk_scenarios_scenario_set_id",
            ondelete="CASCADE",
        ),
        sa.PrimaryKeyConstraint("id", name=op.f("pk_scenarios")),
        sa.UniqueConstraint(
            "scenario_set_id",
            "ordinal",
            name="uq_scenarios_set_ordinal",
        ),
        sa.UniqueConstraint(
            "scenario_set_id",
            "seed",
            name="uq_scenarios_set_seed",
        ),
    )
    op.create_index("ix_scenarios_scenario_set_id", "scenarios", ["scenario_set_id"], unique=False)

    op.create_table(
        "experiments",
        sa.Column("id", sa.Uuid(), nullable=False),
        sa.Column("name", sa.Text(), nullable=False),
        sa.Column("description", sa.Text(), nullable=True),
        sa.Column("scenario_set_id", sa.Uuid(), nullable=False),
        sa.Column("status", sa.Text(), nullable=False),
        sa.Column("requested_by", sa.Text(), nullable=True),
        sa.Column("created_at", sa.DateTime(timezone=True), nullable=False),
        sa.Column("started_at", sa.DateTime(timezone=True), nullable=True),
        sa.Column("completed_at", sa.DateTime(timezone=True), nullable=True),
        sa.Column("source_revision", sa.Text(), nullable=False),
        sa.Column("simulator_image_digest", sa.Text(), nullable=False),
        sa.CheckConstraint(
            "status IN ('draft', 'queued', 'running', 'completed', "
            "'completed_with_errors', 'cancelled', 'failed')",
            name=op.f("ck_experiments_status"),
        ),
        sa.ForeignKeyConstraint(
            ["scenario_set_id"],
            ["scenario_sets.id"],
            name="fk_experiments_scenario_set_id",
            ondelete="RESTRICT",
        ),
        sa.PrimaryKeyConstraint("id", name=op.f("pk_experiments")),
    )
    op.create_index(
        "ix_experiments_scenario_set_id",
        "experiments",
        ["scenario_set_id"],
        unique=False,
    )
    op.create_index("ix_experiments_status", "experiments", ["status"], unique=False)

    op.create_table(
        "experiment_policies",
        sa.Column("experiment_id", sa.Uuid(), nullable=False),
        sa.Column("policy_version_id", sa.Uuid(), nullable=False),
        sa.Column("execution_order", sa.Integer(), nullable=False),
        sa.CheckConstraint(
            "execution_order >= 0",
            name=op.f("ck_experiment_policies_execution_order_nonnegative"),
        ),
        sa.ForeignKeyConstraint(
            ["experiment_id"],
            ["experiments.id"],
            name="fk_experiment_policies_experiment_id",
            ondelete="CASCADE",
        ),
        sa.ForeignKeyConstraint(
            ["policy_version_id"],
            ["policy_versions.id"],
            name="fk_experiment_policies_policy_version_id",
            ondelete="RESTRICT",
        ),
        sa.PrimaryKeyConstraint(
            "experiment_id",
            "policy_version_id",
            name=op.f("pk_experiment_policies"),
        ),
        sa.UniqueConstraint(
            "experiment_id",
            "execution_order",
            name="uq_experiment_policies_order",
        ),
    )
    op.create_index(
        "ix_experiment_policies_policy_version_id",
        "experiment_policies",
        ["policy_version_id"],
        unique=False,
    )

    op.create_table(
        "runs",
        sa.Column("id", sa.Uuid(), nullable=False),
        sa.Column("experiment_id", sa.Uuid(), nullable=False),
        sa.Column("status", sa.Text(), nullable=False),
        sa.Column("idempotency_key", sa.Text(), nullable=False),
        sa.Column("lease_owner", sa.Text(), nullable=True),
        sa.Column("lease_expires_at", sa.DateTime(timezone=True), nullable=True),
        sa.Column("attempt", sa.Integer(), server_default=sa.text("0"), nullable=False),
        sa.Column("error_detail", sa.Text(), nullable=True),
        sa.Column("created_at", sa.DateTime(timezone=True), nullable=False),
        sa.Column("started_at", sa.DateTime(timezone=True), nullable=True),
        sa.Column("completed_at", sa.DateTime(timezone=True), nullable=True),
        sa.CheckConstraint(
            "status IN ('queued', 'claimed', 'running', 'completed', "
            "'completed_with_errors', 'cancelling', 'cancelled', "
            "'lease_expired', 'failed')",
            name=op.f("ck_runs_status"),
        ),
        sa.CheckConstraint(
            "attempt >= 0",
            name=op.f("ck_runs_attempt_nonnegative"),
        ),
        sa.ForeignKeyConstraint(
            ["experiment_id"],
            ["experiments.id"],
            name="fk_runs_experiment_id",
            ondelete="RESTRICT",
        ),
        sa.PrimaryKeyConstraint("id", name=op.f("pk_runs")),
        sa.UniqueConstraint(
            "experiment_id",
            "idempotency_key",
            name="uq_runs_experiment_idempotency_key",
        ),
    )
    op.create_index("ix_runs_experiment_id", "runs", ["experiment_id"], unique=False)
    op.create_index("ix_runs_status", "runs", ["status"], unique=False)
    op.create_index("ix_runs_lease_expires_at", "runs", ["lease_expires_at"], unique=False)

    op.create_table(
        "trials",
        sa.Column("id", sa.Uuid(), nullable=False),
        sa.Column("experiment_id", sa.Uuid(), nullable=False),
        sa.Column("policy_version_id", sa.Uuid(), nullable=False),
        sa.Column("scenario_id", sa.Uuid(), nullable=False),
        sa.Column("status", sa.Text(), nullable=False),
        sa.Column("terminal_outcome", sa.Text(), nullable=True),
        sa.Column("success", sa.Boolean(), nullable=True),
        sa.Column("collision_count", sa.Integer(), nullable=True),
        sa.Column("collision_max_force_newtons", sa.Numeric(18, 6), nullable=True),
        sa.Column("duration_seconds", sa.Numeric(18, 6), nullable=True),
        sa.Column("action_count", sa.Integer(), nullable=True),
        sa.Column("started_at", sa.DateTime(timezone=True), nullable=True),
        sa.Column("completed_at", sa.DateTime(timezone=True), nullable=True),
        sa.Column(
            "simulator_metadata",
            postgresql.JSONB(astext_type=sa.Text()),
            nullable=True,
        ),
        sa.CheckConstraint(
            "status IN ('pending', 'running', 'completed', 'failed', 'cancelled')",
            name=op.f("ck_trials_status"),
        ),
        sa.CheckConstraint(
            "terminal_outcome IS NULL OR terminal_outcome IN ("
            "'system_error', 'collision', 'missed_grasp', 'dropped_object', "
            "'wrong_bin', 'success')",
            name=op.f("ck_trials_terminal_outcome"),
        ),
        sa.CheckConstraint(
            "(status IN ('completed', 'failed') AND terminal_outcome IS NOT NULL) "
            "OR (status NOT IN ('completed', 'failed') AND terminal_outcome IS NULL)",
            name=op.f("ck_trials_terminal_outcome_lifecycle"),
        ),
        sa.CheckConstraint(
            "(status = 'failed' AND terminal_outcome = 'system_error') OR status <> 'failed'",
            name=op.f("ck_trials_failed_is_system_error"),
        ),
        sa.CheckConstraint(
            "(status = 'completed' AND success IS NOT NULL) "
            "OR (status <> 'completed' AND success IS NULL)",
            name=op.f("ck_trials_success_lifecycle"),
        ),
        sa.CheckConstraint(
            "collision_count >= 0",
            name=op.f("ck_trials_collision_count_nonnegative"),
        ),
        sa.CheckConstraint(
            "collision_max_force_newtons >= 0",
            name=op.f("ck_trials_collision_force_nonnegative"),
        ),
        sa.CheckConstraint(
            "duration_seconds >= 0",
            name=op.f("ck_trials_duration_nonnegative"),
        ),
        sa.CheckConstraint(
            "action_count >= 0",
            name=op.f("ck_trials_action_count_nonnegative"),
        ),
        sa.ForeignKeyConstraint(
            ["experiment_id"],
            ["experiments.id"],
            name="fk_trials_experiment_id",
            ondelete="RESTRICT",
        ),
        sa.ForeignKeyConstraint(
            ["policy_version_id"],
            ["policy_versions.id"],
            name="fk_trials_policy_version_id",
            ondelete="RESTRICT",
        ),
        sa.ForeignKeyConstraint(
            ["scenario_id"],
            ["scenarios.id"],
            name="fk_trials_scenario_id",
            ondelete="RESTRICT",
        ),
        sa.PrimaryKeyConstraint("id", name=op.f("pk_trials")),
        sa.UniqueConstraint(
            "experiment_id",
            "policy_version_id",
            "scenario_id",
            name="uq_trials_experiment_policy_scenario",
        ),
    )
    op.create_index("ix_trials_experiment_id", "trials", ["experiment_id"], unique=False)
    op.create_index("ix_trials_policy_version_id", "trials", ["policy_version_id"], unique=False)
    op.create_index("ix_trials_scenario_id", "trials", ["scenario_id"], unique=False)
    op.create_index("ix_trials_status", "trials", ["status"], unique=False)
    op.create_index("ix_trials_terminal_outcome", "trials", ["terminal_outcome"], unique=False)

    op.create_table(
        "trial_events",
        sa.Column("id", sa.BigInteger(), autoincrement=True, nullable=False),
        sa.Column("trial_id", sa.Uuid(), nullable=False),
        sa.Column("ordinal", sa.Integer(), nullable=False),
        sa.Column("timestamp_offset_seconds", sa.Numeric(18, 6), nullable=False),
        sa.Column("event_type", sa.Text(), nullable=False),
        sa.Column("controller_state", sa.Text(), nullable=True),
        sa.Column("action", postgresql.JSONB(astext_type=sa.Text()), nullable=True),
        sa.Column("observation", postgresql.JSONB(astext_type=sa.Text()), nullable=True),
        sa.CheckConstraint(
            "ordinal >= 0",
            name=op.f("ck_trial_events_ordinal_nonnegative"),
        ),
        sa.CheckConstraint(
            "timestamp_offset_seconds >= 0",
            name=op.f("ck_trial_events_timestamp_offset_nonnegative"),
        ),
        sa.CheckConstraint(
            "event_type IN ('state_start', 'state_end', 'state_failure', "
            "'action', 'observation', 'contact', 'timeout')",
            name=op.f("ck_trial_events_event_type"),
        ),
        sa.CheckConstraint(
            "controller_state IS NULL OR controller_state IN ("
            "'reset', 'observe', 'plan', 'approach', 'grasp', 'verify_grasp', "
            "'lift', 'transfer', 'release', 'verify_place', 'retract', 'terminal')",
            name=op.f("ck_trial_events_controller_state"),
        ),
        sa.ForeignKeyConstraint(
            ["trial_id"],
            ["trials.id"],
            name="fk_trial_events_trial_id",
            ondelete="CASCADE",
        ),
        sa.PrimaryKeyConstraint("id", name=op.f("pk_trial_events")),
        sa.UniqueConstraint(
            "trial_id",
            "ordinal",
            name="uq_trial_events_trial_ordinal",
        ),
    )
    op.create_index("ix_trial_events_trial_id", "trial_events", ["trial_id"], unique=False)

    op.create_table(
        "artifacts",
        sa.Column("id", sa.Uuid(), nullable=False),
        sa.Column("trial_id", sa.Uuid(), nullable=False),
        sa.Column("kind", sa.Text(), nullable=False),
        sa.Column("storage_key", sa.Text(), nullable=False),
        sa.Column("media_type", sa.Text(), nullable=False),
        sa.Column("width_px", sa.Integer(), nullable=True),
        sa.Column("height_px", sa.Integer(), nullable=True),
        sa.Column("byte_size", sa.BigInteger(), nullable=False),
        sa.Column("sha256", sa.String(length=64), nullable=False),
        sa.Column("created_at", sa.DateTime(timezone=True), nullable=False),
        sa.CheckConstraint(
            "kind IN ('initial_rgb', 'pre_grasp_rgb', 'post_grasp_rgb', "
            "'pre_release_rgb', 'terminal_rgb', 'trajectory', 'trial_manifest')",
            name=op.f("ck_artifacts_kind"),
        ),
        sa.CheckConstraint(
            "byte_size >= 0",
            name=op.f("ck_artifacts_byte_size_nonnegative"),
        ),
        sa.CheckConstraint(
            "char_length(sha256) = 64",
            name=op.f("ck_artifacts_sha256_length"),
        ),
        sa.CheckConstraint(
            "(width_px IS NULL AND height_px IS NULL) OR (width_px > 0 AND height_px > 0)",
            name=op.f("ck_artifacts_dimensions_pair"),
        ),
        sa.ForeignKeyConstraint(
            ["trial_id"],
            ["trials.id"],
            name="fk_artifacts_trial_id",
            ondelete="RESTRICT",
        ),
        sa.PrimaryKeyConstraint("id", name=op.f("pk_artifacts")),
        sa.UniqueConstraint("trial_id", "kind", name="uq_artifacts_trial_kind"),
    )
    op.create_index("ix_artifacts_trial_id", "artifacts", ["trial_id"], unique=False)
    op.create_index("ix_artifacts_kind", "artifacts", ["kind"], unique=False)

    op.create_table(
        "annotations",
        sa.Column("id", sa.Uuid(), nullable=False),
        sa.Column("trial_id", sa.Uuid(), nullable=False),
        sa.Column("label", sa.Text(), nullable=False),
        sa.Column("note", sa.Text(), nullable=True),
        sa.Column("reviewer", sa.Text(), nullable=False),
        sa.Column("created_at", sa.DateTime(timezone=True), nullable=False),
        sa.Column("updated_at", sa.DateTime(timezone=True), nullable=False),
        sa.Column("revision", sa.Integer(), server_default=sa.text("1"), nullable=False),
        sa.CheckConstraint(
            "revision >= 1",
            name=op.f("ck_annotations_revision_positive"),
        ),
        sa.ForeignKeyConstraint(
            ["trial_id"],
            ["trials.id"],
            name="fk_annotations_trial_id",
            ondelete="RESTRICT",
        ),
        sa.PrimaryKeyConstraint("id", name=op.f("pk_annotations")),
    )
    op.create_index("ix_annotations_trial_id", "annotations", ["trial_id"], unique=False)


def downgrade() -> None:
    op.drop_index("ix_annotations_trial_id", table_name="annotations")
    op.drop_table("annotations")
    op.drop_index("ix_artifacts_kind", table_name="artifacts")
    op.drop_index("ix_artifacts_trial_id", table_name="artifacts")
    op.drop_table("artifacts")
    op.drop_index("ix_trial_events_trial_id", table_name="trial_events")
    op.drop_table("trial_events")
    op.drop_index("ix_trials_terminal_outcome", table_name="trials")
    op.drop_index("ix_trials_status", table_name="trials")
    op.drop_index("ix_trials_scenario_id", table_name="trials")
    op.drop_index("ix_trials_policy_version_id", table_name="trials")
    op.drop_index("ix_trials_experiment_id", table_name="trials")
    op.drop_table("trials")
    op.drop_index("ix_runs_lease_expires_at", table_name="runs")
    op.drop_index("ix_runs_status", table_name="runs")
    op.drop_index("ix_runs_experiment_id", table_name="runs")
    op.drop_table("runs")
    op.drop_index("ix_experiment_policies_policy_version_id", table_name="experiment_policies")
    op.drop_table("experiment_policies")
    op.drop_index("ix_experiments_status", table_name="experiments")
    op.drop_index("ix_experiments_scenario_set_id", table_name="experiments")
    op.drop_table("experiments")
    op.drop_index("ix_scenarios_scenario_set_id", table_name="scenarios")
    op.drop_table("scenarios")
    op.drop_table("scenario_sets")
    op.drop_table("policy_versions")
