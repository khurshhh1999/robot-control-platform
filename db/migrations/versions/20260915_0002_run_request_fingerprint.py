"""Add request fingerprint to runs for idempotent start requests.

Revision ID: 20260915_0002
Revises: 20260907_0001
Create Date: 2026-09-15
"""

from __future__ import annotations

from collections.abc import Sequence

import sqlalchemy as sa
from alembic import op

revision: str = "20260915_0002"
down_revision: str | None = "20260907_0001"
branch_labels: str | Sequence[str] | None = None
depends_on: str | Sequence[str] | None = None

SHA256_HEX_LENGTH = 64
_PLACEHOLDER_FINGERPRINT = "0" * SHA256_HEX_LENGTH


def upgrade() -> None:
    op.add_column(
        "runs",
        sa.Column(
            "request_fingerprint",
            sa.String(length=SHA256_HEX_LENGTH),
            nullable=False,
            server_default=_PLACEHOLDER_FINGERPRINT,
        ),
    )
    op.alter_column("runs", "request_fingerprint", server_default=None)
    op.create_check_constraint(
        "ck_runs_request_fingerprint_length",
        "runs",
        f"char_length(request_fingerprint) = {SHA256_HEX_LENGTH}",
    )


def downgrade() -> None:
    op.drop_constraint("ck_runs_request_fingerprint_length", "runs", type_="check")
    op.drop_column("runs", "request_fingerprint")
