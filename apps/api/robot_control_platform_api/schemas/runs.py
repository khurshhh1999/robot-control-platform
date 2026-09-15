"""Run API schemas."""

from __future__ import annotations

from pydantic import Field

from robot_control_platform_api.schemas import ApiModel


class RunCreate(ApiModel):
    """Optional body for starting an experiment run.

    The request fingerprint includes this body. Callers must send a stable body
    when reusing an ``Idempotency-Key``.
    """

    note: str | None = Field(
        default=None,
        description="Optional caller note included in the idempotency fingerprint.",
        examples=[None],
    )


class RunResponse(ApiModel):
    """Asynchronous experiment run with lease metadata."""

    id: str = Field(examples=["0193f1a2-b3c4-7d8e-9f01-23456789abcd"])
    experiment_id: str
    status: str = Field(examples=["queued"])
    idempotency_key: str = Field(examples=["start-run-1"])
    attempt: int = Field(examples=[0])
    error_detail: str | None = None
    created_at: str
    started_at: str | None = None
    completed_at: str | None = None
