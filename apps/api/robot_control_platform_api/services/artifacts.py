"""Artifact metadata and content service rules."""

from __future__ import annotations

from dataclasses import dataclass
from uuid import UUID

from robot_control_platform_common.artifacts.filesystem import FilesystemArtifactStore
from robot_control_platform_common.db.repositories import artifacts as artifact_repo
from robot_control_platform_common.db.repositories import trials as trial_repo
from sqlalchemy.ext.asyncio import AsyncSession

from robot_control_platform_api.errors import ApiError
from robot_control_platform_api.schemas.artifacts import ArtifactListResponse, ArtifactResponse
from robot_control_platform_api.services.serializers import artifact_response


def _parse_uuid(value: str, *, field_name: str) -> UUID:
    try:
        return UUID(value)
    except ValueError as exc:
        raise ApiError(
            "VALIDATION_ERROR",
            status=422,
            detail=f"{field_name} must be a valid UUID",
        ) from exc


@dataclass(frozen=True, slots=True)
class ArtifactContent:
    """Verified artifact bytes ready for an HTTP response."""

    media_type: str
    byte_size: int
    sha256: str
    filename: str
    payload: bytes


def safe_artifact_filename(kind: str) -> str:
    """Return a path-free download filename derived from artifact kind."""

    safe_kind = "".join(ch if ch.isalnum() or ch in {"-", "_"} else "_" for ch in kind)
    return f"{safe_kind}.bin"


async def list_artifacts_for_trial(
    session: AsyncSession,
    trial_id: str,
) -> ArtifactListResponse:
    identifier = _parse_uuid(trial_id, field_name="trial_id")
    await trial_repo.get_trial(session, identifier)
    rows = await artifact_repo.list_artifacts_for_trial(session, identifier)
    items: list[ArtifactResponse] = [artifact_response(row) for row in rows]
    return ArtifactListResponse(items=items)


async def get_artifact_content(
    session: AsyncSession,
    artifact_store: FilesystemArtifactStore,
    artifact_id: str,
) -> ArtifactContent:
    """Load artifact content by ID only; never accept filesystem paths."""

    identifier = _parse_uuid(artifact_id, field_name="artifact_id")
    artifact = await artifact_repo.get_artifact(session, identifier)
    metadata = artifact_store.verify(
        artifact.storage_key,
        expected_sha256=artifact.sha256,
    )
    with artifact_store.open(artifact.storage_key) as handle:
        payload = handle.read()
    if len(payload) != artifact.byte_size or len(payload) != metadata.byte_size:
        raise ApiError(
            "ARTIFACT_INTEGRITY_FAILURE",
            status=409,
            detail="Artifact integrity verification failed",
        )
    return ArtifactContent(
        media_type=artifact.media_type,
        byte_size=artifact.byte_size,
        sha256=artifact.sha256,
        filename=safe_artifact_filename(artifact.kind),
        payload=payload,
    )
