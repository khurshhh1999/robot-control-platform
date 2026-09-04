"""Artifact reconciliation: missing, corrupt, and orphaned durable bytes."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from enum import StrEnum

from robot_control_platform_common.artifacts.base import (
    ArtifactIntegrityError,
    ArtifactKeyError,
    ArtifactMetadata,
    ArtifactNotFoundError,
    parse_artifact_storage_key,
)
from robot_control_platform_common.artifacts.filesystem import FilesystemArtifactStore


class ReconciliationIssueKind(StrEnum):
    MISSING = "missing"
    CORRUPT = "corrupt"
    ORPHANED = "orphaned"
    DUPLICATE_KIND = "duplicate_kind"


@dataclass(frozen=True)
class ReconciliationIssue:
    kind: ReconciliationIssueKind
    storage_key: str
    detail: str


@dataclass(frozen=True)
class ReconciliationReport:
    healthy: tuple[ArtifactMetadata, ...]
    issues: tuple[ReconciliationIssue, ...]

    @property
    def missing_count(self) -> int:
        return sum(1 for issue in self.issues if issue.kind is ReconciliationIssueKind.MISSING)

    @property
    def corrupt_count(self) -> int:
        return sum(1 for issue in self.issues if issue.kind is ReconciliationIssueKind.CORRUPT)

    @property
    def orphaned_count(self) -> int:
        return sum(1 for issue in self.issues if issue.kind is ReconciliationIssueKind.ORPHANED)

    @property
    def is_clean(self) -> bool:
        return len(self.issues) == 0


@dataclass(frozen=True)
class QuarantineResult:
    storage_key: str
    quarantine_key: str
    reason: str


def reconcile_artifacts(
    store: FilesystemArtifactStore,
    expected: Sequence[ArtifactMetadata],
) -> ReconciliationReport:
    """Compare expected metadata to durable bytes and detect orphans.

    Incomplete temporary siblings are ignored by the store listing and therefore
    never appear as healthy artifacts.
    """

    issues: list[ReconciliationIssue] = []
    healthy: list[ArtifactMetadata] = []
    expected_keys: set[str] = set()
    seen_kinds: dict[tuple[str, str, str], str] = {}

    for record in expected:
        parse_artifact_storage_key(record.storage_key)
        if record.storage_key in expected_keys:
            issues.append(
                ReconciliationIssue(
                    kind=ReconciliationIssueKind.DUPLICATE_KIND,
                    storage_key=record.storage_key,
                    detail="expected metadata lists the same storage key more than once",
                )
            )
            continue
        expected_keys.add(record.storage_key)
        experiment_id, trial_id, kind = parse_artifact_storage_key(record.storage_key)
        kind_key = (str(experiment_id), str(trial_id), kind)
        prior = seen_kinds.get(kind_key)
        if prior is not None:
            issues.append(
                ReconciliationIssue(
                    kind=ReconciliationIssueKind.DUPLICATE_KIND,
                    storage_key=record.storage_key,
                    detail=f"duplicate standard kind {kind} for trial (also {prior})",
                )
            )
            continue
        seen_kinds[kind_key] = record.storage_key

        try:
            verified = store.verify(record.storage_key, expected_sha256=record.sha256)
        except ArtifactNotFoundError:
            issues.append(
                ReconciliationIssue(
                    kind=ReconciliationIssueKind.MISSING,
                    storage_key=record.storage_key,
                    detail="expected artifact bytes are missing",
                )
            )
            continue
        except (ArtifactIntegrityError, ArtifactKeyError) as exc:
            issues.append(
                ReconciliationIssue(
                    kind=ReconciliationIssueKind.CORRUPT,
                    storage_key=record.storage_key,
                    detail=str(exc),
                )
            )
            continue
        if verified.byte_size != record.byte_size:
            issues.append(
                ReconciliationIssue(
                    kind=ReconciliationIssueKind.CORRUPT,
                    storage_key=record.storage_key,
                    detail="artifact byte size does not match metadata",
                )
            )
            continue
        healthy.append(verified)

    for storage_key in store.list_storage_keys():
        if storage_key in expected_keys:
            continue
        issues.append(
            ReconciliationIssue(
                kind=ReconciliationIssueKind.ORPHANED,
                storage_key=storage_key,
                detail="durable artifact has no matching metadata record",
            )
        )

    return ReconciliationReport(healthy=tuple(healthy), issues=tuple(issues))
