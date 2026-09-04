"""Trial manifest construction. Manifest is always written last."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final
from uuid import UUID

from robot_control_platform_common.artifacts.base import (
    ArtifactMetadata,
    ArtifactStore,
    ArtifactStoreError,
    artifact_storage_key,
)
from robot_control_platform_common.time import to_iso8601_z, utc_now

from robot_control_platform_simulator.domain.enums import ArtifactKind, ControllerState
from robot_control_platform_simulator.domain.models import (
    DOMAIN_SCHEMA_VERSION,
    JSONValue,
    canonical_dumps,
)
from robot_control_platform_simulator.recording.milestones import (
    MILESTONE_RGB_KINDS,
    CaptureFn,
    MilestoneFrameRecorder,
)
from robot_control_platform_simulator.recording.trajectory import (
    TrajectoryRecorder,
    TrajectorySample,
)

MANIFEST_SCHEMA_VERSION: Final[str] = DOMAIN_SCHEMA_VERSION
REQUIRED_PRE_MANIFEST_KINDS: Final[frozenset[ArtifactKind]] = frozenset(
    {
        *MILESTONE_RGB_KINDS,
        ArtifactKind.TRAJECTORY,
    }
)


@dataclass(frozen=True)
class TrialProvenance:
    camera_checksum: str
    scenario_checksum: str
    policy_checksum: str
    simulator_version: str
    source_revision: str


class TrialManifestBuilder:
    """Assemble and write ``trial_manifest.json`` after all other artifacts."""

    def __init__(
        self,
        store: ArtifactStore,
        *,
        experiment_id: UUID,
        trial_id: UUID,
    ) -> None:
        self._store = store
        self._experiment_id = experiment_id
        self._trial_id = trial_id
        self._artifacts: dict[ArtifactKind, ArtifactMetadata] = {}
        self._manifest: ArtifactMetadata | None = None

    @property
    def artifacts(self) -> dict[ArtifactKind, ArtifactMetadata]:
        return dict(self._artifacts)

    @property
    def manifest(self) -> ArtifactMetadata | None:
        return self._manifest

    def register(self, kind: ArtifactKind, metadata: ArtifactMetadata) -> None:
        if kind is ArtifactKind.TRIAL_MANIFEST:
            msg = "trial_manifest cannot be registered before finalize"
            raise ValueError(msg)
        if kind in self._artifacts:
            msg = f"artifact kind already registered: {kind.value}"
            raise ValueError(msg)
        if metadata.kind != kind.value:
            msg = "metadata kind does not match registered kind"
            raise ValueError(msg)
        self._artifacts[kind] = metadata

    def missing_required_kinds(self) -> frozenset[ArtifactKind]:
        return REQUIRED_PRE_MANIFEST_KINDS - frozenset(self._artifacts)

    def finalize(self, provenance: TrialProvenance) -> ArtifactMetadata:
        if self._manifest is not None:
            return self._manifest
        missing = self.missing_required_kinds()
        if missing:
            names = ", ".join(sorted(kind.value for kind in missing))
            msg = f"cannot write manifest; missing artifacts: {names}"
            raise ArtifactStoreError(msg)
        for field_name, value in (
            ("camera_checksum", provenance.camera_checksum),
            ("scenario_checksum", provenance.scenario_checksum),
            ("policy_checksum", provenance.policy_checksum),
            ("simulator_version", provenance.simulator_version),
            ("source_revision", provenance.source_revision),
        ):
            if not isinstance(value, str) or value.strip() == "" or value != value.strip():
                msg = f"{field_name} must be a non-empty trimmed string"
                raise ValueError(msg)
            if field_name.endswith("_checksum") and (
                len(value) != 64 or any(ch not in "0123456789abcdef" for ch in value)
            ):
                msg = f"{field_name} must be lowercase hexadecimal of length 64"
                raise ValueError(msg)

        artifact_entries: list[JSONValue] = []
        for kind in sorted(self._artifacts, key=lambda item: item.value):
            meta = self._artifacts[kind]
            artifact_entries.append(
                {
                    "kind": kind.value,
                    "name": meta.storage_key.rsplit("/", 1)[-1],
                    "storage_key": meta.storage_key,
                    "media_type": meta.media_type,
                    "byte_size": meta.byte_size,
                    "sha256": meta.sha256,
                }
            )
        payload: dict[str, JSONValue] = {
            "schema_version": MANIFEST_SCHEMA_VERSION,
            "experiment_id": str(self._experiment_id),
            "trial_id": str(self._trial_id),
            "created_at": to_iso8601_z(utc_now()),
            "camera_checksum": provenance.camera_checksum,
            "scenario_checksum": provenance.scenario_checksum,
            "policy_checksum": provenance.policy_checksum,
            "simulator_version": provenance.simulator_version,
            "source_revision": provenance.source_revision,
            "artifacts": artifact_entries,
        }
        body = canonical_dumps(payload).encode("utf-8")
        key = artifact_storage_key(
            self._experiment_id, self._trial_id, ArtifactKind.TRIAL_MANIFEST.value
        )
        self._manifest = self._store.write(key, body)
        return self._manifest


class TrialRecorder:
    """Coordinates milestone frames, trajectory, and last-write manifest."""

    def __init__(
        self,
        store: ArtifactStore,
        *,
        experiment_id: UUID,
        trial_id: UUID,
        capture: CaptureFn,
    ) -> None:
        self._milestones = MilestoneFrameRecorder(
            store, experiment_id=experiment_id, trial_id=trial_id, capture=capture
        )
        self._trajectory = TrajectoryRecorder(store, experiment_id=experiment_id, trial_id=trial_id)
        self._manifest_builder = TrialManifestBuilder(
            store, experiment_id=experiment_id, trial_id=trial_id
        )
        self._finalized = False

    @property
    def milestones(self) -> MilestoneFrameRecorder:
        return self._milestones

    @property
    def trajectory(self) -> TrajectoryRecorder:
        return self._trajectory

    def on_transition(
        self, source: ControllerState, target: ControllerState
    ) -> ArtifactMetadata | None:
        metadata = self._milestones.maybe_capture(source, target)
        if metadata is not None:
            self._manifest_builder.register(ArtifactKind(metadata.kind), metadata)
        return metadata

    def record_trajectory_sample(self, sample: TrajectorySample) -> None:
        self._trajectory.record(sample)

    def finalize(self, provenance: TrialProvenance) -> dict[ArtifactKind, ArtifactMetadata]:
        if self._finalized:
            msg = "trial recorder already finalized"
            raise ValueError(msg)
        missing_frames = self._milestones.missing_kinds()
        if missing_frames:
            names = ", ".join(sorted(kind.value for kind in missing_frames))
            msg = f"missing milestone frames before finalize: {names}"
            raise ArtifactStoreError(msg)
        for kind, metadata in self._milestones.written.items():
            if kind not in self._manifest_builder.artifacts:
                self._manifest_builder.register(kind, metadata)
        trajectory_meta = self._trajectory.finalize()
        self._manifest_builder.register(ArtifactKind.TRAJECTORY, trajectory_meta)
        if self._milestones.camera_checksum is None:
            msg = "camera checksum was not recorded"
            raise ArtifactStoreError(msg)
        if provenance.camera_checksum != self._milestones.camera_checksum:
            msg = "provenance camera checksum does not match captured frames"
            raise ValueError(msg)
        manifest_meta = self._manifest_builder.finalize(provenance)
        self._finalized = True
        result = dict(self._manifest_builder.artifacts)
        result[ArtifactKind.TRIAL_MANIFEST] = manifest_meta
        return result
