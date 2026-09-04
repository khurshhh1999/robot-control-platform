"""Milestone RGB frame capture at controller state boundaries."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Final
from uuid import UUID

from robot_control_platform_common.artifacts.base import (
    ArtifactMetadata,
    ArtifactStore,
    artifact_storage_key,
)

from robot_control_platform_simulator.domain.enums import ArtifactKind, ControllerState

MILESTONE_RGB_KINDS: Final[frozenset[ArtifactKind]] = frozenset(
    {
        ArtifactKind.INITIAL_RGB,
        ArtifactKind.PRE_GRASP_RGB,
        ArtifactKind.POST_GRASP_RGB,
        ArtifactKind.PRE_RELEASE_RGB,
        ArtifactKind.TERMINAL_RGB,
    }
)

# Capture when the controller crosses these exact state boundaries.
_TRANSITION_MILESTONES: Final[dict[tuple[ControllerState, ControllerState], ArtifactKind]] = {
    (ControllerState.RESET, ControllerState.OBSERVE): ArtifactKind.INITIAL_RGB,
    (ControllerState.APPROACH, ControllerState.GRASP): ArtifactKind.PRE_GRASP_RGB,
    (ControllerState.VERIFY_GRASP, ControllerState.LIFT): ArtifactKind.POST_GRASP_RGB,
    (ControllerState.TRANSFER, ControllerState.RELEASE): ArtifactKind.PRE_RELEASE_RGB,
}


@dataclass(frozen=True)
class MilestoneCapture:
    """PNG bytes and camera provenance for one milestone frame."""

    png_bytes: bytes
    camera_checksum: str
    width_px: int
    height_px: int
    media_type: str = "image/png"


CaptureFn = Callable[[ArtifactKind], MilestoneCapture]


def milestone_kind_for_transition(
    source: ControllerState, target: ControllerState
) -> ArtifactKind | None:
    """Return the milestone kind for a controller boundary, if any."""

    mapped = _TRANSITION_MILESTONES.get((source, target))
    if mapped is not None:
        return mapped
    if target is ControllerState.TERMINAL:
        return ArtifactKind.TERMINAL_RGB
    return None


class MilestoneFrameRecorder:
    """Write the five standard RGB frames through the artifact store."""

    def __init__(
        self,
        store: ArtifactStore,
        *,
        experiment_id: UUID,
        trial_id: UUID,
        capture: CaptureFn,
    ) -> None:
        self._store = store
        self._experiment_id = experiment_id
        self._trial_id = trial_id
        self._capture = capture
        self._written: dict[ArtifactKind, ArtifactMetadata] = {}
        self._camera_checksum: str | None = None

    @property
    def written(self) -> dict[ArtifactKind, ArtifactMetadata]:
        return dict(self._written)

    @property
    def camera_checksum(self) -> str | None:
        return self._camera_checksum

    def maybe_capture(
        self, source: ControllerState, target: ControllerState
    ) -> ArtifactMetadata | None:
        kind = milestone_kind_for_transition(source, target)
        if kind is None:
            return None
        return self.capture_kind(kind)

    def capture_kind(self, kind: ArtifactKind) -> ArtifactMetadata:
        if kind not in MILESTONE_RGB_KINDS:
            msg = f"{kind.value} is not a milestone RGB kind"
            raise ValueError(msg)
        if kind in self._written:
            msg = f"milestone kind already captured: {kind.value}"
            raise ValueError(msg)
        frame = self._capture(kind)
        if not frame.png_bytes:
            msg = "milestone PNG bytes must not be empty"
            raise ValueError(msg)
        if self._camera_checksum is None:
            self._camera_checksum = frame.camera_checksum
        elif frame.camera_checksum != self._camera_checksum:
            msg = "camera checksum changed during milestone capture"
            raise ValueError(msg)
        key = artifact_storage_key(self._experiment_id, self._trial_id, kind.value)
        metadata = self._store.write(key, frame.png_bytes)
        self._written[kind] = metadata
        return metadata

    def missing_kinds(self) -> frozenset[ArtifactKind]:
        return MILESTONE_RGB_KINDS - frozenset(self._written)
