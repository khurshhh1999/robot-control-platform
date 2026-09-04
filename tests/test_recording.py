"""Tests for milestone capture, trajectory encoding, and last-write manifests."""

from __future__ import annotations

import gzip
import json
from pathlib import Path

import numpy as np
import pytest
from robot_control_platform_common.artifacts import (
    ArtifactStoreError,
    FilesystemArtifactStore,
    artifact_storage_key,
    reconcile_artifacts,
    sha256_hex_bytes,
)
from robot_control_platform_common.ids import new_id
from robot_control_platform_simulator.domain.enums import (
    ArtifactKind,
    CanonicalUnit,
    ControllerState,
)
from robot_control_platform_simulator.domain.models import (
    JointState,
    ObjectState,
    Pose,
    QuaternionXYZW,
    Vector3,
)
from robot_control_platform_simulator.physics.camera import (
    CAMERA_HEIGHT_PX,
    CAMERA_WIDTH_PX,
    encode_rgb_png,
)
from robot_control_platform_simulator.recording import (
    MilestoneCapture,
    TrialProvenance,
    TrialRecorder,
    encode_trajectory_gzip,
    milestone_kind_for_transition,
)
from robot_control_platform_simulator.recording.trajectory import TrajectorySample


def _png(seed: int = 1) -> bytes:
    rgb = np.zeros((CAMERA_HEIGHT_PX, CAMERA_WIDTH_PX, 3), dtype=np.uint8)
    rgb[10:40, 20:80] = (seed * 10, 40, 200)
    return encode_rgb_png(rgb)


def _pose() -> Pose:
    return Pose(
        position_meters=Vector3(x=0.5, y=0.0, z=0.2),
        orientation_xyzw=QuaternionXYZW(x=0.0, y=0.0, z=0.0, w=1.0),
        frame="world",
    )


def _object_state() -> ObjectState:
    return ObjectState(
        object_id="cube",
        pose=_pose(),
        mass_kilograms=0.25,
        linear_velocity_meters_per_second=Vector3(x=0.0, y=0.0, z=0.0),
        angular_velocity_radians_per_second=Vector3(x=0.0, y=0.0, z=0.0),
    )


def _sample(time_seconds: float, state: ControllerState) -> TrajectorySample:
    return TrajectorySample(
        simulation_time_seconds=time_seconds,
        controller_state=state,
        joints=(
            JointState(
                name="joint_1",
                position=0.1,
                velocity=0.0,
                position_unit=CanonicalUnit.RADIANS,
            ),
        ),
        end_effector_pose=_pose(),
        object_state=_object_state(),
        gripper_opening_radians=0.04,
        action=None,
        transition=None,
    )


def _camera_checksum() -> str:
    return "a" * 64


def test_milestone_transition_mapping() -> None:
    assert (
        milestone_kind_for_transition(ControllerState.RESET, ControllerState.OBSERVE)
        is ArtifactKind.INITIAL_RGB
    )
    assert (
        milestone_kind_for_transition(ControllerState.APPROACH, ControllerState.GRASP)
        is ArtifactKind.PRE_GRASP_RGB
    )
    assert (
        milestone_kind_for_transition(ControllerState.VERIFY_GRASP, ControllerState.LIFT)
        is ArtifactKind.POST_GRASP_RGB
    )
    assert (
        milestone_kind_for_transition(ControllerState.TRANSFER, ControllerState.RELEASE)
        is ArtifactKind.PRE_RELEASE_RGB
    )
    assert (
        milestone_kind_for_transition(ControllerState.RETRACT, ControllerState.TERMINAL)
        is ArtifactKind.TERMINAL_RGB
    )
    assert milestone_kind_for_transition(ControllerState.PLAN, ControllerState.APPROACH) is None


def test_trial_recorder_writes_frames_trajectory_and_manifest_last(tmp_path: Path) -> None:
    store = FilesystemArtifactStore(tmp_path / "artifacts")
    experiment_id = new_id()
    trial_id = new_id()
    camera = _camera_checksum()
    captures = {kind: 0 for kind in ArtifactKind if kind.value.endswith("_rgb")}

    def capture(kind: ArtifactKind) -> MilestoneCapture:
        captures[kind] += 1
        return MilestoneCapture(
            png_bytes=_png(captures[kind]),
            camera_checksum=camera,
            width_px=CAMERA_WIDTH_PX,
            height_px=CAMERA_HEIGHT_PX,
        )

    recorder = TrialRecorder(store, experiment_id=experiment_id, trial_id=trial_id, capture=capture)
    transitions = (
        (ControllerState.RESET, ControllerState.OBSERVE),
        (ControllerState.APPROACH, ControllerState.GRASP),
        (ControllerState.VERIFY_GRASP, ControllerState.LIFT),
        (ControllerState.TRANSFER, ControllerState.RELEASE),
        (ControllerState.RETRACT, ControllerState.TERMINAL),
    )
    for source, target in transitions:
        assert recorder.on_transition(source, target) is not None
    recorder.record_trajectory_sample(_sample(0.0, ControllerState.OBSERVE))
    recorder.record_trajectory_sample(_sample(1.0 / 60.0, ControllerState.APPROACH))

    provenance = TrialProvenance(
        camera_checksum=camera,
        scenario_checksum="b" * 64,
        policy_checksum="c" * 64,
        simulator_version="0.1.0",
        source_revision="deadbeef",
    )
    written = recorder.finalize(provenance)
    assert set(written) == set(ArtifactKind)

    manifest_key = artifact_storage_key(experiment_id, trial_id, ArtifactKind.TRIAL_MANIFEST.value)
    with store.open(manifest_key) as handle:
        manifest = json.loads(handle.read().decode("utf-8"))
    assert manifest["camera_checksum"] == camera
    assert manifest["scenario_checksum"] == "b" * 64
    assert manifest["policy_checksum"] == "c" * 64
    assert manifest["simulator_version"] == "0.1.0"
    assert manifest["source_revision"] == "deadbeef"
    kinds = [entry["kind"] for entry in manifest["artifacts"]]
    assert ArtifactKind.TRIAL_MANIFEST.value not in kinds
    assert set(kinds) == {
        "initial_rgb",
        "pre_grasp_rgb",
        "post_grasp_rgb",
        "pre_release_rgb",
        "terminal_rgb",
        "trajectory",
    }

    trajectory_key = artifact_storage_key(experiment_id, trial_id, ArtifactKind.TRAJECTORY.value)
    with store.open(trajectory_key) as handle:
        raw = gzip.decompress(handle.read())
    trajectory = json.loads(raw.decode("utf-8"))
    assert trajectory["control_frequency_hz"] == 60
    assert trajectory["sample_count"] == 2
    assert trajectory["samples"][0]["controller_state"] == "observe"

    report = reconcile_artifacts(store, list(written.values()))
    assert report.is_clean


def test_manifest_refuses_missing_milestones(tmp_path: Path) -> None:
    store = FilesystemArtifactStore(tmp_path / "artifacts")
    camera = _camera_checksum()

    def capture(kind: ArtifactKind) -> MilestoneCapture:
        return MilestoneCapture(
            png_bytes=_png(),
            camera_checksum=camera,
            width_px=CAMERA_WIDTH_PX,
            height_px=CAMERA_HEIGHT_PX,
        )

    recorder = TrialRecorder(store, experiment_id=new_id(), trial_id=new_id(), capture=capture)
    recorder.on_transition(ControllerState.RESET, ControllerState.OBSERVE)
    with pytest.raises(ArtifactStoreError, match="missing milestone"):
        recorder.finalize(
            TrialProvenance(
                camera_checksum=camera,
                scenario_checksum="b" * 64,
                policy_checksum="c" * 64,
                simulator_version="0.1.0",
                source_revision="rev",
            )
        )


def test_duplicate_milestone_capture_rejected(tmp_path: Path) -> None:
    store = FilesystemArtifactStore(tmp_path / "artifacts")
    camera = _camera_checksum()

    def capture(kind: ArtifactKind) -> MilestoneCapture:
        return MilestoneCapture(
            png_bytes=_png(),
            camera_checksum=camera,
            width_px=CAMERA_WIDTH_PX,
            height_px=CAMERA_HEIGHT_PX,
        )

    recorder = TrialRecorder(store, experiment_id=new_id(), trial_id=new_id(), capture=capture)
    recorder.on_transition(ControllerState.RESET, ControllerState.OBSERVE)
    with pytest.raises(ValueError, match="already captured"):
        recorder.on_transition(ControllerState.RESET, ControllerState.OBSERVE)


def test_trajectory_gzip_is_canonical_and_deterministic() -> None:
    samples = (
        _sample(0.0, ControllerState.OBSERVE),
        _sample(1.0 / 60.0, ControllerState.PLAN),
    )
    first = encode_trajectory_gzip(samples)
    second = encode_trajectory_gzip(samples)
    assert first == second
    assert sha256_hex_bytes(first) == sha256_hex_bytes(second)
    payload = json.loads(gzip.decompress(first).decode("utf-8"))
    # Compact separators and sorted keys from canonical_dumps.
    assert list(payload.keys()) == sorted(payload.keys())
