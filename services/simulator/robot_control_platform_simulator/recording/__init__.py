"""Trial recording: milestone frames, trajectory samples, and manifests."""

from robot_control_platform_simulator.recording.manifests import (
    REQUIRED_PRE_MANIFEST_KINDS,
    TrialManifestBuilder,
    TrialProvenance,
    TrialRecorder,
)
from robot_control_platform_simulator.recording.milestones import (
    MILESTONE_RGB_KINDS,
    MilestoneCapture,
    MilestoneFrameRecorder,
    milestone_kind_for_transition,
)
from robot_control_platform_simulator.recording.trajectory import (
    TRAJECTORY_SCHEMA_VERSION,
    TrajectoryRecorder,
    TrajectorySample,
    encode_trajectory_gzip,
)

__all__ = [
    "MILESTONE_RGB_KINDS",
    "REQUIRED_PRE_MANIFEST_KINDS",
    "TRAJECTORY_SCHEMA_VERSION",
    "MilestoneCapture",
    "MilestoneFrameRecorder",
    "TrajectoryRecorder",
    "TrajectorySample",
    "TrialManifestBuilder",
    "TrialProvenance",
    "TrialRecorder",
    "encode_trajectory_gzip",
    "milestone_kind_for_transition",
]
