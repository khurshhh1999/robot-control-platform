"""Control-frequency trajectory sampling and canonical gzip JSON encoding."""

from __future__ import annotations

import gzip
from dataclasses import dataclass
from typing import Final
from uuid import UUID

from robot_control_platform_common.artifacts.base import (
    ArtifactMetadata,
    ArtifactStore,
    artifact_storage_key,
)

from robot_control_platform_simulator.control.actions import PHYSICS_CONTROL_HZ
from robot_control_platform_simulator.domain.enums import ArtifactKind, ControllerState
from robot_control_platform_simulator.domain.models import (
    DOMAIN_SCHEMA_VERSION,
    Action,
    JointState,
    JSONValue,
    ObjectState,
    Pose,
    canonical_dumps,
    require_finite,
    require_nonnegative,
)

TRAJECTORY_SCHEMA_VERSION: Final[str] = DOMAIN_SCHEMA_VERSION


@dataclass(frozen=True)
class TrajectorySample:
    """One control-frequency sample of joints, poses, gripper, and optional action."""

    simulation_time_seconds: float
    controller_state: ControllerState
    joints: tuple[JointState, ...]
    end_effector_pose: Pose
    object_state: ObjectState
    gripper_opening_radians: float
    action: Action | None = None
    transition: str | None = None

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "simulation_time_seconds",
            require_nonnegative(
                "simulation_time_seconds",
                require_finite("simulation_time_seconds", self.simulation_time_seconds),
            ),
        )
        object.__setattr__(
            self,
            "gripper_opening_radians",
            require_finite("gripper_opening_radians", self.gripper_opening_radians),
        )
        if not isinstance(self.controller_state, ControllerState):
            msg = "controller_state must be a ControllerState"
            raise ValueError(msg)
        if not isinstance(self.end_effector_pose, Pose):
            msg = "end_effector_pose must be a Pose"
            raise ValueError(msg)
        if not isinstance(self.object_state, ObjectState):
            msg = "object_state must be an ObjectState"
            raise ValueError(msg)
        for joint in self.joints:
            if not isinstance(joint, JointState):
                msg = "joints must contain JointState values"
                raise ValueError(msg)
        if self.action is not None and not isinstance(self.action, Action):
            msg = "action must be an Action or None"
            raise ValueError(msg)

    def to_checksum_payload(self) -> dict[str, JSONValue]:
        return {
            "simulation_time_seconds": self.simulation_time_seconds,
            "controller_state": self.controller_state.value,
            "joints": [joint.to_checksum_payload() for joint in self.joints],
            "end_effector_pose": self.end_effector_pose.to_checksum_payload(),
            "object_state": self.object_state.to_checksum_payload(),
            "gripper_opening_radians": self.gripper_opening_radians,
            "action": None if self.action is None else self.action.to_checksum_payload(),
            "transition": self.transition,
        }


def encode_trajectory_gzip(
    samples: tuple[TrajectorySample, ...],
    *,
    control_frequency_hz: int = PHYSICS_CONTROL_HZ,
) -> bytes:
    """Encode samples as sorted-key canonical JSON compressed with gzip."""

    if control_frequency_hz != PHYSICS_CONTROL_HZ:
        msg = f"trajectory control frequency must be {PHYSICS_CONTROL_HZ} Hz"
        raise ValueError(msg)
    payload: dict[str, JSONValue] = {
        "schema_version": TRAJECTORY_SCHEMA_VERSION,
        "control_frequency_hz": control_frequency_hz,
        "sample_count": len(samples),
        "samples": [sample.to_checksum_payload() for sample in samples],
    }
    canonical = canonical_dumps(payload).encode("utf-8")
    return gzip.compress(canonical, compresslevel=9, mtime=0)


class TrajectoryRecorder:
    """Accumulate control-frequency samples and write ``trajectory.json.gz``."""

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
        self._samples: list[TrajectorySample] = []
        self._last_time_seconds: float | None = None
        self._metadata: ArtifactMetadata | None = None

    @property
    def samples(self) -> tuple[TrajectorySample, ...]:
        return tuple(self._samples)

    @property
    def metadata(self) -> ArtifactMetadata | None:
        return self._metadata

    def record(self, sample: TrajectorySample) -> None:
        if self._metadata is not None:
            msg = "trajectory already finalized"
            raise ValueError(msg)
        if (
            self._last_time_seconds is not None
            and sample.simulation_time_seconds < self._last_time_seconds
        ):
            msg = "trajectory sample times must be nondecreasing"
            raise ValueError(msg)
        self._last_time_seconds = sample.simulation_time_seconds
        self._samples.append(sample)

    def finalize(self) -> ArtifactMetadata:
        if self._metadata is not None:
            return self._metadata
        payload = encode_trajectory_gzip(tuple(self._samples))
        key = artifact_storage_key(
            self._experiment_id, self._trial_id, ArtifactKind.TRAJECTORY.value
        )
        self._metadata = self._store.write(key, payload)
        return self._metadata
