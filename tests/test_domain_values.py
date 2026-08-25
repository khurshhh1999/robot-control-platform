from __future__ import annotations

import json
import math
from enum import StrEnum

import pytest
from robot_control_platform_simulator.domain import (
    DOMAIN_SCHEMA_VERSION,
    UNIT_VECTOR_TOLERANCE,
    Action,
    ArtifactKind,
    CanonicalUnit,
    ContactEvent,
    ControllerState,
    EventType,
    ExperimentStatus,
    JointState,
    ObjectState,
    Pose,
    QuaternionXYZW,
    RunStatus,
    TerminalOutcome,
    TrialStatus,
    Vector3,
    canonical_dumps,
    sha256_hex,
)


def _identity_orientation() -> QuaternionXYZW:
    return QuaternionXYZW(x=0.0, y=0.0, z=0.0, w=1.0)


def _origin_pose(*, frame: str = "world") -> Pose:
    return Pose(
        position_meters=Vector3(x=0.1, y=0.2, z=0.3),
        orientation_xyzw=_identity_orientation(),
        frame=frame,
    )


def test_lifecycle_enum_values_match_frozen_states() -> None:
    assert set(ExperimentStatus) == {
        "draft",
        "queued",
        "running",
        "completed",
        "completed_with_errors",
        "cancelled",
        "failed",
    }
    assert set(RunStatus) == {
        "queued",
        "claimed",
        "running",
        "completed",
        "completed_with_errors",
        "cancelling",
        "cancelled",
        "lease_expired",
        "failed",
    }
    assert set(TrialStatus) == {
        "pending",
        "running",
        "completed",
        "failed",
        "cancelled",
    }


def test_outcome_artifact_controller_and_event_enum_values() -> None:
    assert set(TerminalOutcome) == {
        "system_error",
        "collision",
        "missed_grasp",
        "dropped_object",
        "wrong_bin",
        "success",
    }
    assert set(ArtifactKind) == {
        "initial_rgb",
        "pre_grasp_rgb",
        "post_grasp_rgb",
        "pre_release_rgb",
        "terminal_rgb",
        "trajectory",
        "trial_manifest",
    }
    assert list(ControllerState) == [
        ControllerState.RESET,
        ControllerState.OBSERVE,
        ControllerState.PLAN,
        ControllerState.APPROACH,
        ControllerState.GRASP,
        ControllerState.VERIFY_GRASP,
        ControllerState.LIFT,
        ControllerState.TRANSFER,
        ControllerState.RELEASE,
        ControllerState.VERIFY_PLACE,
        ControllerState.RETRACT,
        ControllerState.TERMINAL,
    ]
    assert set(EventType) == {
        "state_start",
        "state_end",
        "state_failure",
        "action",
        "observation",
        "contact",
        "timeout",
    }
    assert set(CanonicalUnit) == {
        "meters",
        "radians",
        "seconds",
        "kilograms",
        "newtons",
    }


@pytest.mark.parametrize(
    ("enum_type", "invalid"),
    [
        (ExperimentStatus, "complete"),
        (RunStatus, "expired"),
        (TrialStatus, "success"),
        (TerminalOutcome, "timeout"),
        (ArtifactKind, "rgb"),
        (ControllerState, "verify"),
        (EventType, "collision"),
        (CanonicalUnit, "degrees"),
    ],
)
def test_invalid_enum_values_are_rejected(enum_type: type[StrEnum], invalid: str) -> None:
    with pytest.raises(ValueError):
        enum_type(invalid)


def test_vector3_rejects_nonfinite_and_wrong_length() -> None:
    with pytest.raises(ValueError, match="finite"):
        Vector3(x=math.nan, y=0.0, z=0.0)
    with pytest.raises(ValueError, match="finite"):
        Vector3(x=math.inf, y=0.0, z=0.0)
    with pytest.raises(ValueError, match="finite"):
        Vector3(x=True, y=0.0, z=0.0)
    with pytest.raises(ValueError, match="length 3"):
        Vector3.from_xyz([1.0, 2.0])
    with pytest.raises(ValueError, match="length 3"):
        Vector3.from_xyz([1.0, 2.0, 3.0, 4.0])


def test_quaternion_rejects_unnormalized_and_nonfinite_values() -> None:
    QuaternionXYZW(x=1.0 + 5e-7, y=0.0, z=0.0, w=0.0)
    with pytest.raises(ValueError, match="normalized"):
        QuaternionXYZW(x=1.0 + 2e-6, y=0.0, z=0.0, w=0.0)
    with pytest.raises(ValueError, match="normalized"):
        QuaternionXYZW(x=0.0, y=0.0, z=0.0, w=0.0)
    with pytest.raises(ValueError, match="finite"):
        QuaternionXYZW(x=math.nan, y=0.0, z=0.0, w=1.0)
    with pytest.raises(ValueError, match="length 4"):
        QuaternionXYZW.from_xyzw([0.0, 0.0, 1.0])


def test_pose_joint_object_action_and_contact_round_trip() -> None:
    pose = _origin_pose()
    joint = JointState(
        name="iiwa_joint_1",
        position=0.25,
        velocity=-0.1,
        position_unit=CanonicalUnit.RADIANS,
    )
    gripper = JointState(
        name="gripper_finger",
        position=0.02,
        velocity=0.0,
        position_unit=CanonicalUnit.METERS,
    )
    obj = ObjectState(
        object_id="parcel_0",
        pose=pose,
        mass_kilograms=0.4,
        linear_velocity_meters_per_second=Vector3(x=0.0, y=0.0, z=0.0),
        angular_velocity_radians_per_second=Vector3(x=0.0, y=0.0, z=0.1),
    )
    action = Action(
        name="move_end_effector",
        simulation_time_seconds=0.0,
        target_pose=pose,
        joint_targets=(joint, gripper),
    )
    contact = ContactEvent(
        body_a="gripper_left",
        body_b="parcel_0",
        link_a="finger_pad",
        link_b="body",
        position_meters=Vector3(x=0.4, y=0.0, z=0.12),
        normal=Vector3(x=0.0, y=0.0, z=1.0),
        force_newtons=1.5,
        simulation_time_seconds=1.25,
    )

    hold = Action(name="hold", simulation_time_seconds=2.5)
    for value in (pose, joint, obj, action, hold, contact):
        restored = type(value).from_checksum_payload(json.loads(value.canonical_json()))
        assert restored == value
        assert restored.canonical_json() == value.canonical_json()
        assert restored.sha256_hex() == value.sha256_hex()
        assert restored.sha256_hex() == sha256_hex(value.to_checksum_payload())
        assert restored.sha256_hex() == restored.sha256_hex().lower()
        assert len(restored.sha256_hex()) == 64
    assert json.loads(hold.canonical_json())["target_pose"] is None
    assert json.loads(hold.canonical_json())["joint_targets"] == []


def test_canonical_json_is_stable_compact_and_key_ordered() -> None:
    pose = _origin_pose()
    first = pose.canonical_json()
    second = pose.canonical_json()
    assert first == second
    assert ": " not in first
    assert ", " not in first
    parsed = json.loads(first)
    assert list(parsed.keys()) == sorted(parsed.keys())
    assert parsed["schema_version"] == DOMAIN_SCHEMA_VERSION
    assert parsed["orientation_xyzw"] == [0.0, 0.0, 0.0, 1.0]
    assert parsed["position_meters"] == [0.1, 0.2, 0.3]
    shuffled = {
        "orientation_xyzw": parsed["orientation_xyzw"],
        "schema_version": parsed["schema_version"],
        "position_meters": parsed["position_meters"],
        "frame": parsed["frame"],
    }
    assert canonical_dumps(shuffled) == first
    assert pose.sha256_hex() == sha256_hex(shuffled)


def test_negative_zero_does_not_change_checksum() -> None:
    positive = Vector3(x=0.0, y=0.0, z=0.0)
    negative = Vector3(x=-0.0, y=-0.0, z=-0.0)
    assert positive == negative
    assert positive.canonical_json() == "[0.0,0.0,0.0]"
    assert negative.canonical_json() == "[0.0,0.0,0.0]"
    assert positive.sha256_hex() == negative.sha256_hex()


def test_pose_rejects_invalid_frame_and_schema() -> None:
    with pytest.raises(ValueError, match="trimmed string"):
        Pose(
            position_meters=Vector3(x=0.0, y=0.0, z=0.0),
            orientation_xyzw=_identity_orientation(),
            frame=" world",
        )
    pose = _origin_pose()
    payload = pose.to_checksum_payload()
    payload["schema_version"] = "2"
    with pytest.raises(ValueError, match="schema version"):
        Pose.from_checksum_payload(payload)
    payload = pose.to_checksum_payload()
    payload["extra"] = "nope"
    with pytest.raises(ValueError, match="keys are invalid"):
        Pose.from_checksum_payload(payload)


def test_joint_state_rejects_disallowed_units() -> None:
    with pytest.raises(ValueError, match="radians"):
        JointState(
            name="iiwa_joint_1",
            position=0.1,
            velocity=0.0,
            position_unit=CanonicalUnit.SECONDS,
        )
    with pytest.raises(ValueError, match="canonical unit"):
        JointState(
            name="iiwa_joint_1",
            position=0.1,
            velocity=0.0,
            position_unit="degrees",  # type: ignore[arg-type]
        )


def test_object_state_rejects_nonpositive_mass() -> None:
    pose = _origin_pose()
    zeros = Vector3(x=0.0, y=0.0, z=0.0)
    with pytest.raises(ValueError, match="positive"):
        ObjectState(
            object_id="parcel_0",
            pose=pose,
            mass_kilograms=0.0,
            linear_velocity_meters_per_second=zeros,
            angular_velocity_radians_per_second=zeros,
        )
    with pytest.raises(ValueError, match="positive"):
        ObjectState(
            object_id="parcel_0",
            pose=pose,
            mass_kilograms=-1.0,
            linear_velocity_meters_per_second=zeros,
            angular_velocity_radians_per_second=zeros,
        )


def test_action_and_contact_reject_negative_time_and_invalid_normal() -> None:
    with pytest.raises(ValueError, match="nonnegative"):
        Action(name="hold", simulation_time_seconds=-0.01)
    with pytest.raises(ValueError, match="unit vector"):
        ContactEvent(
            body_a="gripper_left",
            body_b="parcel_0",
            link_a="finger_pad",
            link_b="body",
            position_meters=Vector3(x=0.0, y=0.0, z=0.0),
            normal=Vector3(x=0.0, y=0.0, z=0.0),
            force_newtons=1.0,
            simulation_time_seconds=0.0,
        )
    with pytest.raises(ValueError, match="nonnegative"):
        ContactEvent(
            body_a="gripper_left",
            body_b="parcel_0",
            link_a="finger_pad",
            link_b="body",
            position_meters=Vector3(x=0.0, y=0.0, z=0.0),
            normal=Vector3(x=0.0, y=0.0, z=1.0),
            force_newtons=-0.1,
            simulation_time_seconds=0.0,
        )


def test_unit_vector_tolerance_is_the_shared_normalization_bound() -> None:
    assert UNIT_VECTOR_TOLERANCE == 1e-6
    require_normal = Vector3(x=0.0, y=0.0, z=1.0 + 5e-7)
    ContactEvent(
        body_a="a",
        body_b="b",
        link_a="la",
        link_b="lb",
        position_meters=Vector3(x=0.0, y=0.0, z=0.0),
        normal=require_normal,
        force_newtons=0.0,
        simulation_time_seconds=0.0,
    )
