"""Discover and reset every controlled joint on the loaded workcell robot."""

from __future__ import annotations

from dataclasses import dataclass

from robot_control_platform_simulator.domain.enums import CanonicalUnit
from robot_control_platform_simulator.domain.models import JointState
from robot_control_platform_simulator.physics.client import (
    FALLBACK_JOINT_FORCE_NEWTONS,
    JointRecord,
    PhysicsClient,
    SimulationError,
)


@dataclass(frozen=True)
class JointSpec:
    """Controlled joint identity and rest command used during scene reset."""

    index: int
    name: str
    rest_position: float
    force_newtons: float
    position_unit: CanonicalUnit


def discover_controlled_joints(client: PhysicsClient, body_id: int) -> tuple[JointSpec, ...]:
    """Return non-fixed joints. Body ids are ephemeral and must not be stored."""

    specs: list[JointSpec] = []
    for record in client.joint_records(body_id):
        if client.is_fixed_joint(record.joint_type):
            continue
        specs.append(_spec_from_record(record))
    if not specs:
        raise SimulationError("robot has no controlled joints")
    return tuple(specs)


def reset_controlled_joints(
    client: PhysicsClient, body_id: int, specs: tuple[JointSpec, ...]
) -> None:
    """Set every controlled joint pose and POSITION_CONTROL motor explicitly."""

    for spec in specs:
        client.reset_joint_state(body_id, spec.index, spec.rest_position)
        client.set_position_control(
            body_id, spec.index, spec.rest_position, force_newtons=spec.force_newtons
        )


def joint_states_from_specs(
    client: PhysicsClient, body_id: int, specs: tuple[JointSpec, ...]
) -> tuple[JointState, ...]:
    states: list[JointState] = []
    for spec in specs:
        states.append(
            JointState(
                name=spec.name,
                position=client.get_joint_position(body_id, spec.index),
                velocity=0.0,
                position_unit=spec.position_unit,
            )
        )
    return tuple(states)


def _spec_from_record(record: JointRecord) -> JointSpec:
    force = (
        record.max_force_newtons if record.max_force_newtons > 0.0 else FALLBACK_JOINT_FORCE_NEWTONS
    )
    unit = CanonicalUnit.METERS if record.is_prismatic else CanonicalUnit.RADIANS
    return JointSpec(
        index=record.index,
        name=record.name,
        rest_position=record.rest_position,
        force_newtons=force,
        position_unit=unit,
    )
