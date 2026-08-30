from __future__ import annotations

from dataclasses import dataclass

import pytest
from robot_control_platform_simulator.control.state_machine import ControllerStateMachine
from robot_control_platform_simulator.domain.enums import ControllerState, EventType
from robot_control_platform_simulator.domain.events import ContactEvent
from robot_control_platform_simulator.domain.models import Vector3
from robot_control_platform_simulator.physics.client import (
    JointRecord,
    RawContactPoint,
    SimulationError,
    parse_engine_contact_point,
)
from robot_control_platform_simulator.physics.contacts import (
    ANY_LINK,
    BASE_LINK_NAME,
    CollisionConfig,
    CollisionMonitor,
    ProhibitedPair,
    default_collision_config,
    sample_contacts,
)
from robot_control_platform_simulator.physics.scene import ROBOT_BODY_NAME, SCENE_BODY_NAMES


@dataclass(frozen=True)
class FakeContactClient:
    points: tuple[RawContactPoint, ...]
    records: dict[int, tuple[JointRecord, ...]]

    def get_contact_points(self) -> tuple[RawContactPoint, ...]:
        return self.points

    def joint_records(self, body_id: int) -> tuple[JointRecord, ...]:
        return self.records.get(body_id, ())


def _record(index: int, link_name: str) -> JointRecord:
    return JointRecord(
        index=index,
        name=f"joint_{index}",
        link_name=link_name,
        joint_type=0,
        lower_limit=-1.0,
        upper_limit=1.0,
        max_force_newtons=50.0,
        rest_position=0.0,
        is_prismatic=False,
    )


def _raw(
    *,
    body_a: int = 4,
    body_b: int = 7,
    link_a: int = 3,
    link_b: int = -1,
    position: tuple[float, float, float] = (0.4, 0.1, 0.7),
    normal: tuple[float, float, float] = (0.0, 0.0, 2.0),
    force_newtons: float = 4.5,
) -> RawContactPoint:
    return RawContactPoint(
        body_unique_id_a=body_a,
        body_unique_id_b=body_b,
        link_index_a=link_a,
        link_index_b=link_b,
        position_world_on_a_meters=Vector3.from_xyz(position),
        contact_normal_on_b=Vector3.from_xyz(normal),
        normal_force_newtons=force_newtons,
    )


def _contact(
    *,
    body_a: str = "kuka_iiwa",
    body_b: str = "bin_red",
    link_a: str = "lbr_iiwa_link_4",
    link_b: str = "base",
    force_newtons: float = 10.0,
    time_seconds: float = 1.0,
) -> ContactEvent:
    return ContactEvent(
        body_a=body_a,
        body_b=body_b,
        link_a=link_a,
        link_b=link_b,
        position_meters=Vector3(x=0.4, y=0.0, z=0.7),
        normal=Vector3(x=0.0, y=0.0, z=1.0),
        force_newtons=force_newtons,
        simulation_time_seconds=time_seconds,
    )


def _monitor(
    *,
    force_threshold_newtons: float = 5.0,
    duration_threshold_seconds: float = 0.05,
    pairs: tuple[ProhibitedPair, ...] | None = None,
) -> CollisionMonitor:
    if pairs is None:
        pairs = (
            ProhibitedPair(
                body_a="kuka_iiwa",
                link_a=ANY_LINK,
                body_b="bin_red",
                link_b=ANY_LINK,
            ),
        )
    return CollisionMonitor(
        CollisionConfig(
            prohibited_pairs=pairs,
            force_threshold_newtons=force_threshold_newtons,
            duration_threshold_seconds=duration_threshold_seconds,
        )
    )


def test_parse_engine_contact_point_reads_pybullet_layout() -> None:
    parsed = parse_engine_contact_point(
        (
            0,
            1,
            2,
            -1,
            3,
            (0.1, 0.2, 0.3),
            (0.11, 0.21, 0.31),
            (0.0, 0.0, 2.0),
            -0.01,
            4.5,
            "ignored",
        )
    )
    assert parsed.body_unique_id_a == 1
    assert parsed.body_unique_id_b == 2
    assert parsed.link_index_a == -1
    assert parsed.link_index_b == 3
    assert parsed.position_world_on_a_meters == Vector3(x=0.1, y=0.2, z=0.3)
    assert parsed.contact_normal_on_b == Vector3(x=0.0, y=0.0, z=2.0)
    assert parsed.normal_force_newtons == 4.5
    with pytest.raises(SimulationError, match="incomplete contact point"):
        parse_engine_contact_point((0, 1, 2))
    with pytest.raises(SimulationError, match="body id"):
        parse_engine_contact_point(
            (0, -1, 2, 0, 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 0.0, 1.0)
        )


def test_sample_contacts_normalizes_names_unit_normal_force_and_time() -> None:
    client = FakeContactClient(
        points=(_raw(),),
        records={4: (_record(3, "lbr_iiwa_link_4"),)},
    )
    contacts = sample_contacts(
        client,
        {4: "kuka_iiwa", 7: "bin_red"},
        simulation_time_seconds=1.25,
    )
    assert len(contacts) == 1
    contact = contacts[0]
    assert contact.body_a == "kuka_iiwa"
    assert contact.body_b == "bin_red"
    assert contact.link_a == "lbr_iiwa_link_4"
    assert contact.link_b == BASE_LINK_NAME
    assert contact.position_meters == Vector3(x=0.4, y=0.1, z=0.7)
    assert contact.normal == Vector3(x=0.0, y=0.0, z=1.0)
    assert contact.force_newtons == 4.5
    assert contact.simulation_time_seconds == 1.25


def test_sample_contacts_rejects_unknown_bodies_and_zero_normals() -> None:
    named = FakeContactClient(points=(_raw(body_b=99),), records={4: (_record(3, "link_3"),)})
    with pytest.raises(SimulationError, match="unknown body") as unknown:
        sample_contacts(named, {4: "kuka_iiwa", 7: "bin_red"}, simulation_time_seconds=0.0)
    assert "99" not in str(unknown.value)
    zero = FakeContactClient(
        points=(_raw(normal=(0.0, 0.0, 0.0)),),
        records={4: (_record(3, "lbr_iiwa_link_4"),)},
    )
    with pytest.raises(SimulationError, match="contact normal is invalid"):
        sample_contacts(zero, {4: "kuka_iiwa", 7: "bin_red"}, simulation_time_seconds=0.0)
    missing_link = FakeContactClient(points=(_raw(link_a=8),), records={4: (_record(3, "link_3"),)})
    with pytest.raises(SimulationError, match="unknown link"):
        sample_contacts(missing_link, {4: "kuka_iiwa", 7: "bin_red"}, simulation_time_seconds=0.0)


def test_below_force_threshold_is_an_event_not_a_collision() -> None:
    monitor = _monitor(force_threshold_newtons=5.0, duration_threshold_seconds=0.0)
    first = monitor.observe(
        (_contact(force_newtons=1.0, time_seconds=1.0),), simulation_time_seconds=1.0
    )
    second = monitor.observe(
        (_contact(force_newtons=1.0, time_seconds=2.0),), simulation_time_seconds=2.0
    )
    assert first.contacts[0].force_newtons == 1.0
    assert first.collision_detected is False
    assert second.collision_detected is False
    assert second.collision_contacts == ()


def test_force_without_duration_is_an_event_not_a_collision() -> None:
    monitor = _monitor(force_threshold_newtons=5.0, duration_threshold_seconds=0.05)
    assessment = monitor.observe(
        (_contact(force_newtons=10.0, time_seconds=1.0),), simulation_time_seconds=1.0
    )
    assert assessment.collision_detected is False
    assert assessment.collision_duration_seconds == 0.0
    assert assessment.contacts[0].force_newtons == 10.0


def test_force_and_duration_thresholds_classify_a_collision() -> None:
    monitor = _monitor(force_threshold_newtons=5.0, duration_threshold_seconds=0.05)
    monitor.observe((_contact(force_newtons=10.0, time_seconds=1.0),), simulation_time_seconds=1.0)
    assessment = monitor.observe(
        (_contact(force_newtons=9.0, time_seconds=1.06),), simulation_time_seconds=1.06
    )
    assert assessment.collision_detected is True
    assert assessment.collision_duration_seconds == pytest.approx(0.06)
    assert assessment.collision_contacts[0].body_b == "bin_red"


def test_non_prohibited_high_force_contact_stays_harmless() -> None:
    monitor = _monitor()
    parcel = _contact(body_b="parcel_0", force_newtons=80.0, time_seconds=1.0)
    first = monitor.observe((parcel,), simulation_time_seconds=1.0)
    second = monitor.observe(
        (_contact(body_b="parcel_0", force_newtons=80.0, time_seconds=2.0),),
        simulation_time_seconds=2.0,
    )
    assert first.collision_detected is False
    assert second.collision_detected is False
    assert second.contacts[0].body_b == "parcel_0"


def test_duration_resets_when_prohibited_contact_clears() -> None:
    monitor = _monitor(duration_threshold_seconds=0.05)
    monitor.observe((_contact(time_seconds=1.0),), simulation_time_seconds=1.0)
    cleared = monitor.observe((), simulation_time_seconds=1.06)
    assert cleared.collision_detected is False
    restarted = monitor.observe((_contact(time_seconds=1.12),), simulation_time_seconds=1.12)
    assert restarted.collision_detected is False
    assert restarted.collision_duration_seconds == 0.0
    held = monitor.observe((_contact(time_seconds=1.18),), simulation_time_seconds=1.18)
    assert held.collision_detected is True
    assert held.collision_duration_seconds == pytest.approx(0.06)


def test_prohibited_pairs_match_undirected_and_wildcard_links() -> None:
    pair = ProhibitedPair(
        body_a="kuka_iiwa",
        link_a=ANY_LINK,
        body_b="bin_red",
        link_b=ANY_LINK,
    )
    reversed_contact = _contact(
        body_a="bin_red", body_b="kuka_iiwa", link_a="base", link_b="left_finger"
    )
    assert pair.matches(reversed_contact)
    exact = ProhibitedPair(
        body_a="kuka_iiwa",
        link_a="lbr_iiwa_link_4",
        body_b="table",
        link_b="base",
    )
    assert exact.matches(_contact(body_b="table", link_b="base"))
    assert not exact.matches(
        _contact(body_b="table", link_a="left_finger", link_b="base", force_newtons=20.0)
    )


def test_zero_duration_threshold_collides_on_the_first_force_sample() -> None:
    monitor = _monitor(duration_threshold_seconds=0.0)
    assessment = monitor.observe(
        (_contact(force_newtons=10.0, time_seconds=0.4),), simulation_time_seconds=0.4
    )
    assert assessment.collision_detected is True
    assert assessment.collision_duration_seconds == 0.0


def test_default_collision_config_is_stable_and_excludes_visual_pickup() -> None:
    first = default_collision_config()
    second = default_collision_config()
    assert first.canonical_json() == second.canonical_json()
    assert first.sha256_hex() == second.sha256_hex()
    bodies = {(pair.body_a, pair.body_b) for pair in first.prohibited_pairs}
    assert (ROBOT_BODY_NAME, "bin_red") in bodies or ("bin_red", ROBOT_BODY_NAME) in bodies
    involved = {pair.body_a for pair in first.prohibited_pairs} | {
        pair.body_b for pair in first.prohibited_pairs
    }
    assert "pickup_region" not in involved
    assert involved <= set(SCENE_BODY_NAMES)


def test_collision_during_approach_is_recorded_then_aborted_by_the_controller() -> None:
    machine = ControllerStateMachine()
    machine.transition(ControllerState.OBSERVE, simulation_time_seconds=0.1)
    machine.transition(ControllerState.PLAN, simulation_time_seconds=0.2)
    machine.transition(ControllerState.APPROACH, simulation_time_seconds=0.3)
    monitor = _monitor(duration_threshold_seconds=0.0)
    contact = _contact(force_newtons=12.0, time_seconds=0.4)
    assessment = monitor.observe((contact,), simulation_time_seconds=0.4)
    recorded = machine.record_contact(contact)
    assert assessment.collision_detected is True
    assert recorded.event_type is EventType.CONTACT
    machine.transition(
        ControllerState.RETRACT, simulation_time_seconds=0.4, failed=True, detail="collision"
    )
    assert machine.state is ControllerState.RETRACT
    assert machine.events[-2].event_type is EventType.STATE_FAILURE
    assert machine.events[-2].controller_state is ControllerState.APPROACH


def test_workcell_body_map_uses_logical_names() -> None:
    pytest.importorskip("pybullet")
    from robot_control_platform_simulator.physics.client import PhysicsClient
    from robot_control_platform_simulator.physics.scene import WorkcellScene

    with PhysicsClient(gui=False) as client:
        scene = WorkcellScene(client)
        scene.reset()
        lookup = scene.body_id_to_name()
        assert set(lookup.values()) == set(SCENE_BODY_NAMES)
        assert all(isinstance(body_id, int) for body_id in lookup)
        assert lookup[scene.body_id("table")] == "table"
