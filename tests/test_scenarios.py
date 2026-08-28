from __future__ import annotations

import inspect
import json
import math
from dataclasses import replace

import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st
from robot_control_platform_simulator.domain.models import Vector3, canonical_dumps, sha256_hex
from robot_control_platform_simulator.physics.client import WORLD_FRAME
from robot_control_platform_simulator.physics.scene import default_scene_config
from robot_control_platform_simulator.scenarios.generator import (
    OBJECT_SHAPE_BOX,
    AllowedPerturbations,
    ObjectTypeSpec,
    Scenario,
    ScenarioGenerationError,
    ScenarioGeneratorConfig,
    default_object_types,
    default_scenario_generator_config,
    generate_scenario,
    generate_scenarios,
    quaternion_from_yaw_radians,
    scenario_rng,
)
from robot_control_platform_simulator.scenarios.golden import GOLDEN_CHECKSUMS, GOLDEN_SEEDS
from robot_control_platform_simulator.scenarios.serializer import (
    deserialize_scenario,
    scenario_checksum,
    serialize_scenario,
)

_POLICY_NAMES = ("fixed", "pose_aware", "collision_aware")


def _huge_object() -> ObjectTypeSpec:
    return ObjectTypeSpec(
        name="oversized",
        category="oversized",
        shape=OBJECT_SHAPE_BOX,
        half_extents_meters=Vector3(x=0.50, y=0.50, z=0.05),
    )


def test_scenario_rng_is_local_pcg64_and_seeded() -> None:
    first = scenario_rng(5)
    second = scenario_rng(5)
    other = scenario_rng(6)
    assert type(first.bit_generator).__name__ == "PCG64"
    assert first.random() == second.random()
    assert scenario_rng(5).random() != other.random()


def test_generate_scenario_signature_excludes_policy_identity() -> None:
    parameters = inspect.signature(generate_scenario).parameters
    assert list(parameters) == ["seed", "config", "scene"]
    assert "policy" not in ScenarioGeneratorConfig.__dataclass_fields__
    for name in ("policy", "policy_id", "policy_version", "policy_name"):
        assert name not in parameters
        with pytest.raises(TypeError):
            generate_scenario(1, **{name: "fixed"})  # type: ignore[arg-type]


def test_golden_seeds_are_stable_and_distinct() -> None:
    checksums: list[str] = []
    for seed in GOLDEN_SEEDS:
        scenario = generate_scenario(seed)
        checksum = scenario_checksum(scenario)
        checksums.append(checksum)
        assert checksum == GOLDEN_CHECKSUMS[seed]
        assert checksum == scenario.sha256_hex()
        restored = deserialize_scenario(serialize_scenario(scenario))
        assert restored == scenario
        assert restored.sha256_hex() == checksum
        assert scenario.initial_pose.frame == WORLD_FRAME
        assert scenario.initial_pose.orientation_xyzw.to_checksum_payload() == [
            scenario.initial_pose.orientation_xyzw.x,
            scenario.initial_pose.orientation_xyzw.y,
            scenario.initial_pose.orientation_xyzw.z,
            scenario.initial_pose.orientation_xyzw.w,
        ]
    assert len(set(checksums)) == len(GOLDEN_SEEDS)


def test_generated_floats_are_quantized_for_canonical_json() -> None:
    scenario = generate_scenario(1)
    pose = scenario.initial_pose
    assert pose.position_meters.x == round(pose.position_meters.x, 12)
    assert pose.position_meters.y == round(pose.position_meters.y, 12)
    assert pose.orientation_xyzw.z == round(pose.orientation_xyzw.z, 12)
    assert pose.orientation_xyzw.w == round(pose.orientation_xyzw.w, 12)
    assert scenario.mass_kilograms == round(scenario.mass_kilograms, 12)
    assert scenario.lateral_friction == round(scenario.lateral_friction, 12)


def test_canonical_json_is_compact_sorted_and_hashed() -> None:
    scenario = generate_scenario(1)
    text = serialize_scenario(scenario)
    assert text == scenario.canonical_json()
    assert ": " not in text
    assert ", " not in text
    parsed = json.loads(text)
    assert list(parsed.keys()) == sorted(parsed.keys())
    assert parsed["schema_version"] == "1"
    assert parsed["generator_version"] == "1"
    assert parsed["seed"] == 1
    assert "mass_kilograms" in parsed
    assert "lateral_friction" in parsed
    shuffled = dict(reversed(list(parsed.items())))
    assert canonical_dumps(shuffled) == text
    assert sha256_hex(shuffled) == scenario.sha256_hex()


def test_same_seed_ignores_global_numpy_state() -> None:
    np.random.seed(0)
    first = generate_scenario(11)
    np.random.seed(99)
    np.random.default_rng(123)
    second = generate_scenario(11)
    assert first.canonical_json() == second.canonical_json()
    assert first.sha256_hex() == second.sha256_hex()


def test_each_seed_uses_an_independent_generator() -> None:
    scenarios = generate_scenarios((3, 4, 3))
    assert scenarios[0].sha256_hex() == generate_scenario(3).sha256_hex()
    assert scenarios[1].sha256_hex() == generate_scenario(4).sha256_hex()
    assert scenarios[2].sha256_hex() == scenarios[0].sha256_hex()
    assert scenarios[0].sha256_hex() != scenarios[1].sha256_hex()


def test_changing_bounds_changes_checksum() -> None:
    scene = default_scene_config()
    base = default_scenario_generator_config(scene)
    heavier = replace(base, mass_kilograms_min=0.15, mass_kilograms_max=0.20)
    first = generate_scenario(7, config=base, scene=scene)
    second = generate_scenario(7, config=heavier, scene=scene)
    assert first.sha256_hex() != second.sha256_hex()
    assert first.generator_config_checksum != second.generator_config_checksum


def test_seed_zero_is_allowed() -> None:
    scenario = generate_scenario(0)
    assert scenario.seed == 0
    assert generate_scenario(0).sha256_hex() == scenario.sha256_hex()


def test_invalid_seeds_are_rejected() -> None:
    with pytest.raises(ValueError, match="nonnegative integer"):
        generate_scenario(-1)
    true_seed: object = True
    with pytest.raises(ValueError, match="nonnegative integer"):
        generate_scenario(true_seed)  # type: ignore[arg-type]
    with pytest.raises(ValueError, match="nonnegative integer"):
        scenario_rng(1.5)  # type: ignore[arg-type]


def test_invalid_config_is_rejected() -> None:
    base = default_scenario_generator_config()
    with pytest.raises(ValueError, match="object_types must be a non-empty tuple"):
        replace(base, object_types=())
    with pytest.raises(ValueError, match="workcell bin names"):
        replace(base, target_bins=("bin_purple",))
    with pytest.raises(ValueError, match="mass_kilograms max"):
        replace(base, mass_kilograms_min=0.3, mass_kilograms_max=0.1)
    with pytest.raises(ValueError, match="placement_retry_limit"):
        replace(base, placement_retry_limit=0)
    with pytest.raises(ValueError, match="cylinder radius extents"):
        ObjectTypeSpec(
            name="bad_cyl",
            category="bad_cyl",
            shape="cylinder",
            half_extents_meters=Vector3(x=0.02, y=0.03, z=0.02),
        )


def test_oversized_object_exhausts_placement_retries() -> None:
    scene = default_scene_config()
    config = replace(
        default_scenario_generator_config(scene),
        object_types=(_huge_object(),),
        placement_retry_limit=8,
    )
    with pytest.raises(ScenarioGenerationError, match="placement retry exhausted for seed 1"):
        generate_scenario(1, config=config, scene=scene)


def test_bin_intersection_is_rejected() -> None:
    scene = default_scene_config()
    bin_pose = dict(scene.bin_poses)["bin_red"]
    config = replace(
        default_scenario_generator_config(scene),
        pickup_center_meters=bin_pose.position_meters,
        pickup_half_extents_meters=Vector3(x=0.04, y=0.04, z=0.001),
        placement_retry_limit=6,
    )
    with pytest.raises(ScenarioGenerationError, match="placement retry exhausted"):
        generate_scenario(2, config=config, scene=scene)


def test_yaw_quaternion_is_xyzw() -> None:
    identity = quaternion_from_yaw_radians(0.0)
    assert identity.to_checksum_payload() == [0.0, 0.0, 0.0, 1.0]
    quarter = quaternion_from_yaw_radians(math.pi / 2.0)
    assert quarter.x == 0.0
    assert quarter.y == 0.0
    assert quarter.z > 0.0
    assert quarter.w > 0.0


def test_default_object_types_and_bins_are_public_neutral() -> None:
    names = tuple(spec.name for spec in default_object_types())
    assert names == ("cube", "box", "cylinder", "tall_box")
    config = default_scenario_generator_config()
    assert config.target_bins == ("bin_red", "bin_green", "bin_blue", "bin_yellow")
    assert config.allowed_perturbations == AllowedPerturbations(
        pose_translation_noise_meters=0.0,
        yaw_noise_radians=0.0,
        camera_intensity_noise=0.0,
        lighting_scale_min=1.0,
        lighting_scale_max=1.0,
    )


def _footprint_inside_pickup(scenario: Scenario, config: ScenarioGeneratorConfig) -> None:
    spec = scenario.object_type
    x = scenario.initial_pose.position_meters.x
    y = scenario.initial_pose.position_meters.y
    quat = scenario.initial_pose.orientation_xyzw
    yaw = math.atan2(quat.z, quat.w) * 2.0
    xmin = config.pickup_center_meters.x - config.pickup_half_extents_meters.x
    xmax = config.pickup_center_meters.x + config.pickup_half_extents_meters.x
    ymin = config.pickup_center_meters.y - config.pickup_half_extents_meters.y
    ymax = config.pickup_center_meters.y + config.pickup_half_extents_meters.y
    if spec.shape == "cylinder":
        radius = spec.half_extents_meters.x
        assert x - radius >= xmin - 1e-12
        assert x + radius <= xmax + 1e-12
        assert y - radius >= ymin - 1e-12
        assert y + radius <= ymax + 1e-12
        return
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    hx = spec.half_extents_meters.x
    hy = spec.half_extents_meters.y
    for ox, oy in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
        cx = x + ox * cos_yaw - oy * sin_yaw
        cy = y + ox * sin_yaw + oy * cos_yaw
        assert xmin - 1e-12 <= cx <= xmax + 1e-12
        assert ymin - 1e-12 <= cy <= ymax + 1e-12


def _assert_in_bounds(scenario_seed: int) -> None:
    config = default_scenario_generator_config()
    scenario = generate_scenario(scenario_seed, config=config)
    names = {item.name for item in config.object_types}
    assert scenario.object_type.name in names
    assert scenario.target_bin in config.target_bins
    assert config.mass_kilograms_min <= scenario.mass_kilograms <= config.mass_kilograms_max
    assert config.lateral_friction_min <= scenario.lateral_friction <= config.lateral_friction_max
    z = scenario.initial_pose.position_meters.z
    assert z == config.table_top_z_meters + scenario.object_type.half_extents_meters.z
    _footprint_inside_pickup(scenario, config)
    assert scenario.generator_config_checksum == config.sha256_hex()
    assert len(scenario.sha256_hex()) == 64
    assert scenario.sha256_hex() == scenario.sha256_hex().lower()


@given(st.integers(min_value=0, max_value=10_000))
@settings(max_examples=40, deadline=None)
def test_hypothesis_values_stay_within_config_bounds(seed: int) -> None:
    _assert_in_bounds(seed)
    again = generate_scenario(seed)
    assert again.sha256_hex() == generate_scenario(seed).sha256_hex()


@given(
    st.integers(min_value=0, max_value=10_000),
    st.sampled_from(_POLICY_NAMES),
)
@settings(max_examples=40, deadline=None)
def test_hypothesis_checksum_is_independent_of_policy_identity(seed: int, policy_name: str) -> None:
    first = generate_scenario(seed)
    second = generate_scenario(seed)
    assert first.canonical_json() == second.canonical_json()
    assert policy_name not in first.canonical_json()
    assert "policy" not in first.to_checksum_payload()
