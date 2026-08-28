"""Seeded scenario generator: bounds, canonical checksums, and policy independence."""

from robot_control_platform_simulator.scenarios.generator import (
    AllowedPerturbations,
    ObjectTypeSpec,
    Scenario,
    ScenarioGenerationError,
    ScenarioGeneratorConfig,
    default_allowed_perturbations,
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

__all__ = [
    "GOLDEN_CHECKSUMS",
    "GOLDEN_SEEDS",
    "AllowedPerturbations",
    "ObjectTypeSpec",
    "Scenario",
    "ScenarioGenerationError",
    "ScenarioGeneratorConfig",
    "default_allowed_perturbations",
    "default_object_types",
    "default_scenario_generator_config",
    "deserialize_scenario",
    "generate_scenario",
    "generate_scenarios",
    "quaternion_from_yaw_radians",
    "scenario_checksum",
    "scenario_rng",
    "serialize_scenario",
]
