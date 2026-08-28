"""Canonical JSON serialization for generated scenarios.

Checksum input uses sorted keys, compact separators, and floats quantized to 12
decimal places by the generator so hashes are stable across libm implementations.
"""

from __future__ import annotations

import json

from robot_control_platform_simulator.domain.models import canonical_dumps, sha256_hex
from robot_control_platform_simulator.scenarios.generator import Scenario


def serialize_scenario(scenario: Scenario) -> str:
    """Return compact, sorted-key JSON used as checksum input."""

    return canonical_dumps(scenario.to_checksum_payload())


def deserialize_scenario(text: str) -> Scenario:
    """Parse canonical scenario JSON. Rejects extra or missing keys."""

    return Scenario.from_checksum_payload(json.loads(text))


def scenario_checksum(scenario: Scenario) -> str:
    """Return the lowercase SHA-256 hex digest of canonical scenario JSON."""

    return sha256_hex(scenario.to_checksum_payload())
