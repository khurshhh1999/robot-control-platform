"""Scenario set API schemas."""

from __future__ import annotations

from typing import Any

from pydantic import Field, field_validator

from robot_control_platform_api.schemas import ApiModel

_SHA256_LEN = 64


def _require_sha256(value: str) -> str:
    digest = value.lower()
    if len(digest) != _SHA256_LEN or any(ch not in "0123456789abcdef" for ch in digest):
        msg = "checksum must be lowercase hexadecimal of length 64"
        raise ValueError(msg)
    return digest


class ScenarioCreate(ApiModel):
    """One scenario belonging to a newly created scenario set."""

    ordinal: int = Field(ge=0, examples=[0])
    seed: int = Field(ge=0, examples=[1])
    object_name: str = Field(min_length=1, examples=["cube"])
    object_category: str = Field(min_length=1, examples=["parcel"])
    target_bin: str = Field(min_length=1, examples=["bin_a"])
    initial_pose: dict[str, Any] = Field(
        examples=[{"position_m": [0.0, 0.0, 0.1], "orientation_xyzw": [0, 0, 0, 1]}]
    )
    physical_properties: dict[str, Any] = Field(examples=[{"mass_kg": 0.2, "friction": 0.5}])
    checksum: str = Field(examples=["a" * 64])

    @field_validator("checksum")
    @classmethod
    def _checksum(cls, value: str) -> str:
        return _require_sha256(value)


class ScenarioSetCreate(ApiModel):
    """Create a frozen scenario set with embedded scenarios."""

    name: str = Field(min_length=1, max_length=200, examples=["demo-set"])
    generator_version: str = Field(min_length=1, examples=["1.0.0"])
    seed_manifest: dict[str, Any] = Field(examples=[{"seeds": [1]}])
    scene_config: dict[str, Any] = Field(examples=[{"scene": "baseline"}])
    checksum: str = Field(examples=["a" * 64])
    scenarios: list[ScenarioCreate] = Field(min_length=1)

    @field_validator("checksum")
    @classmethod
    def _checksum(cls, value: str) -> str:
        return _require_sha256(value)


class ScenarioResponse(ApiModel):
    """Scenario metadata returned with a scenario set."""

    id: str
    ordinal: int
    seed: int
    object_name: str
    object_category: str
    target_bin: str
    initial_pose: dict[str, Any]
    physical_properties: dict[str, Any]
    checksum: str


class ScenarioSetResponse(ApiModel):
    """Frozen scenario set with member scenarios."""

    id: str = Field(examples=["0193f1a2-b3c4-7d8e-9f01-23456789abcd"])
    name: str = Field(examples=["demo-set"])
    generator_version: str = Field(examples=["1.0.0"])
    scenario_count: int = Field(examples=[1])
    seed_manifest: dict[str, Any]
    scene_config: dict[str, Any]
    checksum: str
    scenarios: list[ScenarioResponse]
