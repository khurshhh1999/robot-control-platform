"""Artifact store contract, storage-key validation, and metadata types.

Callers never supply filesystem paths. All access uses generated storage keys of
the form ``<experiment>/<trial>/<kind>.<extension>``.
"""

from __future__ import annotations

import hashlib
import re
from dataclasses import dataclass
from datetime import datetime
from typing import BinaryIO, Final, Protocol, runtime_checkable
from uuid import UUID

STANDARD_ARTIFACT_KINDS: Final[frozenset[str]] = frozenset(
    {
        "initial_rgb",
        "pre_grasp_rgb",
        "post_grasp_rgb",
        "pre_release_rgb",
        "terminal_rgb",
        "trajectory",
        "trial_manifest",
    }
)
MILESTONE_RGB_KINDS: Final[frozenset[str]] = frozenset(
    {
        "initial_rgb",
        "pre_grasp_rgb",
        "post_grasp_rgb",
        "pre_release_rgb",
        "terminal_rgb",
    }
)
ARTIFACT_EXTENSIONS: Final[dict[str, str]] = {
    "initial_rgb": "png",
    "pre_grasp_rgb": "png",
    "post_grasp_rgb": "png",
    "pre_release_rgb": "png",
    "terminal_rgb": "png",
    "trajectory": "json.gz",
    "trial_manifest": "json",
}
ARTIFACT_MEDIA_TYPES: Final[dict[str, str]] = {
    "initial_rgb": "image/png",
    "pre_grasp_rgb": "image/png",
    "post_grasp_rgb": "image/png",
    "pre_release_rgb": "image/png",
    "terminal_rgb": "image/png",
    "trajectory": "application/gzip",
    "trial_manifest": "application/json",
}
DEFAULT_MAX_BYTES_BY_KIND: Final[dict[str, int]] = {
    "initial_rgb": 8 * 1024 * 1024,
    "pre_grasp_rgb": 8 * 1024 * 1024,
    "post_grasp_rgb": 8 * 1024 * 1024,
    "pre_release_rgb": 8 * 1024 * 1024,
    "terminal_rgb": 8 * 1024 * 1024,
    "trajectory": 32 * 1024 * 1024,
    "trial_manifest": 1 * 1024 * 1024,
}
_UUID_PATTERN: Final[re.Pattern[str]] = re.compile(
    r"^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$"
)
_KIND_PATTERN: Final[re.Pattern[str]] = re.compile(r"^[a-z][a-z0-9_]*$")


class ArtifactStoreError(ValueError):
    """Base error for artifact key, integrity, and store failures."""


class ArtifactKeyError(ArtifactStoreError):
    """Raised when a storage key is invalid or escapes the store root."""


class ArtifactConflictError(ArtifactStoreError):
    """Raised when a standard artifact kind already exists for a trial."""


class ArtifactNotFoundError(ArtifactStoreError):
    """Raised when an artifact key has no durable bytes."""


class ArtifactIntegrityError(ArtifactStoreError):
    """Raised when stored bytes fail size or checksum verification."""


class ArtifactSizeError(ArtifactStoreError):
    """Raised when payload bytes exceed the configured kind limit."""


@dataclass(frozen=True)
class ArtifactMetadata:
    """Durable artifact metadata. Database rows store these fields, not bytes."""

    storage_key: str
    kind: str
    media_type: str
    byte_size: int
    sha256: str
    created_at: datetime

    def __post_init__(self) -> None:
        if self.kind not in STANDARD_ARTIFACT_KINDS:
            msg = "kind is not a standard artifact kind"
            raise ValueError(msg)
        if self.byte_size < 0:
            msg = "byte_size must be nonnegative"
            raise ValueError(msg)
        if len(self.sha256) != 64 or any(ch not in "0123456789abcdef" for ch in self.sha256):
            msg = "sha256 must be lowercase hexadecimal of length 64"
            raise ValueError(msg)
        if self.created_at.tzinfo is None:
            msg = "created_at must be timezone-aware UTC"
            raise ValueError(msg)


def sha256_hex_bytes(data: bytes) -> str:
    """Return the lowercase SHA-256 hex digest of raw artifact bytes."""

    return hashlib.sha256(data).hexdigest()


def validate_entity_id(field: str, value: object) -> UUID:
    """Validate an experiment or trial identifier as a lowercase UUID string."""

    if isinstance(value, UUID):
        identifier = value
    elif isinstance(value, str):
        text = value.strip()
        if text != value or not _UUID_PATTERN.fullmatch(text):
            msg = f"{field} must be a lowercase UUID string"
            raise ArtifactKeyError(msg)
        try:
            identifier = UUID(text)
        except ValueError as exc:
            msg = f"{field} must be a valid UUID"
            raise ArtifactKeyError(msg) from exc
    else:
        msg = f"{field} must be a UUID"
        raise ArtifactKeyError(msg)
    if identifier.version != 7:
        msg = f"{field} must be a UUIDv7"
        raise ArtifactKeyError(msg)
    return identifier


def validate_artifact_kind(kind: object) -> str:
    if not isinstance(kind, str) or not _KIND_PATTERN.fullmatch(kind):
        msg = "artifact kind is invalid"
        raise ArtifactKeyError(msg)
    if kind not in STANDARD_ARTIFACT_KINDS:
        msg = "artifact kind is not a standard kind"
        raise ArtifactKeyError(msg)
    return kind


def artifact_storage_key(experiment_id: object, trial_id: object, kind: object) -> str:
    """Build ``<experiment>/<trial>/<kind>.<extension>`` from validated IDs/enums."""

    experiment = validate_entity_id("experiment_id", experiment_id)
    trial = validate_entity_id("trial_id", trial_id)
    kind_name = validate_artifact_kind(kind.value if hasattr(kind, "value") else kind)
    extension = ARTIFACT_EXTENSIONS[kind_name]
    return f"{experiment}/{trial}/{kind_name}.{extension}"


def parse_artifact_storage_key(storage_key: object) -> tuple[UUID, UUID, str]:
    """Parse and validate a storage key. Rejects traversal and absolute paths."""

    if not isinstance(storage_key, str) or storage_key.strip() == "":
        msg = "storage_key must be a non-empty string"
        raise ArtifactKeyError(msg)
    key = storage_key
    if key != key.strip():
        msg = "storage_key must not have leading or trailing whitespace"
        raise ArtifactKeyError(msg)
    if key.startswith("/") or key.startswith("\\") or ":" in key:
        msg = "storage_key must not be an absolute path"
        raise ArtifactKeyError(msg)
    if "\\" in key:
        msg = "storage_key must use forward slashes only"
        raise ArtifactKeyError(msg)
    if ".." in key.split("/"):
        msg = "storage_key must not contain path traversal"
        raise ArtifactKeyError(msg)
    if "//" in key or key.startswith("./") or "/./" in key:
        msg = "storage_key must not contain relative path segments"
        raise ArtifactKeyError(msg)
    parts = key.split("/")
    if len(parts) != 3:
        msg = "storage_key must be <experiment>/<trial>/<kind>.<extension>"
        raise ArtifactKeyError(msg)
    experiment_text, trial_text, filename = parts
    experiment = validate_entity_id("experiment_id", experiment_text)
    trial = validate_entity_id("trial_id", trial_text)
    if filename.count(".") < 1:
        msg = "storage_key filename must include an extension"
        raise ArtifactKeyError(msg)
    # Prefer longest known extension (trajectory uses ``json.gz``).
    kind_name: str | None = None
    for candidate, extension in ARTIFACT_EXTENSIONS.items():
        expected = f"{candidate}.{extension}"
        if filename == expected:
            kind_name = candidate
            break
    if kind_name is None:
        msg = "storage_key filename does not match a standard artifact kind"
        raise ArtifactKeyError(msg)
    return experiment, trial, kind_name


@runtime_checkable
class ArtifactStore(Protocol):
    """Artifact byte store. Exposes no arbitrary filesystem path API."""

    def write(self, storage_key: str, data: bytes) -> ArtifactMetadata:
        """Atomically write bytes and return checksummed metadata."""

    def open(self, storage_key: str) -> BinaryIO:
        """Open durable artifact bytes for reading."""

    def stat(self, storage_key: str) -> ArtifactMetadata:
        """Return metadata for a durable artifact without exposing paths."""

    def verify(self, storage_key: str, *, expected_sha256: str | None = None) -> ArtifactMetadata:
        """Verify size and SHA-256 of durable bytes."""

    def quarantine(self, storage_key: str, *, reason: str) -> str:
        """Move a suspicious artifact into quarantine. Returns quarantine key."""
