"""Atomic filesystem artifact store behind the ``ArtifactStore`` contract."""

from __future__ import annotations

import os
import secrets
from collections.abc import Mapping
from pathlib import Path
from typing import BinaryIO, Final

from robot_control_platform_common.artifacts.base import (
    ARTIFACT_MEDIA_TYPES,
    DEFAULT_MAX_BYTES_BY_KIND,
    ArtifactConflictError,
    ArtifactIntegrityError,
    ArtifactKeyError,
    ArtifactMetadata,
    ArtifactNotFoundError,
    ArtifactSizeError,
    ArtifactStoreError,
    parse_artifact_storage_key,
    sha256_hex_bytes,
)
from robot_control_platform_common.time import utc_now

_QUARANTINE_PREFIX: Final[str] = "_quarantine"
_TEMP_PREFIX: Final[str] = ".tmp-"


class FilesystemArtifactStore:
    """Store artifact bytes under an absolute root using atomic temp/rename writes."""

    def __init__(
        self,
        root: Path,
        *,
        max_bytes_by_kind: Mapping[str, int] | None = None,
    ) -> None:
        if not isinstance(root, Path):
            msg = "artifact root must be a Path"
            raise ArtifactStoreError(msg)
        if not root.is_absolute():
            msg = "artifact root must be an absolute path"
            raise ArtifactStoreError(msg)
        if root.exists() and root.is_symlink():
            msg = "artifact root must not be a symbolic link"
            raise ArtifactStoreError(msg)
        self._root = root.resolve(strict=False)
        self._max_bytes_by_kind = dict(max_bytes_by_kind or DEFAULT_MAX_BYTES_BY_KIND)
        self._root.mkdir(parents=True, exist_ok=True)
        if self._root.is_symlink():
            msg = "artifact root must not be a symbolic link"
            raise ArtifactStoreError(msg)

    @property
    def root(self) -> Path:
        """Absolute store root. Not part of the public ArtifactStore protocol."""

        return self._root

    def write(self, storage_key: str, data: bytes) -> ArtifactMetadata:
        _experiment_id, _trial_id, kind = parse_artifact_storage_key(storage_key)
        if not isinstance(data, (bytes, bytearray)):
            msg = "artifact payload must be bytes"
            raise ArtifactStoreError(msg)
        payload = bytes(data)
        max_bytes = self._max_bytes_by_kind.get(kind)
        if max_bytes is None:
            msg = f"no size limit configured for kind {kind}"
            raise ArtifactStoreError(msg)
        if len(payload) > max_bytes:
            msg = f"artifact exceeds maximum size for kind {kind}"
            raise ArtifactSizeError(msg)
        if len(payload) == 0:
            msg = "artifact payload must not be empty"
            raise ArtifactSizeError(msg)

        destination = self._resolved_path(storage_key)
        if destination.is_symlink():
            msg = "refusing to write through a symbolic link"
            raise ArtifactKeyError(msg)
        if destination.exists():
            msg = f"standard artifact kind already exists: {kind}"
            raise ArtifactConflictError(msg)

        destination.parent.mkdir(parents=True, exist_ok=True)
        self._assert_directory_safe(destination.parent)

        digest = sha256_hex_bytes(payload)
        temp_name = f"{_TEMP_PREFIX}{destination.name}.{secrets.token_hex(8)}"
        temp_path = destination.parent / temp_name
        try:
            with open(temp_path, "wb") as handle:
                handle.write(payload)
                handle.flush()
                os.fsync(handle.fileno())
            if temp_path.is_symlink():
                msg = "temporary artifact path resolved to a symbolic link"
                raise ArtifactKeyError(msg)
            written_size = temp_path.stat().st_size
            if written_size != len(payload):
                msg = "temporary artifact size does not match payload"
                raise ArtifactIntegrityError(msg)
            os.replace(temp_path, destination)
            self._fsync_directory(destination.parent)
        except Exception:
            if temp_path.exists():
                temp_path.unlink(missing_ok=True)
            raise

        created_at = utc_now()
        return ArtifactMetadata(
            storage_key=storage_key,
            kind=kind,
            media_type=ARTIFACT_MEDIA_TYPES[kind],
            byte_size=len(payload),
            sha256=digest,
            created_at=created_at,
        )

    def open(self, storage_key: str) -> BinaryIO:
        path = self._require_durable_file(storage_key)
        return open(path, "rb")

    def stat(self, storage_key: str) -> ArtifactMetadata:
        path = self._require_durable_file(storage_key)
        _, _, kind = parse_artifact_storage_key(storage_key)
        size = path.stat().st_size
        # Stat does not re-hash; verify() is the integrity authority.
        # Still return a digest computed from current bytes so callers can persist it.
        digest = sha256_hex_bytes(path.read_bytes())
        return ArtifactMetadata(
            storage_key=storage_key,
            kind=kind,
            media_type=ARTIFACT_MEDIA_TYPES[kind],
            byte_size=size,
            sha256=digest,
            created_at=utc_now(),
        )

    def verify(self, storage_key: str, *, expected_sha256: str | None = None) -> ArtifactMetadata:
        path = self._require_durable_file(storage_key)
        _, _, kind = parse_artifact_storage_key(storage_key)
        payload = path.read_bytes()
        digest = sha256_hex_bytes(payload)
        if expected_sha256 is not None:
            expected = expected_sha256.lower()
            if len(expected) != 64 or any(ch not in "0123456789abcdef" for ch in expected):
                msg = "expected_sha256 must be lowercase hexadecimal of length 64"
                raise ArtifactIntegrityError(msg)
            if digest != expected:
                msg = "artifact checksum mismatch"
                raise ArtifactIntegrityError(msg)
        max_bytes = self._max_bytes_by_kind.get(kind)
        if max_bytes is not None and len(payload) > max_bytes:
            msg = "artifact exceeds maximum size for kind"
            raise ArtifactIntegrityError(msg)
        if len(payload) == 0:
            msg = "artifact bytes are empty"
            raise ArtifactIntegrityError(msg)
        return ArtifactMetadata(
            storage_key=storage_key,
            kind=kind,
            media_type=ARTIFACT_MEDIA_TYPES[kind],
            byte_size=len(payload),
            sha256=digest,
            created_at=utc_now(),
        )

    def quarantine(self, storage_key: str, *, reason: str) -> str:
        if not isinstance(reason, str) or reason.strip() == "":
            msg = "quarantine reason must be a non-empty string"
            raise ArtifactStoreError(msg)
        parse_artifact_storage_key(storage_key)
        source = self._resolved_path(storage_key)
        if not source.exists():
            msg = f"artifact not found: {storage_key}"
            raise ArtifactNotFoundError(msg)
        if source.is_symlink():
            msg = "refusing to quarantine through a symbolic link source"
            raise ArtifactKeyError(msg)

        stamp = utc_now().strftime("%Y%m%dT%H%M%S%fZ")
        quarantine_key = f"{_QUARANTINE_PREFIX}/{stamp}/{storage_key}"
        destination = self._root / quarantine_key
        destination.parent.mkdir(parents=True, exist_ok=True)
        self._assert_under_root(destination)
        os.replace(source, destination)
        self._fsync_directory(destination.parent)
        reason_path = destination.with_suffix(destination.suffix + ".reason.txt")
        reason_path.write_text(reason.strip() + "\n", encoding="utf-8")
        return quarantine_key

    def list_storage_keys(self, *, include_quarantine: bool = False) -> tuple[str, ...]:
        """List durable storage keys under the root. Used by reconciliation."""

        keys: list[str] = []
        if not self._root.exists():
            return ()
        for path in sorted(self._root.rglob("*")):
            if not path.is_file() or path.is_symlink():
                continue
            if path.name.startswith(_TEMP_PREFIX):
                continue
            if path.name.endswith(".reason.txt"):
                continue
            relative = path.relative_to(self._root).as_posix()
            if relative.startswith(f"{_QUARANTINE_PREFIX}/") and not include_quarantine:
                continue
            if relative.startswith(f"{_QUARANTINE_PREFIX}/"):
                continue
            try:
                parse_artifact_storage_key(relative)
            except ArtifactKeyError:
                continue
            keys.append(relative)
        return tuple(keys)

    def _require_durable_file(self, storage_key: str) -> Path:
        path = self._path_for_key(storage_key)
        if not path.exists():
            msg = f"artifact not found: {storage_key}"
            raise ArtifactNotFoundError(msg)
        if path.is_symlink():
            msg = "refusing to read a symbolic-link artifact"
            raise ArtifactKeyError(msg)
        if not path.is_file():
            msg = f"artifact is not a regular file: {storage_key}"
            raise ArtifactIntegrityError(msg)
        return path

    def _path_for_key(self, storage_key: str) -> Path:
        parse_artifact_storage_key(storage_key)
        current = self._root
        for part in storage_key.split("/"):
            current = current / part
            if current.is_symlink():
                msg = "refusing path that includes a symbolic link"
                raise ArtifactKeyError(msg)
        resolved = current.resolve(strict=False)
        self._assert_under_root(resolved)
        return current

    def _resolved_path(self, storage_key: str) -> Path:
        return self._path_for_key(storage_key)

    def _assert_under_root(self, path: Path) -> None:
        try:
            path.relative_to(self._root)
        except ValueError as exc:
            msg = "storage path escapes artifact root"
            raise ArtifactKeyError(msg) from exc

    def _assert_directory_safe(self, directory: Path) -> None:
        current = directory
        while True:
            if current.is_symlink():
                msg = "artifact directory path must not include symbolic links"
                raise ArtifactKeyError(msg)
            if current == self._root or current.parent == current:
                break
            current = current.parent

    @staticmethod
    def _fsync_directory(directory: Path) -> None:
        flags = os.O_RDONLY
        if hasattr(os, "O_DIRECTORY"):
            flags |= os.O_DIRECTORY
        try:
            fd = os.open(directory, flags)
        except OSError:
            return
        try:
            os.fsync(fd)
        except OSError:
            pass
        finally:
            os.close(fd)
