"""Tests for atomic artifact store integrity and reconciliation."""

from __future__ import annotations

import os
from pathlib import Path

import pytest
from robot_control_platform_common.artifacts import (
    ArtifactConflictError,
    ArtifactIntegrityError,
    ArtifactKeyError,
    ArtifactNotFoundError,
    ArtifactSizeError,
    FilesystemArtifactStore,
    ReconciliationIssueKind,
    artifact_storage_key,
    reconcile_artifacts,
    sha256_hex_bytes,
)
from robot_control_platform_common.ids import new_id


@pytest.fixture
def store(tmp_path: Path) -> FilesystemArtifactStore:
    root = tmp_path / "artifacts"
    return FilesystemArtifactStore(root)


def _key(kind: str = "initial_rgb") -> str:
    return artifact_storage_key(new_id(), new_id(), kind)


def test_write_open_stat_verify_round_trip(store: FilesystemArtifactStore) -> None:
    key = _key()
    payload = b"png-bytes-not-empty"
    meta = store.write(key, payload)
    assert meta.storage_key == key
    assert meta.kind == "initial_rgb"
    assert meta.byte_size == len(payload)
    assert meta.sha256 == sha256_hex_bytes(payload)
    assert meta.media_type == "image/png"
    with store.open(key) as handle:
        assert handle.read() == payload
    verified = store.verify(key, expected_sha256=meta.sha256)
    assert verified.sha256 == meta.sha256
    assert store.stat(key).byte_size == len(payload)


def test_rejects_traversal_and_absolute_keys(store: FilesystemArtifactStore) -> None:
    experiment = new_id()
    trial = new_id()
    with pytest.raises(ArtifactKeyError, match="absolute"):
        store.write(f"/{experiment}/{trial}/initial_rgb.png", b"data-bytes")
    with pytest.raises(ArtifactKeyError, match="traversal"):
        store.write(f"{experiment}/../{trial}/initial_rgb.png", b"data-bytes")
    with pytest.raises(ArtifactKeyError, match="relative"):
        store.write(f"{experiment}/./{trial}/initial_rgb.png", b"data-bytes")


def test_rejects_non_uuidv7_ids() -> None:
    with pytest.raises(ArtifactKeyError, match="UUIDv7"):
        artifact_storage_key(
            "00000000-0000-4000-8000-000000000001",
            new_id(),
            "initial_rgb",
        )


def test_duplicate_standard_kind_rejected(store: FilesystemArtifactStore) -> None:
    key = _key("terminal_rgb")
    store.write(key, b"first-payload-bytes")
    with pytest.raises(ArtifactConflictError, match="already exists"):
        store.write(key, b"second-payload-bytes")


def test_excessive_size_rejected(tmp_path: Path) -> None:
    store = FilesystemArtifactStore(
        tmp_path / "artifacts",
        max_bytes_by_kind={
            "initial_rgb": 8,
            **{
                kind: 1024
                for kind in (
                    "pre_grasp_rgb",
                    "post_grasp_rgb",
                    "pre_release_rgb",
                    "terminal_rgb",
                    "trajectory",
                    "trial_manifest",
                )
            },
        },
    )
    with pytest.raises(ArtifactSizeError, match="maximum size"):
        store.write(_key(), b"0123456789")


def test_interruption_leaves_no_healthy_artifact(store: FilesystemArtifactStore) -> None:
    key = _key("pre_grasp_rgb")
    destination = store.root / key
    destination.parent.mkdir(parents=True, exist_ok=True)
    temp = destination.parent / f".tmp-{destination.name}.deadbeef"
    temp.write_bytes(b"partial-write-bytes")
    assert not destination.exists()
    with pytest.raises(ArtifactNotFoundError):
        store.verify(key)
    assert key not in store.list_storage_keys()
    assert temp.exists()


def test_corruption_detected_by_verify_and_reconcile(store: FilesystemArtifactStore) -> None:
    key = _key("post_grasp_rgb")
    meta = store.write(key, b"healthy-payload-bytes")
    path = store.root / key
    path.write_bytes(b"tampered-payload-bytes")
    with pytest.raises(ArtifactIntegrityError, match="checksum"):
        store.verify(key, expected_sha256=meta.sha256)
    report = reconcile_artifacts(store, [meta])
    assert report.corrupt_count == 1
    assert report.missing_count == 0
    assert report.issues[0].kind is ReconciliationIssueKind.CORRUPT


def test_missing_file_reconcile(store: FilesystemArtifactStore) -> None:
    key = _key("pre_release_rgb")
    meta = store.write(key, b"present-then-removed")
    (store.root / key).unlink()
    report = reconcile_artifacts(store, [meta])
    assert report.missing_count == 1
    assert report.issues[0].kind is ReconciliationIssueKind.MISSING


def test_orphaned_file_reconcile_and_quarantine(store: FilesystemArtifactStore) -> None:
    key = _key("trajectory")
    meta = store.write(key, b"orphan-payload-bytes-xx")
    report = reconcile_artifacts(store, expected=())
    assert report.orphaned_count == 1
    assert report.issues[0].storage_key == key
    quarantine_key = store.quarantine(key, reason="orphaned during reconcile")
    assert quarantine_key.startswith("_quarantine/")
    assert key not in store.list_storage_keys()
    with pytest.raises(ArtifactNotFoundError):
        store.open(key)
    assert (store.root / quarantine_key).exists()
    assert meta.sha256 == sha256_hex_bytes(b"orphan-payload-bytes-xx")


def test_symlink_artifact_rejected(store: FilesystemArtifactStore) -> None:
    key = _key("terminal_rgb")
    target = store.root / key
    target.parent.mkdir(parents=True, exist_ok=True)
    real = store.root / "outside-blob.bin"
    real.write_bytes(b"linked-bytes")
    os.symlink(real, target)
    with pytest.raises(ArtifactKeyError, match="symbolic"):
        store.open(key)
    with pytest.raises(ArtifactKeyError, match="symbolic"):
        store.write(key, b"new-bytes-here")


def test_atomic_write_uses_temp_sibling_then_rename(
    store: FilesystemArtifactStore, monkeypatch: pytest.MonkeyPatch
) -> None:
    key = _key("trial_manifest")
    seen: dict[str, object] = {}
    real_replace = os.replace

    def tracking_replace(src: str | os.PathLike[str], dst: str | os.PathLike[str]) -> None:
        src_path = Path(src)
        dst_path = Path(dst)
        seen["temp_name"] = src_path.name
        seen["temp_exists_before"] = src_path.exists()
        seen["dest_exists_before"] = dst_path.exists()
        real_replace(src, dst)
        seen["temp_exists_after"] = src_path.exists()
        seen["dest_exists_after"] = dst_path.exists()

    monkeypatch.setattr(os, "replace", tracking_replace)
    store.write(key, b'{"ok":true}')
    assert str(seen["temp_name"]).startswith(".tmp-")
    assert seen["temp_exists_before"] is True
    assert seen["dest_exists_before"] is False
    assert seen["temp_exists_after"] is False
    assert seen["dest_exists_after"] is True
