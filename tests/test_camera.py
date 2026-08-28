from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from robot_control_platform_simulator.domain.models import Vector3
from robot_control_platform_simulator.physics.camera import (
    CAMERA_FAR_PLANE_METERS,
    CAMERA_FIELD_OF_VIEW_DEGREES,
    CAMERA_FRAME,
    CAMERA_HEIGHT_PX,
    CAMERA_IMAGE_MODE,
    CAMERA_MEDIA_TYPE,
    CAMERA_NEAR_PLANE_METERS,
    CAMERA_RENDERER,
    CAMERA_WIDTH_PX,
    CameraConfig,
    capture_rgb_frame,
    decode_rgb_png,
    default_camera_config,
    encode_rgb_png,
    look_at_view_matrix,
    rgba_to_uint8_rgb,
    write_rgb_png,
)
from robot_control_platform_simulator.physics.client import (
    PhysicsClient,
    SimulationError,
    engine_projection_matrix_fov,
    engine_view_matrix,
)
from robot_control_platform_simulator.physics.scene import WorkcellScene

REPO_ROOT = Path(__file__).resolve().parents[1]


def _nonzero_rgb() -> np.ndarray:
    rgb = np.zeros((CAMERA_HEIGHT_PX, CAMERA_WIDTH_PX, 3), dtype=np.uint8)
    rgb[40:80, 50:90] = (12, 140, 220)
    rgb[200:240, 300:360] = (200, 30, 40)
    return rgb


def test_default_camera_config_is_fixed_and_checksum_stable() -> None:
    camera = default_camera_config()
    again = default_camera_config()
    assert camera.width_px == CAMERA_WIDTH_PX == 640
    assert camera.height_px == CAMERA_HEIGHT_PX == 480
    assert camera.aspect_ratio == 640 / 480
    assert camera.field_of_view_degrees == CAMERA_FIELD_OF_VIEW_DEGREES
    assert camera.near_plane_meters == CAMERA_NEAR_PLANE_METERS
    assert camera.far_plane_meters == CAMERA_FAR_PLANE_METERS
    assert camera.renderer == CAMERA_RENDERER == "tiny"
    assert camera.frame == CAMERA_FRAME == "world"
    assert len(camera.view_matrix) == 16
    assert len(camera.projection_matrix) == 16
    assert camera.sha256_hex() == again.sha256_hex()
    assert camera.canonical_json() == again.canonical_json()
    payload = camera.to_checksum_payload()
    assert payload["renderer"] == "tiny"
    assert payload["view_matrix"] == list(camera.view_matrix)
    assert payload["projection_matrix"] == list(camera.projection_matrix)


def test_camera_checksum_changes_when_eye_moves() -> None:
    camera = default_camera_config()
    eye = Vector3(x=camera.eye_position_meters.x + 0.05, y=-1.15, z=1.65)
    moved = CameraConfig(
        eye_position_meters=eye,
        target_position_meters=camera.target_position_meters,
        up_vector=camera.up_vector,
        field_of_view_degrees=camera.field_of_view_degrees,
        near_plane_meters=camera.near_plane_meters,
        far_plane_meters=camera.far_plane_meters,
        width_px=camera.width_px,
        height_px=camera.height_px,
        renderer=camera.renderer,
        frame=camera.frame,
        view_matrix=look_at_view_matrix(eye, camera.target_position_meters, camera.up_vector),
        projection_matrix=camera.projection_matrix,
    )
    assert moved.sha256_hex() != camera.sha256_hex()


def test_invalid_camera_config_is_rejected() -> None:
    camera = default_camera_config()
    with pytest.raises(ValueError, match="field_of_view_degrees must be in \\(0, 180\\)"):
        CameraConfig(
            eye_position_meters=camera.eye_position_meters,
            target_position_meters=camera.target_position_meters,
            up_vector=camera.up_vector,
            field_of_view_degrees=0.0,
            near_plane_meters=camera.near_plane_meters,
            far_plane_meters=camera.far_plane_meters,
            width_px=camera.width_px,
            height_px=camera.height_px,
            renderer=camera.renderer,
            frame=camera.frame,
            view_matrix=camera.view_matrix,
            projection_matrix=camera.projection_matrix,
        )
    with pytest.raises(ValueError, match="far_plane_meters must be greater than near_plane_meters"):
        CameraConfig(
            eye_position_meters=camera.eye_position_meters,
            target_position_meters=camera.target_position_meters,
            up_vector=camera.up_vector,
            field_of_view_degrees=camera.field_of_view_degrees,
            near_plane_meters=5.0,
            far_plane_meters=5.0,
            width_px=camera.width_px,
            height_px=camera.height_px,
            renderer=camera.renderer,
            frame=camera.frame,
            view_matrix=camera.view_matrix,
            projection_matrix=camera.projection_matrix,
        )
    with pytest.raises(ValueError, match="width_px must be 640"):
        CameraConfig(
            eye_position_meters=camera.eye_position_meters,
            target_position_meters=camera.target_position_meters,
            up_vector=camera.up_vector,
            field_of_view_degrees=camera.field_of_view_degrees,
            near_plane_meters=camera.near_plane_meters,
            far_plane_meters=camera.far_plane_meters,
            width_px=320,
            height_px=camera.height_px,
            renderer=camera.renderer,
            frame=camera.frame,
            view_matrix=camera.view_matrix,
            projection_matrix=camera.projection_matrix,
        )
    with pytest.raises(ValueError, match="renderer must be tiny"):
        CameraConfig(
            eye_position_meters=camera.eye_position_meters,
            target_position_meters=camera.target_position_meters,
            up_vector=camera.up_vector,
            field_of_view_degrees=camera.field_of_view_degrees,
            near_plane_meters=camera.near_plane_meters,
            far_plane_meters=camera.far_plane_meters,
            width_px=camera.width_px,
            height_px=camera.height_px,
            renderer="opengl",
            frame=camera.frame,
            view_matrix=camera.view_matrix,
            projection_matrix=camera.projection_matrix,
        )
    with pytest.raises(ValueError, match="up_vector must be a unit vector"):
        CameraConfig(
            eye_position_meters=camera.eye_position_meters,
            target_position_meters=camera.target_position_meters,
            up_vector=Vector3(x=0.0, y=0.0, z=2.0),
            field_of_view_degrees=camera.field_of_view_degrees,
            near_plane_meters=camera.near_plane_meters,
            far_plane_meters=camera.far_plane_meters,
            width_px=camera.width_px,
            height_px=camera.height_px,
            renderer=camera.renderer,
            frame=camera.frame,
            view_matrix=camera.view_matrix,
            projection_matrix=camera.projection_matrix,
        )
    with pytest.raises(ValueError, match="camera forward must be a non-zero vector"):
        look_at_view_matrix(
            camera.eye_position_meters, camera.eye_position_meters, camera.up_vector
        )


def test_png_round_trip_has_required_size_and_mode() -> None:
    rgb = _nonzero_rgb()
    png = encode_rgb_png(rgb)
    image = decode_rgb_png(png)
    assert image.size == (CAMERA_WIDTH_PX, CAMERA_HEIGHT_PX)
    assert image.mode == CAMERA_IMAGE_MODE
    decoded = np.asarray(image)
    assert decoded.dtype == np.uint8
    assert decoded.shape == (CAMERA_HEIGHT_PX, CAMERA_WIDTH_PX, 3)
    assert int(np.count_nonzero(np.any(decoded > 0, axis=2))) > 0


def test_rgba_to_uint8_rgb_drops_alpha() -> None:
    rgba = np.zeros((CAMERA_HEIGHT_PX, CAMERA_WIDTH_PX, 4), dtype=np.uint8)
    rgba[0, 0] = (10, 20, 30, 255)
    rgb = rgba_to_uint8_rgb(rgba, height_px=CAMERA_HEIGHT_PX, width_px=CAMERA_WIDTH_PX)
    assert rgb.shape == (CAMERA_HEIGHT_PX, CAMERA_WIDTH_PX, 3)
    assert rgb.dtype == np.uint8
    assert tuple(rgb[0, 0]) == (10, 20, 30)


def test_write_rgb_png_rejects_repository_and_relative_paths() -> None:
    png = encode_rgb_png(_nonzero_rgb())
    inside = REPO_ROOT / "review_rgb.png"
    with pytest.raises(SimulationError, match="outside the repository"):
        write_rgb_png(inside, png)
    assert not inside.exists()
    with pytest.raises(SimulationError, match="must be absolute"):
        write_rgb_png(Path("review_rgb.png"), png)


def test_write_rgb_png_to_temporary_path(tmp_path: Path) -> None:
    png = encode_rgb_png(_nonzero_rgb())
    output = tmp_path / "review_rgb.png"
    if any((parent / ".git").exists() for parent in (output, *output.parents)):
        pytest.skip("pytest tmp_path is inside a git work tree")
    write_rgb_png(output, png)
    image = decode_rgb_png(output.read_bytes())
    assert image.size == (CAMERA_WIDTH_PX, CAMERA_HEIGHT_PX)
    assert image.mode == CAMERA_IMAGE_MODE


def test_review_camera_capture_decodes_to_nonblank_rgb() -> None:
    pytest.importorskip("pybullet")
    with PhysicsClient(gui=False) as client:
        WorkcellScene(client).reset()
        capture = capture_rgb_frame(client)
    assert capture.width_px == 640
    assert capture.height_px == 480
    assert capture.mode == CAMERA_IMAGE_MODE
    assert capture.media_type == CAMERA_MEDIA_TYPE
    assert capture.renderer == CAMERA_RENDERER
    assert capture.camera_checksum == default_camera_config().sha256_hex()
    assert capture.rgba_captured
    assert capture.depth_captured
    assert capture.segmentation_captured
    assert capture.depth_shape == (480, 640)
    assert capture.segmentation_shape == (480, 640)
    assert capture.nonblank_pixel_count > 0
    image = decode_rgb_png(capture.png_bytes)
    assert image.size == (640, 480)
    assert image.mode == "RGB"
    pixels = np.asarray(image)
    assert pixels.shape == (480, 640, 3)
    assert pixels.dtype == np.uint8
    assert int(np.count_nonzero(np.any(pixels > 0, axis=2))) == capture.nonblank_pixel_count


def test_camera_matrices_match_pybullet_engine() -> None:
    pytest.importorskip("pybullet")
    camera = default_camera_config()
    view = engine_view_matrix(
        camera.eye_position_meters, camera.target_position_meters, camera.up_vector
    )
    projection = engine_projection_matrix_fov(
        camera.field_of_view_degrees,
        camera.aspect_ratio,
        camera.near_plane_meters,
        camera.far_plane_meters,
    )
    np.testing.assert_allclose(camera.view_matrix, view, rtol=1e-5, atol=1e-5)
    np.testing.assert_allclose(camera.projection_matrix, projection, rtol=1e-5, atol=1e-5)
