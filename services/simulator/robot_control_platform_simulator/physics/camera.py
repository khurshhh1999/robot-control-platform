"""Fixed review camera: TinyRenderer, 640x480 RGB PNG, world-frame eye/target/up."""

from __future__ import annotations

import io
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Final

import numpy as np
import numpy.typing as npt
from PIL import Image

from robot_control_platform_simulator.domain.models import (
    DOMAIN_SCHEMA_VERSION,
    JSONValue,
    Vector3,
    canonical_dumps,
    require_finite,
    require_positive,
    require_unit_vector,
    sha256_hex,
)
from robot_control_platform_simulator.physics.client import (
    WORLD_FRAME,
    PhysicsClient,
    SimulationError,
)

CAMERA_SCHEMA_VERSION: Final[str] = DOMAIN_SCHEMA_VERSION
CAMERA_WIDTH_PX: Final[int] = 640
CAMERA_HEIGHT_PX: Final[int] = 480
CAMERA_FIELD_OF_VIEW_DEGREES: Final[float] = 60.0
CAMERA_NEAR_PLANE_METERS: Final[float] = 0.10
CAMERA_FAR_PLANE_METERS: Final[float] = 10.0
CAMERA_RENDERER: Final[str] = "tiny"
CAMERA_IMAGE_MODE: Final[str] = "RGB"
CAMERA_MEDIA_TYPE: Final[str] = "image/png"
CAMERA_FRAME: Final[str] = WORLD_FRAME
MATRIX_LENGTH: Final[int] = 16

# World-frame meters. Elevated review camera covering robot, table, pickup, and bins.
CAMERA_EYE_POSITION_METERS: Final[Vector3] = Vector3(x=0.70, y=-1.15, z=1.65)
CAMERA_TARGET_POSITION_METERS: Final[Vector3] = Vector3(x=0.80, y=0.00, z=0.45)
CAMERA_UP_VECTOR: Final[Vector3] = Vector3(x=0.00, y=0.00, z=1.00)


def look_at_view_matrix(eye: Vector3, target: Vector3, up: Vector3) -> tuple[float, ...]:
    """OpenGL column-major look-at matrix matching PyBullet ``computeViewMatrix``."""

    forward = _normalize("camera forward", (target.x - eye.x, target.y - eye.y, target.z - eye.z))
    up_hat = _normalize("up_vector", (up.x, up.y, up.z))
    right = _normalize("camera right", _cross(forward, up_hat))
    true_up = _cross(right, forward)
    eye_tuple = (eye.x, eye.y, eye.z)
    return _finite_matrix16(
        (
            right[0],
            true_up[0],
            -forward[0],
            0.0,
            right[1],
            true_up[1],
            -forward[1],
            0.0,
            right[2],
            true_up[2],
            -forward[2],
            0.0,
            -_dot(right, eye_tuple),
            -_dot(true_up, eye_tuple),
            _dot(forward, eye_tuple),
            1.0,
        ),
        "view_matrix",
    )


def perspective_projection_matrix(
    field_of_view_degrees: float, aspect_ratio: float, near_meters: float, far_meters: float
) -> tuple[float, ...]:
    """OpenGL column-major FOV matrix matching PyBullet ``computeProjectionMatrixFOV``."""

    y_scale = 1.0 / math.tan(math.radians(field_of_view_degrees) / 2.0)
    x_scale = y_scale / aspect_ratio
    return _finite_matrix16(
        (
            x_scale,
            0.0,
            0.0,
            0.0,
            0.0,
            y_scale,
            0.0,
            0.0,
            0.0,
            0.0,
            (near_meters + far_meters) / (near_meters - far_meters),
            -1.0,
            0.0,
            0.0,
            (2.0 * far_meters * near_meters) / (near_meters - far_meters),
            0.0,
        ),
        "projection_matrix",
    )


@dataclass(frozen=True)
class CameraConfig:
    """Fixed review camera. Eye/target/up are meters in the world frame."""

    eye_position_meters: Vector3
    target_position_meters: Vector3
    up_vector: Vector3
    field_of_view_degrees: float
    near_plane_meters: float
    far_plane_meters: float
    width_px: int
    height_px: int
    renderer: str
    frame: str
    view_matrix: tuple[float, ...]
    projection_matrix: tuple[float, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.eye_position_meters, Vector3):
            raise ValueError("eye_position_meters must be a Vector3")
        if not isinstance(self.target_position_meters, Vector3):
            raise ValueError("target_position_meters must be a Vector3")
        if not isinstance(self.up_vector, Vector3):
            raise ValueError("up_vector must be a Vector3")
        require_unit_vector("up_vector", self.up_vector)
        object.__setattr__(
            self,
            "field_of_view_degrees",
            require_finite("field_of_view_degrees", self.field_of_view_degrees),
        )
        if not 0.0 < self.field_of_view_degrees < 180.0:
            raise ValueError("field_of_view_degrees must be in (0, 180)")
        object.__setattr__(
            self,
            "near_plane_meters",
            require_positive(
                "near_plane_meters", require_finite("near_plane_meters", self.near_plane_meters)
            ),
        )
        object.__setattr__(
            self,
            "far_plane_meters",
            require_positive(
                "far_plane_meters", require_finite("far_plane_meters", self.far_plane_meters)
            ),
        )
        if self.far_plane_meters <= self.near_plane_meters:
            raise ValueError("far_plane_meters must be greater than near_plane_meters")
        if not isinstance(self.width_px, int) or isinstance(self.width_px, bool):
            raise ValueError("width_px must be 640")
        if not isinstance(self.height_px, int) or isinstance(self.height_px, bool):
            raise ValueError("height_px must be 480")
        if self.width_px != CAMERA_WIDTH_PX:
            raise ValueError("width_px must be 640")
        if self.height_px != CAMERA_HEIGHT_PX:
            raise ValueError("height_px must be 480")
        if self.renderer != CAMERA_RENDERER:
            raise ValueError("renderer must be tiny")
        if self.frame != CAMERA_FRAME:
            raise ValueError("camera frame must be world")
        view = _finite_matrix16(self.view_matrix, "view_matrix")
        projection = _finite_matrix16(self.projection_matrix, "projection_matrix")
        expected_view = look_at_view_matrix(
            self.eye_position_meters, self.target_position_meters, self.up_vector
        )
        expected_projection = perspective_projection_matrix(
            self.field_of_view_degrees,
            self.aspect_ratio,
            self.near_plane_meters,
            self.far_plane_meters,
        )
        if view != expected_view:
            raise ValueError("view_matrix must match eye, target, and up")
        if projection != expected_projection:
            raise ValueError("projection_matrix must match FOV, aspect, near, and far")
        object.__setattr__(self, "view_matrix", view)
        object.__setattr__(self, "projection_matrix", projection)

    @property
    def aspect_ratio(self) -> float:
        return self.width_px / self.height_px

    def to_checksum_payload(self) -> dict[str, JSONValue]:
        return {
            "schema_version": CAMERA_SCHEMA_VERSION,
            "frame": self.frame,
            "renderer": self.renderer,
            "width_px": self.width_px,
            "height_px": self.height_px,
            "aspect_ratio": self.aspect_ratio,
            "field_of_view_degrees": self.field_of_view_degrees,
            "near_plane_meters": self.near_plane_meters,
            "far_plane_meters": self.far_plane_meters,
            "eye_position_meters": self.eye_position_meters.to_checksum_payload(),
            "target_position_meters": self.target_position_meters.to_checksum_payload(),
            "up_vector": self.up_vector.to_checksum_payload(),
            "view_matrix": list(self.view_matrix),
            "projection_matrix": list(self.projection_matrix),
        }

    def canonical_json(self) -> str:
        return canonical_dumps(self.to_checksum_payload())

    def sha256_hex(self) -> str:
        return sha256_hex(self.to_checksum_payload())


def default_camera_config() -> CameraConfig:
    view = look_at_view_matrix(
        CAMERA_EYE_POSITION_METERS, CAMERA_TARGET_POSITION_METERS, CAMERA_UP_VECTOR
    )
    projection = perspective_projection_matrix(
        CAMERA_FIELD_OF_VIEW_DEGREES,
        CAMERA_WIDTH_PX / CAMERA_HEIGHT_PX,
        CAMERA_NEAR_PLANE_METERS,
        CAMERA_FAR_PLANE_METERS,
    )
    return CameraConfig(
        eye_position_meters=CAMERA_EYE_POSITION_METERS,
        target_position_meters=CAMERA_TARGET_POSITION_METERS,
        up_vector=CAMERA_UP_VECTOR,
        field_of_view_degrees=CAMERA_FIELD_OF_VIEW_DEGREES,
        near_plane_meters=CAMERA_NEAR_PLANE_METERS,
        far_plane_meters=CAMERA_FAR_PLANE_METERS,
        width_px=CAMERA_WIDTH_PX,
        height_px=CAMERA_HEIGHT_PX,
        renderer=CAMERA_RENDERER,
        frame=CAMERA_FRAME,
        view_matrix=view,
        projection_matrix=projection,
    )


@dataclass(frozen=True)
class CameraCapture:
    """In-memory capture metadata. Only RGB PNG bytes are persisted in v1."""

    width_px: int
    height_px: int
    mode: str
    media_type: str
    png_bytes: bytes
    renderer: str
    camera_checksum: str
    view_matrix: tuple[float, ...]
    projection_matrix: tuple[float, ...]
    nonblank_pixel_count: int
    rgba_captured: bool
    depth_captured: bool
    segmentation_captured: bool
    depth_shape: tuple[int, int]
    segmentation_shape: tuple[int, int]


def rgba_to_uint8_rgb(buffer: object, *, height_px: int, width_px: int) -> npt.NDArray[np.uint8]:
    array = np.asarray(buffer)
    if array.ndim == 1:
        if array.size != height_px * width_px * 4:
            raise SimulationError("RGBA camera buffer has the wrong size")
        array = array.reshape((height_px, width_px, 4))
    elif array.ndim == 3:
        if array.shape[0] != height_px or array.shape[1] != width_px or array.shape[2] < 3:
            raise SimulationError("RGBA camera buffer has the wrong shape")
    else:
        raise SimulationError("RGBA camera buffer has the wrong rank")
    return np.ascontiguousarray(array[:, :, :3], dtype=np.uint8)


def encode_rgb_png(rgb: npt.NDArray[np.uint8]) -> bytes:
    if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise SimulationError("RGB frame must be uint8 with shape (height, width, 3)")
    image = Image.fromarray(rgb, mode=CAMERA_IMAGE_MODE)
    buffer = io.BytesIO()
    image.save(buffer, format="PNG", optimize=False)
    return buffer.getvalue()


def decode_rgb_png(png_bytes: bytes) -> Image.Image:
    if not png_bytes:
        raise SimulationError("RGB PNG is empty")
    image = Image.open(io.BytesIO(png_bytes))
    image.load()
    if image.mode != CAMERA_IMAGE_MODE:
        raise SimulationError("decoded PNG mode is not RGB")
    return image


def capture_rgb_frame(client: PhysicsClient, config: CameraConfig | None = None) -> CameraCapture:
    """Capture RGBA/depth/segmentation, then keep only uint8 RGB PNG bytes."""

    camera = config if config is not None else default_camera_config()
    rgba, depth, segmentation = client.get_camera_image(
        width_px=camera.width_px,
        height_px=camera.height_px,
        view_matrix=camera.view_matrix,
        projection_matrix=camera.projection_matrix,
        renderer=camera.renderer,
    )
    rgb = rgba_to_uint8_rgb(rgba, height_px=camera.height_px, width_px=camera.width_px)
    depth_array = _as_hw_array(
        depth, height_px=camera.height_px, width_px=camera.width_px, name="depth"
    )
    segmentation_array = _as_hw_array(
        segmentation,
        height_px=camera.height_px,
        width_px=camera.width_px,
        name="segmentation",
    )
    png_bytes = encode_rgb_png(rgb)
    decoded = decode_rgb_png(png_bytes)
    if decoded.size != (camera.width_px, camera.height_px):
        raise SimulationError("encoded PNG dimensions do not match the camera")
    nonblank = int(np.count_nonzero(np.any(rgb > 0, axis=2)))
    if nonblank <= 0:
        raise SimulationError("captured RGB frame is blank")
    return CameraCapture(
        width_px=camera.width_px,
        height_px=camera.height_px,
        mode=CAMERA_IMAGE_MODE,
        media_type=CAMERA_MEDIA_TYPE,
        png_bytes=png_bytes,
        renderer=camera.renderer,
        camera_checksum=camera.sha256_hex(),
        view_matrix=camera.view_matrix,
        projection_matrix=camera.projection_matrix,
        nonblank_pixel_count=nonblank,
        rgba_captured=True,
        depth_captured=True,
        segmentation_captured=True,
        depth_shape=(int(depth_array.shape[0]), int(depth_array.shape[1])),
        segmentation_shape=(
            int(segmentation_array.shape[0]),
            int(segmentation_array.shape[1]),
        ),
    )


def write_rgb_png(path: Path, png_bytes: bytes) -> None:
    """Write PNG bytes to an absolute path. Artifact-store atomicity is a later card."""

    if not path.is_absolute():
        raise SimulationError("camera PNG output path must be absolute")
    if path.suffix.lower() != ".png":
        raise SimulationError("camera PNG output path must use the .png suffix")
    if _is_inside_git_work_tree(path):
        raise SimulationError("camera PNG output must be outside the repository")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(png_bytes)


def _is_inside_git_work_tree(path: Path) -> bool:
    resolved = path.resolve()
    for parent in (resolved, *resolved.parents):
        if (parent / ".git").exists():
            return True
    return False


def _as_hw_array(
    buffer: object, *, height_px: int, width_px: int, name: str
) -> npt.NDArray[np.generic]:
    array = np.asarray(buffer)
    if array.size != height_px * width_px:
        raise SimulationError(f"{name} camera buffer has the wrong size")
    return array.reshape((height_px, width_px))


def _finite_matrix16(values: object, field: str) -> tuple[float, ...]:
    if not isinstance(values, (list, tuple)) or len(values) != MATRIX_LENGTH:
        raise ValueError(f"{field} must have length {MATRIX_LENGTH}")
    return tuple(
        require_finite(f"{field}[{index}]", values[index]) for index in range(MATRIX_LENGTH)
    )


def _dot(left: tuple[float, float, float], right: tuple[float, float, float]) -> float:
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]


def _cross(
    left: tuple[float, float, float], right: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _normalize(field: str, vector: tuple[float, float, float]) -> tuple[float, float, float]:
    norm = math.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)
    if not math.isfinite(norm) or norm == 0.0:
        raise ValueError(f"{field} must be a non-zero vector")
    return (vector[0] / norm, vector[1] / norm, vector[2] / norm)
