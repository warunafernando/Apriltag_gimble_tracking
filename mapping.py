"""Environment mapping: pan/IMU + AprilTag detections -> world azimuth per tag."""

import math
from dataclasses import dataclass, field
from typing import List, Dict
import time

# Camera: Arducam AR0234 (1/2.6" sensor 5.76×3.6mm) + M2504ZH05S 4mm lens.
# Calibrated horizontal FOV (pan sweep with full-tag visibility).
SENSOR_WIDTH_MM = 5.76
LENS_FOCAL_MM = 4.0
CAMERA_H_FOV_DEG = 66.0

# FRC standard AprilTag size: outer black border, 6 inches = 0.1524 m.
APRILTAG_SIZE_M = 0.1524
# Calibrate distance: set to (known_distance_ft / displayed_distance_ft) if reading is off (e.g. 8/7.2).
DISTANCE_SCALE = 1.0


def tag_size_pixels(corners) -> float:
    """Average edge length of tag quad in pixels. corners: 4x2 array."""
    import numpy as np
    c = np.asarray(corners, dtype=np.float64)
    if c.shape[0] < 4:
        return 0.0
    edges = [
        np.linalg.norm(c[1] - c[0]),
        np.linalg.norm(c[2] - c[1]),
        np.linalg.norm(c[3] - c[2]),
        np.linalg.norm(c[0] - c[3]),
    ]
    return float(np.mean(edges))


def tag_width_pixels(corners) -> float:
    """Horizontal span of tag quad in pixels (for distance via horizontal FOV). corners: 4x2 array."""
    import numpy as np
    c = np.asarray(corners, dtype=np.float64)
    if c.shape[0] < 4:
        return 0.0
    return float(np.max(c[:, 0]) - np.min(c[:, 0]))


def tag_distance(tag_size_px: float, img_width: float, tag_size_m: float = APRILTAG_SIZE_M,
                 h_fov_deg: float = CAMERA_H_FOV_DEG, scale: float = DISTANCE_SCALE) -> float:
    """
    Distance to tag from pinhole geometry:
      focal_px = (img_width/2) / tan(h_fov/2)
      distance_m = (tag_size_m * focal_px) / tag_size_px
    tag_size_px = horizontal span of tag in image (same axis as h_fov).
    """
    if tag_size_px <= 0 or img_width <= 0:
        return float("nan")
    h_fov_rad = math.radians(h_fov_deg)
    focal_length_px = (img_width / 2) / math.tan(h_fov_rad / 2)
    distance_m = (tag_size_m * focal_length_px) / tag_size_px
    return distance_m * scale


def pixel_offset_to_azimuth(tag_center_x: float, img_width: float, h_fov_deg: float = CAMERA_H_FOV_DEG) -> float:
    """Convert tag's horizontal pixel offset from image center to azimuth offset (degrees).
    Positive = tag to right of center. Returns degrees to add to camera azimuth."""
    if img_width <= 0:
        return 0.0
    offset_norm = (tag_center_x - img_width / 2) / img_width
    return offset_norm * h_fov_deg


def tag_azimuth(camera_azimuth: float, tag_center_x: float, img_width: float, h_fov_deg: float = CAMERA_H_FOV_DEG) -> float:
    """Compute tag's world azimuth from camera azimuth and tag position in image."""
    offset = pixel_offset_to_azimuth(tag_center_x, img_width, h_fov_deg)
    return (camera_azimuth + offset + 360) % 360


@dataclass
class TagAtAzimuth:
    """A tag seen at a given world azimuth (degrees 0-360)."""
    tag_id: int
    azimuth: float  # world heading in degrees
    pan: float
    tilt: float
    imu_yaw: float


@dataclass
class MapEntry:
    """One scan step: pan, tilt, IMU yaw, and tags detected."""
    pan: float
    tilt: float
    imu_yaw: float
    timestamp: float
    tags: List[dict]  # [{tag_id, center, corners}, ...]


def world_azimuth(imu_yaw: float, pan: float) -> float:
    """Camera pointing direction in world frame (degrees 0-360)."""
    a = (imu_yaw + pan + 360) % 360
    return a if a < 360 else 0.0


def entries_to_tag_azimuths(entries: List[MapEntry], img_width: float = 1920, img_height: float = 1200) -> Dict[int, float]:
    """
    From scan entries, build tag_id -> world azimuth (mean if seen multiple times).
    Uses tag center position in image to compute exact azimuth, not just camera center.
    """
    by_tag: Dict[int, List[float]] = {}
    for e in entries:
        cam_az = world_azimuth(e.imu_yaw, e.pan)
        for t in e.tags:
            tid = t.get("tag_id")
            if tid is None:
                continue
            center = t.get("center")
            if center and len(center) >= 1:
                tag_x = float(center[0])
                az = tag_azimuth(cam_az, tag_x, img_width)
            else:
                az = cam_az
            by_tag.setdefault(tid, []).append(az)
    out = {}
    for tid, azs in by_tag.items():
        # Assume raw az is compass (0=North). Average as vectors: compass 0 -> (0,1) so use (a+90).
        x = sum(math.cos(math.radians(a + 90)) for a in azs) / len(azs)
        y = sum(math.sin(math.radians(a + 90)) for a in azs) / len(azs)
        math_az = math.degrees(math.atan2(y, x))
        out[tid] = (math_az - 90 + 360) % 360  # math -> compass: 0 = North
    return out
