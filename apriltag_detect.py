"""AprilTag detection wrapper."""

from dataclasses import dataclass
from typing import List

import cv2
import numpy as np

try:
    import apriltag
    HAS_APRILTAG = True
except ImportError:
    HAS_APRILTAG = False


@dataclass
class AprilTagDetection:
    """Single AprilTag detection."""
    tag_id: int
    center: tuple  # (x, y)
    corners: np.ndarray  # 4x2


class AprilTagDetector:
    """AprilTag detector using apriltag library."""

    def __init__(self, family: str = "tag36h11", quad_decimate: float = 2.0):
        if not HAS_APRILTAG:
            raise ImportError("Install apriltag: pip install apriltag")
        options = apriltag.DetectorOptions(
            families=family,
            border=1,
            nthreads=2,
            quad_decimate=quad_decimate,
            quad_blur=0.0,
            refine_edges=True,
            refine_decode=False,
            refine_pose=False,
            debug=False,
            quad_contours=True,
        )
        self.detector = apriltag.Detector(options)
        self.family = family
        self.quad_decimate = quad_decimate

    def detect(self, gray: np.ndarray, blur: int = 3, use_clahe: bool = False) -> List[AprilTagDetection]:
        """
        Detect AprilTags in grayscale frame.
        gray: HxW uint8
        blur: Gaussian blur kernel size (odd, 0=off). Reduces noise to avoid apriltag segfault.
        use_clahe: Apply CLAHE contrast enhancement (off by default for FPS).
        """
        if gray.dtype != np.uint8:
            gray = (np.clip(gray, 0, 1) * 255).astype(np.uint8)
        if len(gray.shape) == 3:
            gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
        if use_clahe:
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            gray = clahe.apply(gray)
        if blur > 0 and blur % 2 == 1:
            gray = cv2.GaussianBlur(gray, (blur, blur), 0)

        raw = self.detector.detect(gray)
        out = []
        for det in raw:
            if det.tag_id is None:
                continue
            margin = getattr(det, "decision_margin", 100)
            if margin < 22:
                continue
            corners = np.array(det.corners, dtype=np.float32)
            cx = float(np.mean(corners[:, 0]))
            cy = float(np.mean(corners[:, 1]))
            out.append(AprilTagDetection(
                tag_id=int(det.tag_id),
                center=(cx, cy),
                corners=corners,
            ))
        return out

    def draw_overlay(self, frame: np.ndarray, detections: List[AprilTagDetection]) -> np.ndarray:
        """Draw tag polygons, center dot, and label on stream frame."""
        if len(frame.shape) == 2:
            overlay = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        else:
            overlay = frame.copy()
        h, w = overlay.shape[:2]

        for d in detections:
            pts = np.ascontiguousarray(d.corners).astype(np.int32)
            if pts.size >= 4:
                cv2.polylines(overlay, [pts], True, (0, 255, 0), 3, cv2.LINE_AA)
            cx, cy = int(round(d.center[0])), int(round(d.center[1]))
            cv2.circle(overlay, (cx, cy), 6, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.circle(overlay, (cx, cy), 6, (255, 255, 255), 1, cv2.LINE_AA)
            label = f"Tag {d.tag_id}"
            tx = max(5, min(cx - 25, w - 55))
            ty = max(25, min(cy - 10, h - 10))
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
            cv2.rectangle(overlay, (tx - 2, ty - th - 2), (tx + tw + 2, ty + 2), (0, 0, 0), -1)
            cv2.putText(
                overlay, label,
                (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA
            )
        return overlay
