"""Camera capture for streaming and AprilTag detection."""

import os
from typing import List, Optional, Tuple

import cv2
import numpy as np


def list_video_devices() -> List[str]:
    """Return available /dev/video* devices."""
    return [f"/dev/video{i}" for i in range(10) if os.path.exists(f"/dev/video{i}")]


def find_working_device(prefer: str = None) -> Optional[str]:
    """Find device that opens at 1920x1200. Prefer /dev/video1 (Arducam B0495 capture node)."""
    devices = list_video_devices()
    if prefer and prefer in devices:
        devices = [prefer] + [d for d in devices if d != prefer]
    elif "/dev/video1" in devices:
        devices = ["/dev/video1"] + [d for d in devices if d != "/dev/video1"]
    for dev in devices:
        cap = SimpleCamera(device=dev)
        try:
            if cap.open():
                ok, _ = cap.read()
                w, h = cap._width, cap._height
                cap.close()
                if ok and w == CAMERA_WIDTH and h == CAMERA_HEIGHT:
                    return dev
        except Exception:
            pass
    for dev in devices:
        cap = SimpleCamera(device=dev)
        try:
            if cap.open():
                ok, _ = cap.read()
                cap.close()
                if ok:
                    return dev
        except Exception:
            pass
    return None


CAMERA_WIDTH = 1920   # 1200p (Arducam B0495 native)
CAMERA_HEIGHT = 1200
CAMERA_FPS = 50


def _gstreamer_pipeline_bgr(device: str, width: int, height: int, fps: int) -> str:
    """GStreamer pipeline: YUY2 -> BGR."""
    return (
        f"v4l2src device={device} ! "
        f"video/x-raw,format=YUY2,width={width},height={height},framerate={fps}/1 ! "
        "videoconvert ! video/x-raw,format=BGR ! appsink"
    )


def _gstreamer_pipeline_y(device: str, width: int, height: int, fps: int) -> str:
    """GStreamer pipeline: YUY2 -> GRAY8 (Y channel only, for AprilTag)."""
    return (
        f"v4l2src device={device} ! "
        f"video/x-raw,format=YUY2,width={width},height={height},framerate={fps}/1 ! "
        "videoconvert ! video/x-raw,format=GRAY8 ! appsink"
    )


class SimpleCamera:
    """Minimal camera capture. Opens with default backend, accepts any format."""

    def __init__(self, device: str = "/dev/video0", width: int = None, height: int = None, fps: int = None):
        self.device = device
        self._cap: Optional[cv2.VideoCapture] = None
        self._width = width or CAMERA_WIDTH
        self._height = height or CAMERA_HEIGHT
        self._fps = fps or CAMERA_FPS

    def _device_index(self) -> int:
        if isinstance(self.device, str) and "/dev/video" in self.device:
            try:
                return int(self.device.replace("/dev/video", "").strip())
            except ValueError:
                pass
        return 0

    def open(self) -> bool:
        """Open Arducam at 1920x1200 @ 50fps, Y channel only (GRAY8). Tries GStreamer then OpenCV."""
        self.close()
        w, h, fps = self._width, self._height, self._fps
        configs = [(w, h, fps), (960, 600, 50), (640, 480, 30)]
        for cw, ch, cfps in configs:
            for pipe_fn in (_gstreamer_pipeline_y,):  # Y channel only, no color
                pipeline = pipe_fn(self.device, cw, ch, cfps)
                cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
                if cap and cap.isOpened():
                    for _ in range(3):
                        if cap.read()[0]:
                            self._cap = cap
                            self._width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or cw
                            self._height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or ch
                            return True
                if cap:
                    cap.release()
        idx = self._device_index()
        for cw, ch, cfps in configs:
            for src in [idx, self.device]:
                cap = cv2.VideoCapture(src)
                if cap and cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cw)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ch)
                    cap.set(cv2.CAP_PROP_FPS, cfps)
                    for _ in range(3):
                        if cap.read()[0]:
                            self._cap = cap
                            self._width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or cw
                            self._height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or ch
                            return True
                if cap:
                    cap.release()
        return False

    def close(self) -> None:
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

    def is_open(self) -> bool:
        return self._cap is not None and self._cap.isOpened()

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Read frame as BGR. Returns (ok, frame)."""
        ok, frame = self._read_raw()
        if not ok or frame is None:
            return False, None
        if len(frame.shape) == 2:
            return True, cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if frame.shape[2] == 2:
            gray = cv2.cvtColor(frame, cv2.COLOR_YUV2GRAY_YUY2)
            return True, cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        return True, frame

    def _read_raw(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Read raw frame from device (GRAY8 or BGR)."""
        if not self._cap:
            return False, None
        return self._cap.read()

    def read_y(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Read Y channel only (grayscale) at 1200p for AprilTag. Returns (ok, gray)."""
        ok, frame = self._read_raw()
        if not ok or frame is None:
            return False, None
        if len(frame.shape) == 2:
            return True, frame
        if frame.shape[2] == 2:
            return True, cv2.cvtColor(frame, cv2.COLOR_YUV2GRAY_YUY2)
        return True, cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def read_gray(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Alias for read_y (grayscale for AprilTag)."""
        return self.read_y()
