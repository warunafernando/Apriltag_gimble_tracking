# AprilTag + Gimbal Compass Compensation — Integration Plan

## Overview

**Standalone project** (`gimbal_apriltag_vision`) that combines:
1. **AprilTag detection** on the Arducam (1200p/50fps, Y channel only)
2. **Gimbal serial protocol** (IMU, pan/tilt) — protocol reference: `gimble_conrol`
3. **Environment mapping** — scan and record AprilTag positions as gimbal pans
4. **Rotation compensation** — when base rotates, drive pan motor to keep view stable
5. **Web GUI** — circular viz (tags, gimbal, compass) + live camera stream

---

## 1. System Components

| Component | Description |
|-----------|-------------|
| **This project** | Standalone Python app: capture, AprilTag, mapping, compensation, web GUI. |
| **Arducam** | USB camera, 1920×1200 @ 50fps, YUYV. Mounted on gimbal. |
| **Gimbal** | Waveshare 2-axis pan-tilt, ESP32, IMU (QMI8658 + AK09918), ST3215 servos. |
| **gimble_conrol** | Separate project; reference for serial protocol (GET_IMU=126, PAN_TILT_ABS=133, IMU=1002). |
| **SVTVision** | Optional reference for camera config / vision pipeline patterns. |

---

## 2. Arducam: 1200p/50fps with Y Channel Only

### 2.1 Rationale
- AprilTag detection requires **grayscale** input; Y (luma) is sufficient.
- Capture Y only to reduce bandwidth and avoid RGB conversion.
- 1920×1200 @ 50fps ≈ 115 MB/s raw RGB vs ~38 MB/s Y-only.

### 2.2 Capture Options

| Option | Description | Notes |
|--------|-------------|-------|
| **YUYV + extract Y** | OpenCV capture YUYV, extract Y plane. | Simple, works with V4L2. |
| **GREY / Y800** | If camera supports native grayscale. | Check `v4l2-ctl -d /dev/videoX --list-formats-ext`. |
| **GStreamer** | Pipeline with `videoconvert ! video/x-raw,format=GRAY8` | If CPU decode is bottleneck. |

### 2.3 Implementation Sketch
```python
cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y','U','Y','V'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
cap.set(cv2.CAP_PROP_FPS, 50)
ret, frame = cap.read()
gray = cv2.cvtColor(frame, cv2.COLOR_YUV2GRAY_YUY2)  # or frame[:, ::2]
```

---

## 3. AprilTag Detection Pipeline

### 3.1 Stack
- **Library**: `apriltag` (Python) or OpenCV ArUco `DICT_APRILTAG_36h11`.
- **Input**: Grayscale frame 1920×1200.
- **Target**: Real-time at 50fps; do not decimate.

### 3.2 Pipeline Stages
1. Capture — Arducam, Y channel only.
2. Optional decimation — 960×600 for faster detection.
3. Preprocess — Blur, adaptive threshold.
4. AprilTag detect — Output: tag IDs, corners, pose (if calibrated).
5. Overlay — Draw detections on stream (optional).

---

## 4. Environment Mapping (Phase 1)

### 4.1 Goal
Build a map: **pan angle → AprilTag positions**.

### 4.2 Flow
1. **Scan mode**: Gimbal pans from −180° to +180° in steps.
2. At each step: read IMU yaw, capture frame, detect tags.
3. Record: `{ pan_angle, tilt_angle, imu_yaw, tags: [...] }`.
4. Use for circular viz and compensation.

### 4.3 Data Structures
```python
@dataclass
class MapEntry:
    pan: float
    tilt: float
    imu_yaw: float
    timestamp: float
    tags: List[AprilTagDetection]
```

---

## 5. Gimbal Rotation Compensation (Phase 2)

### 5.1 Goal
When base rotates, IMU yaw changes. Compensate by moving pan motor:  
`pan_command = target_heading - imu_yaw` (with angle wrapping).

### 5.2 Control Loop
- Target from map or user
- Feedback: IMU yaw
- Output: PAN_TILT_ABS or PAN_ONLY_ABS
- Throttle pan commands to ~20–50 Hz.

---

## 6. Web GUI: Circular Visualization + Stream

### 6.1 Overview
**A. Circular (radar/polar) display:**
- AprilTag locations by world azimuth
- Gimbal position (camera pan)
- Compass heading (IMU yaw)

**B. Live camera stream** — Arducam video with optional AprilTag overlay.

### 6.2 Visual Elements

| Element | Representation |
|---------|----------------|
| AprilTag | Colored dot on circumference; tooltip with tag ID |
| Gimbal | Line from center; distinct color (e.g. blue) |
| Compass | Needle/arrow; distinct color; "N" at top |
| Camera stream | Live video panel (MJPEG or JPEG via SocketIO) |

### 6.3 Implementation
- **Circular viz**: Canvas or SVG; SocketIO events.
- **Stream**: MJPEG `/video_feed` or JPEG frames via SocketIO (5–15 fps).

---

## 7. Implementation Phases

### Phase 1: Y-Channel Capture + AprilTag (Weeks 1–2)
1. Arducam capture 1920×1200 @ 50fps, Y only.
2. AprilTag detection integration.
3. Validate detection rate (≥25 fps).

### Phase 2: Mapping + GUI (Weeks 2–3)
1. Scan/map mode; store `List[MapEntry]`.
2. Circular visualization (tags, gimbal, compass).
3. Live camera stream.

### Phase 3: Compensation Loop (Weeks 3–4)
1. Track/compensate mode.
2. IMU polling, pan command loop.
3. Tuning.

### Phase 4: Integration & Calibration (Week 4+)
1. Map → compensation linkage.
2. IMU/pan alignment calibration.

---

## 8. Project Structure

```
gimbal_apriltag_vision/
├── README.md
├── APRILTAG_GIMBAL_COMPENSATION_PLAN.md   # this doc
├── requirements.txt
├── app.py                  # Flask + SocketIO main app
├── protocol.py             # gimbal serial protocol (copy/adapt from gimble_conrol)
├── camera_capture.py       # Arducam Y-channel capture
├── apriltag_detect.py      # AprilTag wrapper
├── gimbal_compensate.py    # mapping + compensation logic
└── static/
    └── index.html          # circular viz + stream + controls
```

---

## 9. Dependencies

- `opencv-python` or `opencv-python-headless`
- `apriltag` or `opencv-contrib-python`
- `pyserial`
- `flask`
- `flask-socketio`

---

## 10. Calibration & Risks

- **Calibration**: Camera–gimbal axis, IMU–pan zero offset.
- **Risks**: 50 fps CPU load (decimate), serial contention, IMU drift — see plan for mitigations.
