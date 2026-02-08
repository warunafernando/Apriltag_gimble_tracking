# Gimbal AprilTag Vision — Logic and Math

This document explains the core logic and mathematical formulas used in the gimbal AprilTag vision system.

---

## 1. System Overview

### Architecture

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│  Gimbal (ESP32) │────▶│  Flask + SocketIO │◀────│  Web Browser    │
│  IMU + Pan/Tilt │     │  app.py           │     │  index.html     │
└─────────────────┘     └────────┬─────────┘     └─────────────────┘
                                 │
                                 ▼
                        ┌──────────────────┐
                        │  Arducam Camera  │
                        │  1920×1200 @50fps│
                        └──────────────────┘
```

- **Gimbal**: Waveshare 2-axis pan-tilt, ESP32, IMU (QMI8658 + AK09918). Serial @ 921600 baud.
- **Camera**: Arducam, grayscale (Y channel only), mounted on gimbal.
- **Backend**: Python (Flask + SocketIO), handles serial, camera, AprilTag detection, mapping.
- **Frontend**: HTML/JS, circular radar viz, live stream, controls.

### Data Flow

1. IMU poll (20 Hz) → `yaw` (compass heading)
2. Camera loop (50 fps) → grayscale frame → AprilTag detect → `tags_live` (tags with azimuth, distance)
3. Scan mode: pan steps → IMU + detect → build map `tag_id → world azimuth`
4. Keep-in-view: drive pan to keep a chosen tag centered when robot rotates

---

## 2. Coordinate Systems and Conventions

### Compass / World Azimuth

- **0°** = North (forward, top of radar)
- **90°** = East (right)
- **180°** = South (back)
- **270°** = West (left)
- Range: **[0, 360)**

### Gimbal Angles

- **Pan**: -180° to +180° (relative to robot forward). Positive = right.
- **Tilt**: typically 42° (fixed for horizontal scanning).
- Pan 0° = camera center aligned with robot forward.

### Image Coordinates

- Origin (0,0) at top-left.
- X increases right, Y increases down.
- Center: `(img_width/2, img_height/2)`.

---

## 3. Core Math Formulas

### 3.1 Camera World Azimuth

**Where is the camera pointing in the world?**

```
world_azimuth = (imu_yaw + pan + 360) mod 360
```

- `imu_yaw`: IMU compass heading (0–360°)
- `pan`: gimbal pan angle (-180…+180°)
- Result: direction the camera center points in world frame.

**Source**: `mapping.py` → `world_azimuth(imu_yaw, pan)`.

---

### 3.2 Tag Azimuth from Pixel Position

**Tag center not at image center: add an offset.**

**Step 1 — Pixel offset to angle:**

```
offset_norm = (tag_center_x - img_width/2) / img_width
pixel_offset_deg = offset_norm × h_fov_deg
```

- `tag_center_x`: horizontal pixel of tag center.
- `offset_norm`: -0.5…+0.5 (left…right).
- `h_fov_deg`: horizontal FOV (default 66°).
- Positive = tag right of center.

**Step 2 — Tag world azimuth:**

```
tag_azimuth = (camera_azimuth + pixel_offset_deg + 360) mod 360
```

**Source**: `mapping.py` → `pixel_offset_to_azimuth()`, `tag_azimuth()`.

---

### 3.3 Distance from Tag Size (Pinhole)

```
focal_px = (img_width / 2) / tan(h_fov_deg / 2)
distance_m = (tag_size_m × focal_px) / tag_width_px
distance_m *= scale   // calibration
```

- **tag_size_m**: FRC AprilTag size = 0.1524 m (6 inches).
- **tag_width_px**: horizontal span of tag quad in pixels.
- **h_fov_deg**: horizontal FOV (default 66°).
- **scale**: `(known_distance_ft / displayed_distance_ft)` for calibration.

**Source**: `mapping.py` → `tag_distance()`.

---

### 3.4 Tag Width in Pixels

```
tag_width_px = max(corners[:, 0]) - min(corners[:, 0])
```

Horizontal span of the AprilTag quad in the image.

**Source**: `mapping.py` → `tag_width_pixels()`.

---

### 3.5 Averaging Azimuths (Vector Mean)

For multiple azimuth readings of the same tag:

```
x = Σ cos((a + 90)°) / n
y = Σ sin((a + 90)°) / n
math_az = atan2(y, x)
compass_az = (math_az - 90 + 360) mod 360
```

- Compass 0° → math (0, 1); 90° → (1, 0).
- Vector mean avoids wrap-around at 0°/360°.

**Source**: `mapping.py` → `entries_to_tag_azimuths()`.

---

## 4. Angle Wrapping and Blending

### 4.1 Shortest-Path Angle Difference

```
d = b - a
if d > 180:  d -= 360
if d < -180: d += 360
```

**Source**: `static/index.html` → `blendAngle()`.

### 4.2 Angle Blending (Smoothing)

```
result = a + d × t   // then normalize to [0, 360)
```

- `d`: shortest-path difference.
- `t`: blend factor (0–1).

Used for: gyro calibration, North tag display, pan smoothing.

---

### 4.3 Is Tag in Camera FOV?

```
diff = |(tag_azimuth - camera_azimuth + 540) mod 360 - 180|
in_fov = (diff <= fov_deg / 2)
```

**Source**: `static/index.html` → `angleInFov()`.

---

## 5. Environment Mapping (Scan)

### Logic

1. Pan gimbal from `pan_start` to `pan_end` in `step_deg` increments.
2. At each step:
   - Send pan command.
   - Wait ~0.2 s.
   - Read IMU yaw, capture frame, run AprilTag detection.
   - For each tag: compute `tag_azimuth(camera_az, center_x, img_width)`.
   - Store `MapEntry(pan, tilt, imu_yaw, timestamp, tags)`.
3. Aggregate: `entries_to_tag_azimuths()` → `tag_id → mean azimuth` (vector mean).

**Source**: `app.py` → `scan_map()`; `mapping.py` → `entries_to_tag_azimuths()`.

---

## 6. Keep-in-View Tracking

### Goal

Keep a chosen tag (e.g. Tag 3) centered when the robot rotates. Gimbal pan compensates for IMU yaw change.

### Desired Pan

```
desired_pan_raw = (tag_azimuth - compass_deg - TRACK_PAN_ZERO_OFFSET + 540) mod 360 - 180
desired_pan = clamp(desired_pan_raw, -180, 180)
desired_pan = TRACK_PAN_SIGN × desired_pan
```

- `tag_azimuth`: world azimuth of the tag (live or last-known).
- `compass_deg`: IMU heading (0–360°).
- `TRACK_PAN_ZERO_OFFSET`: calibration (typically 0).
- `TRACK_PAN_SIGN`: 1 or -1 depending on gimbal direction.

### Pan Smoothing

```
angleDelta = desired_pan - gimbalPan   // wrap to [-180, 180]
absDelta = |angleDelta|
blend = minBlend + (maxBlend - minBlend) × min(absDelta/25, 1)
smoothedPan = gimbalPan + blend × angleDelta
smoothedPan = clamp(smoothedPan, -180, 180)
```

- `minBlend = 0.06`, `maxBlend = 0.42`.
- Larger error → faster correction.

### Scan Mode

If tag not visible: pan in steps (-55° to +55°, 10° steps) until tag appears, then switch to tracking.

**Source**: `static/index.html` → `runTrackTag()`, `startTracking()`.

---

## 7. Gyro Calibration (North = Tag Direction)

When Tag 3 (North marker) is visible:

```
gyroCalOffset = blendAngle(gyroCalOffset, gimbalPan - tagAzimuth, 0.12)
displayGyro = (compassDeg + gyroCalOffset + 360) mod 360
```

- `gyroCalOffset` aligns IMU so that 0° points at the tag.
- Blending factor 0.12 for stability.
- `gyroCalOffset` is kept when tag is lost (no jump).

**Source**: `static/index.html` → `tags_live` handler, `updateApriltagThreeValues()`.

---

## 8. FOV Calibration

### Method

1. Pan slowly from -50° to +50°, step 0.5°, pause ~1.2 s per step.
2. Record `(pan, camera_az, tag_id, center_x, width_px)` when tag is fully visible.
3. Per tag: FOV = span of pan angles from first to last detection:
   ```
   fov_deg = |pan_last - pan_first|
   if fov_deg > 180: fov_deg = 360 - fov_deg
   ```

**Source**: `app.py` → `_run_fov_calibration()`.

---

## 9. Radar Visualization

### Display Conventions

- **N** at top; **E** right; **S** bottom; **W** left.
- North marker (e.g. Tag 3) defines "North" on display when available.

### Azimuth to Display

```
display_az = (raw_az + north_offset) mod 360
north_offset = (360 - northTagAzimuthForDisplay) mod 360   // when tag used as North
```

### Polar to Cartesian (Canvas)

```
rad = (deg - 90) × π/180   // compass 0° = top
x = cx + r × cos(rad)
y = cy + r × sin(rad)
```

### Elements

| Element         | Color  | Meaning                              |
|----------------|--------|--------------------------------------|
| Robot forward  | Red    | Compass / IMU heading                |
| Camera center  | Blue   | Middle of FOV                        |
| FOV wedge      | Blue   | Camera horizontal FOV                |
| Tag in view    | Green  | Tag currently detected               |
| Tag out of view| Yellow | Tag known but not in current frame   |
| Tag box size   |        | Proportional to `width_px` (distance)|

**Source**: `static/index.html` → `drawRadar()`.

---

## 10. Serial Protocol (Gimbal)

### Frame Format

```
STX (0x02) | length | seq | type_id | payload | CRC8 | ETX (0x03)
```

- **CRC8**: polynomial 0x07, over body (length, seq, type_id, payload).

### Commands / Responses

| Type   | ID   | Description      |
|--------|------|------------------|
| CMD    | 126  | GET_IMU          |
| CMD    | 133  | PAN_TILT_ABS     |
| CMD    | 610  | GET_FW_INFO      |
| RSP    | 1002 | IMU (yaw, pitch, roll, accel, gyro, mag) |
| RSP    | 2610 | FW_INFO          |

### IMU Payload (50+ bytes)

- `yaw`, `pitch`, `roll`: float32 @ 0, 4, 8
- `ax`, `ay`, `az`: float32 @ 12, 16, 20
- `gx`, `gy`, `gz`: float32 @ 24, 28, 32
- `mx`, `my`, `mz`: int16 @ 36, 38, 40

### PAN_TILT_ABS Payload

```
pan: float32, tilt: float32, speed: uint16, accel: uint16
```

**Source**: `protocol.py`.

---

## 11. AprilTag Detection Pipeline

### Input

- Grayscale frame (Y channel, 1920×1200 or scaled to 960×600).
- Optional: Gaussian blur (3×3) to reduce noise.
- Optional: CLAHE (off by default for speed).

### Detection

- Family: `tag36h11`.
- `quad_decimate = 2.0` (faster, lower res).
- Filter: `decision_margin >= 22`.

### Output per Tag

- `tag_id`, `center (x,y)`, `corners` (4×2).
- Backend adds: `azimuth`, `width_px`, `distance_m`, `distance_ft`.

**Source**: `apriltag_detect.py`; `app.py` → `_detection_to_dict()`, `_camera_loop()`.

---

## 12. Camera Pipeline

### Capture

- Device: `/dev/video0` or `/dev/video1` (Arducam).
- Format: YUYV → GRAY8 (Y only).
- Resolution: 1920×1200 @ 50 fps preferred; fallbacks 960×600, 640×480.

### Processing

1. Read grayscale frame.
2. Optionally resize to max 960 px for detection.
3. Run AprilTag detector.
4. For each detection: compute azimuth, distance, draw overlay.
5. Encode JPEG, store for `/api/frame` and MJPEG stream.

**Source**: `camera_capture.py`; `app.py` → `_camera_loop()`.

---

## 13. Constants Reference

| Constant           | Value   | Description                    |
|--------------------|---------|--------------------------------|
| APRILTAG_SIZE_M    | 0.1524  | FRC tag size (6 in)            |
| CAMERA_H_FOV_DEG   | 66.0    | Default horizontal FOV         |
| SENSOR_WIDTH_MM    | 5.76    | Arducam sensor width           |
| LENS_FOCAL_MM      | 4.0     | Lens focal length              |
| NORTH_MARKER_TAG_ID| 3       | Tag used for North / tracking  |
| STREAM_FPS         | 50      | Target camera loop rate        |

---

## 14. File Reference

| File               | Purpose                                      |
|--------------------|----------------------------------------------|
| `app.py`           | Flask app, serial, camera, mapping, routes   |
| `mapping.py`       | Azimuth, distance, mapping math              |
| `apriltag_detect.py` | AprilTag detection wrapper                 |
| `camera_capture.py`  | Arducam capture (Y channel)                |
| `protocol.py`      | Gimbal serial protocol                       |
| `static/index.html`| Web UI, radar, tracking, SocketIO client     |
