# Gimbal AprilTag Vision

Standalone project that combines **AprilTag detection** on an Arducam with **gimbal IMU** and **pan motor control** for environment mapping and rotation compensation.

## Features
- AprilTag detection at 1200p/50fps (Y channel only)
- Environment mapping by scanning with gimbal pan
- Gimbal rotation compensation (keep camera view stable when base rotates)
- Web GUI: circular visualization (tags, gimbal, compass) + live camera stream

## Dependencies
- **Gimbal**: Waveshare 2-axis pan-tilt (ESP32) — connected via serial
- **Arducam**: USB camera, 1920×1200 @ 50fps
- **gimble_conrol**: Reference for serial protocol (GET_IMU, PAN_TILT_ABS)

## Quick Start
```bash
pip install -r requirements.txt
# Phase 1 benchmark (optional)
python run_phase1.py --test
python run_phase1.py --device /dev/video0 --frames 100

# Phase 2: Web GUI (serial + camera + map + stream)
python app.py 5001
# Open http://localhost:5001 — connect gimbal serial, start camera, scan map
```

## Docs
- [Integration Plan](APRILTAG_GIMBAL_COMPENSATION_PLAN.md)
- [Logic and Math](LOGIC_AND_MATH.md) — coordinate systems, formulas, tracking, mapping, radar viz
