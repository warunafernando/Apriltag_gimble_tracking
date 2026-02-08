#!/usr/bin/env python3
"""Debug AprilTag detection - run with camera and tag in view.
   REQUIRED: Stop the backend first (pkill -f 'python3 app.py') so camera is free."""


import sys
import time

sys.path.insert(0, ".")

from camera_capture import SimpleCamera, find_working_device, list_video_devices
from apriltag_detect import AprilTagDetector


def main():
    print("Video devices:", list_video_devices())
    dev = find_working_device()
    if not dev:
        print("FAIL: No working camera found")
        return 1
    print("Using device:", dev)

    cap = SimpleCamera(device=dev)
    if not cap.open():
        print("FAIL: Could not open camera")
        return 1

    w = cap._width
    h = cap._height
    print(f"Camera: {w}x{h}")

    try:
        detector = AprilTagDetector()
    except Exception as e:
        print("FAIL: AprilTagDetector:", e)
        cap.close()
        return 1

    print("AprilTag detector ready (tag36h11). Point tag at camera, press Ctrl+C to stop.")
    print("-" * 50)

    import cv2
    ok_reads = 0
    det_count = 0
    for i in range(60):
        ok, frame = cap.read()
        if not ok or frame is None:
            time.sleep(0.1)
            continue
        ok_reads += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        dets = detector.detect(gray)
        if dets:
            det_count += len(dets)
            for d in dets:
                print(f"  Tag {d.tag_id} at ({d.center[0]:.1f}, {d.center[1]:.1f})")
        if (i + 1) % 10 == 0:
            print(f"[{i+1}] reads={ok_reads} detections={len(dets) if dets else 0} total={det_count}")
        time.sleep(0.1)

    cap.close()
    print("-" * 50)
    print("Done. If you saw 'Tag N' above, AprilTag is working.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
