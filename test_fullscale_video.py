#!/usr/bin/env python3
"""Open full-scale (1920x1200) video in an OpenCV window for test. Press 'q' to quit."""

import sys
import cv2

from camera_capture import SimpleCamera, find_working_device

def main():
    dev = find_working_device()
    if not dev:
        print("No camera found")
        sys.exit(1)
    cam = SimpleCamera(device=dev)
    if not cam.open():
        print("Failed to open camera")
        sys.exit(1)
    w, h = cam._width, cam._height
    print(f"Full-scale test: {w}x{h} @ {dev}")
    print("Press 'q' to quit")
    cv2.namedWindow("Full-scale test", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Full-scale test", w, h)
    try:
        while True:
            ok, frame = cam.read()
            if not ok or frame is None:
                continue
            cv2.imshow("Full-scale test", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cam.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
