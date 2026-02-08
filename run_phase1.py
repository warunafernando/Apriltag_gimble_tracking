#!/usr/bin/env python3
"""Phase 1: Validate Arducam capture + AprilTag detection rate (target >= 25 fps)."""

import argparse
import sys
import time

import numpy as np

from camera_capture import ArducamCapture, list_video_devices, DEFAULT_WIDTH, DEFAULT_HEIGHT, DEFAULT_FPS
from apriltag_detect import AprilTagDetector


def run_benchmark(detector: AprilTagDetector, frames_iter, num_frames: int) -> tuple:
    """Run detection on frames from iterator. Returns (count, tag_count, elapsed)."""
    t0 = time.perf_counter()
    count = 0
    tag_count = 0
    for gray in frames_iter:
        if count >= num_frames:
            break
        if gray is None:
            continue
        dets = detector.detect(gray)
        count += 1
        tag_count += len(dets)
    elapsed = time.perf_counter() - t0
    return count, tag_count, elapsed


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/video0", help="Camera device")
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS)
    parser.add_argument("--frames", type=int, default=100, help="Frames to benchmark")
    parser.add_argument("--list", action="store_true", help="List video devices and exit")
    parser.add_argument("--test", action="store_true", help="Benchmark with synthetic frames (no camera)")
    parser.add_argument("--capture-only", action="store_true", help="Benchmark capture FPS only (no AprilTag)")
    parser.add_argument("--profile", action="store_true", help="Show capture vs detect timing breakdown")
    parser.add_argument("--no-gst", action="store_true", help="Use V4L2 instead of GStreamer")
    args = parser.parse_args()

    if args.list:
        devs = list_video_devices()
        print("Video devices:", devs if devs else "none")
        return 0

    try:
        detector = AprilTagDetector()
    except ImportError as e:
        print(e)
        return 1

    if args.test:
        # Synthetic frames: uniform gray (random noise can trigger apriltag segfault)
        def synth_frames():
            for _ in range(args.frames):
                yield np.full((args.height, args.width), 128, dtype=np.uint8)

        print("Phase 1 (test mode): AprilTag on synthetic frames (no camera)")
        count, tag_count, elapsed = run_benchmark(detector, synth_frames(), args.frames)
        fps = count / elapsed if elapsed > 0 else 0
        print(f"Frames: {count} in {elapsed:.2f}s -> {fps:.1f} fps")
        print(f"Total tags detected: {tag_count} (expected 0 for synthetic)")
        if fps >= 25:
            print("PASS: Detection pipeline throughput >= 25 fps")
        return 0

    cam = ArducamCapture(
        device=args.device,
        width=args.width,
        height=args.height,
        fps=args.fps,
        use_gstreamer=not args.no_gst,
    )
    if not cam.open():
        print("Failed to open camera. Try --list, --test, or --no-gst (V4L2 fallback).")
        return 1

    print(f"Camera: {args.width}x{args.height} @ {cam.fps:.1f} fps (backend={cam._backend})")

    if args.capture_only:
        t0 = time.perf_counter()
        count = 0
        for _ in range(args.frames):
            ok, gray = cam.read_gray()
            if not ok or gray is None:
                break
            count += 1
        elapsed = time.perf_counter() - t0
        cam.close()
        fps = count / elapsed if elapsed > 0 else 0
        print(f"Capture only: {count} frames in {elapsed:.2f}s -> {fps:.1f} fps")
        if fps < args.fps * 0.9:
            print(f"  Target {args.fps} fps. 1920x1200@50 needs ~115MB/s (USB3). Check: USB3 port, v4l2-ctl -d /dev/video0 --all")
        return 0

    def cam_frames():
        while True:
            ok, gray = cam.read_gray()
            if not ok:
                break
            yield gray

    if args.profile:
        detect_times = []
        count = 0
        for gray in cam_frames():
            if count >= args.frames:
                break
            t0 = time.perf_counter()
            detector.detect(gray)
            detect_times.append(time.perf_counter() - t0)
            count += 1
        cam.close()
        total = sum(detect_times)
        avg_detect_ms = (total / len(detect_times)) * 1000 if detect_times else 0
        fps = count / total if total > 0 else 0
        print(f"Profile: {count} frames")
        print(f"  Detect: {avg_detect_ms:.1f} ms/frame -> max {1000/avg_detect_ms:.0f} fps if capture were free")
        print(f"  Total: {total:.2f}s -> {fps:.1f} fps (run --capture-only for capture FPS)")
        return 0

    print(f"Phase 1: capture + AprilTag @ {args.width}x{args.height}")
    print(f"Running {args.frames} frames...")
    count, tag_count, elapsed = run_benchmark(detector, cam_frames(), args.frames)
    cam.close()
    fps = count / elapsed if elapsed > 0 else 0
    print(f"Frames: {count} in {elapsed:.2f}s -> {fps:.1f} fps")
    print(f"Total tags detected: {tag_count}")
    if fps >= 25:
        print("PASS: Detection rate >= 25 fps")
        return 0
    print("WARN: Detection rate < 25 fps (may need decimation or GPU preprocess)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
