"""
Phase 2: Web GUI – serial (IMU, pan/tilt), circular viz (tags, gimbal, compass), live stream, map/scan.
"""

import base64
import logging
import concurrent.futures
import io
import os
import sys
import threading
import time

from flask import Flask, send_from_directory, jsonify, Response, stream_with_context
from flask_socketio import SocketIO, emit

try:
    import serial
    from serial.tools import list_ports as serial_list_ports
except ImportError:
    serial = None
    serial_list_ports = None

from protocol import (
    CMD_GET_FW_INFO,
    CMD_GET_IMU,
    MODEL_ID_TARGET,
    RSP_FW_INFO,
    RSP_IMU,
    build_frame,
    build_pan_tilt_abs,
    com_sort_key,
    decode_fw_info,
    decode_imu,
    parse_frame,
    serial_port_name,
)
from camera_capture import SimpleCamera, list_video_devices, find_working_device
from apriltag_detect import AprilTagDetection, AprilTagDetector
from mapping import MapEntry, entries_to_tag_azimuths, world_azimuth, tag_azimuth, tag_size_pixels, tag_width_pixels, tag_distance, CAMERA_H_FOV_DEG

app = Flask(__name__, static_folder="static", static_url_path="")
app.config["SECRET_KEY"] = "gimbal-apriltag-vision"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading", max_http_buffer_size=2 * 1024 * 1024)

STX = 0x02
_serial = None
_running = False
_poll_thread = None
_keepalive_thread = None
_rx_buf = bytearray()
_seq = 0
_last_imu = None
_ping_sent_at = None
_connected = False
_port_name = ""
_current_pan = 0.0
_current_tilt = 42.0

# Map (scan result)
_map_entries: list = []
_tag_azimuths: dict = {}

# Camera
_cam = None
_detector = None
_cam_thread = None
_cam_running = False
_cam_device = os.environ.get("CAMERA_DEVICE", "/dev/video0")
_last_jpeg_bytes = None  # raw JPEG bytes for MJPEG stream
_last_tags = []
_last_img_width = 960  # width of frame used for detection (for FOV calibration)
_distance_scale = 1.0  # multiply distance by this (set via socket for calibration, e.g. 8/displayed_ft)
STREAM_FPS = 50
_fov_calibration_running = False

# Logs for UI (last 200 lines)
_log_buffer = []
_LOG_MAX = 200


def _emit_log(level: str, msg: str):
    """Append to buffer and emit to connected clients."""
    import datetime
    entry = {"ts": datetime.datetime.now().strftime("%H:%M:%S"), "level": level, "msg": msg}
    _log_buffer.append(entry)
    if len(_log_buffer) > _LOG_MAX:
        _log_buffer.pop(0)
    try:
        with app.app_context():
            socketio.emit("log", entry)
    except Exception:
        pass


def find_stx(data: bytes) -> int:
    try:
        return data.index(STX)
    except ValueError:
        return -1


def next_seq() -> int:
    global _seq
    _seq = (_seq + 1) & 0xFFFF
    return _seq


def _read_loop():
    global _last_imu
    while _running and _serial and getattr(_serial, "is_open", False):
        try:
            chunk = _serial.read(256)
            if chunk:
                _rx_buf.extend(chunk)
                _process_rx()
        except Exception:
            break
        time.sleep(0.001)


def _process_rx():
    global _last_imu, _ping_sent_at
    while len(_rx_buf) >= 8:
        idx = find_stx(bytes(_rx_buf))
        if idx < 0:
            _rx_buf.clear()
            return
        if idx > 0:
            del _rx_buf[:idx]
        length = _rx_buf[1]
        if length < 4 or length > 251:
            _rx_buf.pop(0)
            continue
        frame_size = 4 + length
        if len(_rx_buf) < frame_size:
            return
        frame_bytes = bytes(_rx_buf[:frame_size])
        result = parse_frame(frame_bytes)
        del _rx_buf[:frame_size]
        if result is None:
            continue
        _seq_rx, type_id, payload = result
        if type_id == RSP_FW_INFO and len(payload) >= 70:
            _ping_sent_at = None
        elif type_id == RSP_IMU and len(payload) >= 50:
            imu = decode_imu(payload)
            if imu:
                _last_imu = imu
                with app.app_context():
                    socketio.emit("imu", imu)
    if len(_rx_buf) > 4096:
        _rx_buf.clear()


def _poll_imu_loop():
    while _running and _serial and getattr(_serial, "is_open", False):
        try:
            frame = build_frame(next_seq(), CMD_GET_IMU, b"")
            _serial.write(frame)
        except Exception:
            break
        time.sleep(0.05)


PROBE_TIMEOUT = 2.0
KEEPALIVE_INTERVAL = 30.0
KEEPALIVE_NO_RESPONSE = 5.0


def _probe_one(port: str, baud: int = 921600):
    """Open port, send GET_FW_INFO, read until FW_INFO or timeout. Return (serial, fw_info) or (None, None)."""
    if serial is None:
        return None, None
    port_actual = serial_port_name(port)
    ser = None
    try:
        ser = serial.Serial(port_actual, baud, timeout=0.05)
        ser.reset_input_buffer()
        ser.write(build_frame(next_seq(), CMD_GET_FW_INFO, b""))
        ser.write(build_frame(next_seq(), CMD_GET_IMU, b""))
        deadline = time.time() + PROBE_TIMEOUT
        buf = bytearray()
        while time.time() < deadline:
            chunk = ser.read(256)
            if chunk:
                buf.extend(chunk)
            while len(buf) >= 8:
                idx = find_stx(bytes(buf))
                if idx < 0:
                    buf.clear()
                    break
                if idx > 0:
                    del buf[:idx]
                length = buf[1]
                if length < 4 or length > 251:
                    buf.pop(0)
                    continue
                frame_size = 4 + length
                if len(buf) < frame_size:
                    break
                frame_bytes = bytes(buf[:frame_size])
                result = parse_frame(frame_bytes)
                del buf[:frame_size]
                if result is None:
                    continue
                _, type_id, payload = result
                if type_id == RSP_FW_INFO and len(payload) >= 65:
                    fw_info = decode_fw_info(payload)
                    if fw_info:
                        mid = fw_info.get("model_id")
                        if mid == MODEL_ID_TARGET or mid is None:
                            ser.timeout = 0.02
                            return ser, fw_info
                elif type_id == RSP_IMU and len(payload) >= 50:
                    fw_info = {"version_a": "?", "version_b": "?", "model_id": None}
                    ser.timeout = 0.02
                    return ser, fw_info
                elif type_id not in (1, 2):
                    break
            time.sleep(0.01)
    except Exception:
        pass
    if ser and getattr(ser, "is_open", False):
        try:
            ser.close()
        except Exception:
            pass
    return None, None


def probe_port_for_gimbal(port: str, baud: int = 921600):
    """Probe one port. Returns (serial, fw_info) or (None, None). Does NOT close serial on success."""
    return _probe_one(port, baud)


def auto_connect_serial(baud: int = 921600):
    """Probe all ports in parallel, return first gimbal found. Returns (ok, port, fw_info)."""
    global _serial, _running, _poll_thread, _keepalive_thread, _connected, _port_name, _rx_buf
    disconnect_serial()
    if serial is None:
        return False, "", None
    ports = [p["device"] for p in list_serial_ports() if p.get("device")]
    if sys.platform == "win32":
        ports = sorted(ports, key=com_sort_key)
    result_port = None
    result_ser = None
    result_fw = None
    other_serials = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=min(16, max(len(ports), 1))) as ex:
        futures = {ex.submit(_probe_one, p, baud): p for p in ports}
        for fut in concurrent.futures.as_completed(futures):
            ser, fw = fut.result()
            if ser is not None and fw is not None:
                if result_ser is None:
                    result_ser = ser
                    result_port = futures[fut]
                    result_fw = fw
                    for f in futures:
                        f.cancel()
                else:
                    other_serials.append(ser)
            elif ser is not None:
                other_serials.append(ser)
    for s in other_serials:
        try:
            if s and getattr(s, "is_open", False):
                s.close()
        except Exception:
            pass
    if result_ser is None or result_port is None:
        return False, "", None
    _serial = result_ser
    _rx_buf.clear()
    _running = True
    _connected = True
    _port_name = result_port
    _ping_sent_at = None
    t = threading.Thread(target=_read_loop, daemon=True)
    t.start()
    p = threading.Thread(target=_poll_imu_loop, daemon=True)
    p.start()
    _poll_thread = p
    k = threading.Thread(target=_keepalive_loop, daemon=True)
    k.start()
    _keepalive_thread = k
    with app.app_context():
        socketio.emit("state", {"connected": True, "port": result_port})
        if result_fw:
            socketio.emit("fw_info", result_fw)
    return True, result_port, result_fw


def _keepalive_loop():
    global _ping_sent_at
    while _running and _serial and getattr(_serial, "is_open", False):
        time.sleep(KEEPALIVE_INTERVAL)
        if not _running:
            break
        try:
            frame = build_frame(next_seq(), CMD_GET_FW_INFO, b"")
            _serial.write(frame)
            _ping_sent_at = time.time()
        except Exception:
            break
        deadline = time.time() + KEEPALIVE_NO_RESPONSE
        while time.time() < deadline and _ping_sent_at is not None and _running:
            time.sleep(0.2)
        if _ping_sent_at is not None and _running and _serial and getattr(_serial, "is_open", False):
            with app.app_context():
                socketio.emit("state", {"connected": False, "port": "", "error": "Keepalive timeout"})
            disconnect_serial()
            break


def connect_serial(port: str, baud: int = 921600, existing_serial=None) -> bool:
    global _serial, _running, _poll_thread, _keepalive_thread, _connected, _port_name, _rx_buf
    disconnect_serial()
    if serial is None:
        return False
    try:
        if existing_serial and getattr(existing_serial, "is_open", False):
            _serial = existing_serial
            _serial.timeout = 0.02
            _port_name = port
        else:
            port_actual = serial_port_name(port)
            _serial = serial.Serial(port_actual, baud, timeout=0.02)
            _port_name = port
        _rx_buf.clear()
        _running = True
        _connected = True
        t = threading.Thread(target=_read_loop, daemon=True)
        t.start()
        p = threading.Thread(target=_poll_imu_loop, daemon=True)
        p.start()
        _poll_thread = p
        k = threading.Thread(target=_keepalive_loop, daemon=True)
        k.start()
        _keepalive_thread = k
        with app.app_context():
            socketio.emit("state", {"connected": True, "port": _port_name})
        return True
    except Exception as e:
        with app.app_context():
            socketio.emit("state", {"connected": False, "port": "", "error": str(e)})
        return False


def send_pan_tilt(pan: float, tilt: float) -> bool:
    global _current_pan, _current_tilt
    if not _serial or not getattr(_serial, "is_open", False):
        return False
    try:
        frame = build_pan_tilt_abs(next_seq(), pan, tilt)
        _serial.write(frame)
        _current_pan = pan
        _current_tilt = tilt
        return True
    except Exception:
        return False


def disconnect_serial():
    global _serial, _running, _poll_thread, _keepalive_thread, _connected, _port_name
    _running = False
    time.sleep(0.2)
    if _serial and getattr(_serial, "is_open", False):
        try:
            _serial.close()
        except Exception:
            pass
        _serial = None
    _connected = False
    _port_name = ""
    with app.app_context():
        socketio.emit("state", {"connected": False, "port": ""})


def list_serial_ports():
    result = []
    if serial_list_ports is not None:
        try:
            for p in serial_list_ports.comports():
                result.append({"device": p.device, "description": p.description or "", "hwid": p.hwid or ""})
        except Exception as e:
            result.append({"device": "", "description": "Error: " + str(e), "hwid": ""})
    if sys.platform == "linux":
        for name in ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyAMA0", "/dev/ttyS0", "/dev/ttyS1"]:
            if os.path.exists(name) and not any(r["device"] == name for r in result):
                result.append({"device": name, "description": "serial port", "hwid": ""})
    if sys.platform == "win32":
        result = sorted(result, key=lambda r: com_sort_key(r.get("device", "")))
    return result


def _detection_to_dict(d, camera_azimuth: float = None, img_width: int = None) -> dict:
    out = {"tag_id": d.tag_id, "center": list(d.center), "corners": d.corners.tolist()}
    width_px = tag_width_pixels(d.corners)
    out["width_px"] = round(width_px, 1)
    if camera_azimuth is not None and img_width is not None:
        out["azimuth"] = tag_azimuth(camera_azimuth, float(d.center[0]), float(img_width))
    # Distance from horizontal tag size (matches horizontal FOV): distance = (tag_size_m * focal_px) / tag_width_px
    w = img_width if img_width is not None else 960
    if width_px > 0 and w > 0:
        dist_m = tag_distance(width_px, float(w), scale=_distance_scale)
        out["distance_m"] = round(dist_m, 3)
        out["distance_ft"] = round(dist_m * 3.28084, 2)
    return out


def _camera_loop():
    """Capture Y channel only at 1200p 50fps; detect AprilTags; stream grayscale with overlay."""
    global _cam, _detector, _last_jpeg_bytes, _last_tags
    import cv2
    import numpy as np
    fps_smooth = 0.0
    frame_cnt = 0
    while _cam_running and _cam and _detector:
        try:
            t0 = time.perf_counter()
            ok, gray = _cam.read_y()
            if not ok or gray is None:
                time.sleep(0.02)
                continue
            frame_cnt += 1
            h, w = gray.shape[:2]
            scale = min(960 / w, 960 / h, 1.0)
            if scale < 1.0:
                frame_disp = cv2.resize(gray, (int(w * scale), int(h * scale)))
            else:
                frame_disp = gray.copy()
            dets = _detector.detect(frame_disp)
            imu = _last_imu
            yaw = imu["yaw"] if imu else 0.0
            yaw = (yaw % 360 + 360) % 360
            cam_az = world_azimuth(yaw, _current_pan)
            w_disp = frame_disp.shape[1]
            _last_img_width = w_disp
            tag_list = [_detection_to_dict(d, cam_az, w_disp) for d in dets]
            _last_tags = tag_list
            dt = time.perf_counter() - t0
            fps_smooth = fps_smooth * 0.8 + (1.0 / dt) * 0.2 if dt > 0 else fps_smooth
            with app.app_context():
                socketio.emit("tags_live", {"tags": tag_list, "fps": round(fps_smooth, 1)})
            try:
                frame_overlay = _detector.draw_overlay(frame_disp, dets)
            except Exception:
                frame_overlay = frame_disp
            if len(frame_overlay.shape) == 2:
                frame_overlay = cv2.cvtColor(frame_overlay, cv2.COLOR_GRAY2BGR)
            frame_overlay = np.ascontiguousarray(frame_overlay, dtype=np.uint8)
            ok_enc, jpeg = cv2.imencode(".jpg", frame_overlay, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok_enc and jpeg is not None and jpeg.size > 0:
                _last_jpeg_bytes = jpeg.tobytes()
            else:
                frame_fallback = cv2.cvtColor(frame_disp, cv2.COLOR_GRAY2BGR) if len(frame_disp.shape) == 2 else frame_disp
                ok2, buf = cv2.imencode(".jpg", frame_fallback, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ok2 and buf is not None and buf.size > 0:
                    _last_jpeg_bytes = buf.tobytes()
            elapsed = time.perf_counter() - t0
            sleep_time = (1.0 / STREAM_FPS) - elapsed
            if sleep_time > 0.001:
                time.sleep(sleep_time)
        except Exception:
            time.sleep(0.1)
    if _cam:
        try:
            _cam.close()
        except Exception:
            pass
        _cam = None


def _camera_res_fps():
    """Return (width, height, fps) for current camera, or (0, 0, 0) if none."""
    global _cam, STREAM_FPS
    if _cam and _cam.is_open():
        w = getattr(_cam, "_width", 0) or 0
        h = getattr(_cam, "_height", 0) or 0
        return (w, h, STREAM_FPS)
    return (0, 0, 0)


def start_camera(device: str = None):
    global _cam, _detector, _cam_thread, _cam_running, _last_tags, _last_jpeg_bytes
    prefer = (device or _cam_device or "").strip() or _cam_device
    if _cam_running and _cam and _cam.is_open():
        return True, None
    stop_camera()
    time.sleep(1.0)
    _last_tags = []
    _last_jpeg_bytes = None
    last_err = None
    for attempt in range(3):
        dev = find_working_device(prefer=prefer)
        if dev:
            try:
                _cam = SimpleCamera(device=dev)
                if _cam.open():
                    _detector = AprilTagDetector()
                    _cam_running = True
                    _cam_thread = threading.Thread(target=_camera_loop, daemon=True)
                    _cam_thread.start()
                    return True, None
            except Exception as e:
                _cam = None
                _detector = None
                last_err = str(e)
        time.sleep(0.3)
    return False, last_err or "No working camera. Ensure /dev/video0 is free (close other apps) and try again."


def stop_camera():
    global _cam_running, _last_tags
    _cam_running = False
    _last_tags = []
    time.sleep(0.3)


def scan_map(pan_start: float, pan_end: float, step_deg: float) -> list:
    """Pan gimbal in steps, at each step read IMU + capture + detect, return entries."""
    global _map_entries, _tag_azimuths, _cam_running, _detector
    if not _cam:
        return []
    if _detector is None:
        try:
            _detector = AprilTagDetector()
        except Exception:
            return []
    # Pause camera loop so we can read frames here
    was_running = _cam_running
    _cam_running = False
    time.sleep(0.2)
    entries = []
    step = step_deg if pan_end >= pan_start else -abs(step_deg)
    n = max(1, int(abs(pan_end - pan_start) / abs(step)) + 1)
    try:
        for i in range(n):
            pan = pan_start + i * step
            send_pan_tilt(pan, _current_tilt)
            time.sleep(0.2)
            imu = _last_imu
            ok, gray = _cam.read_gray()
            if not ok or gray is None:
                continue
            dets = _detector.detect(gray)
            yaw = imu["yaw"] if imu else 0.0
            if yaw < 0:
                yaw += 360
            yaw = (yaw % 360 + 360) % 360
            tag_list = [{"tag_id": d.tag_id, "center": list(d.center)} for d in dets]
            entries.append(MapEntry(
                pan=pan,
                tilt=_current_tilt,
                imu_yaw=yaw,
                timestamp=time.time(),
                tags=tag_list,
            ))
            with app.app_context():
                socketio.emit("scan_progress", {"current": pan, "total": n, "entry": {"pan": pan, "tilt": _current_tilt, "imu_yaw": yaw, "tags": [t["tag_id"] for t in tag_list]}})
        _map_entries = entries
        _tag_azimuths = entries_to_tag_azimuths(entries)
    finally:
        if was_running:
            _cam_running = True
    return entries


def _run_fov_calibration():
    """Pan gimbal slowly from -50° to +50°. Record only when full tag is visible; FOV = pan span from first to last detection."""
    global _fov_calibration_running
    if _fov_calibration_running:
        return
    _fov_calibration_running = True
    try:
        with app.app_context():
            socketio.emit("fov_calibration_status", {"running": True, "msg": "Moving slowly – detecting only when full tag is visible..."})
        pan_start = -50
        pan_end = 50
        step = 0.5
        step_pause_s = 1.2
        samples = []  # (pan, camera_az, tag_id, center_x, width_px)
        n_steps = int(round((pan_end - pan_start) / step)) + 1
        for i in range(n_steps):
            pan = pan_start + i * step
            send_pan_tilt(pan, _current_tilt)
            time.sleep(step_pause_s)
            imu = _last_imu
            yaw = (imu["yaw"] % 360 + 360) % 360 if imu else 0.0
            camera_az = world_azimuth(yaw, _current_pan)
            for t in _last_tags:
                tid = t.get("tag_id")
                center = t.get("center")
                width_px = t.get("width_px") or 0
                if tid is not None and center and len(center) >= 1:
                    samples.append((pan, camera_az, tid, float(center[0]), float(width_px)))
            with app.app_context():
                socketio.emit("fov_calibration_progress", {"pan": round(pan, 1), "step": i + 1, "total": n_steps})
        # Per tag: first and last pan where tag was detected (full tag visible); FOV = that span. Include tag size.
        by_tag = {}
        for (pan, camera_az, tid, cx, width_px) in samples:
            by_tag.setdefault(tid, []).append((pan, camera_az, cx, width_px))
        results = []
        w = _last_img_width or 960
        for tid, points in by_tag.items():
            if len(points) < 3:
                continue
            pans = [p[0] for p in points]
            pan_first = min(pans)
            pan_last = max(pans)
            fov_deg = abs(pan_last - pan_first)
            if fov_deg > 180:
                fov_deg = 360 - fov_deg
            widths = [p[3] for p in points if p[3] > 0]
            avg_width_px = round(sum(widths) / len(widths), 1) if widths else 0
            first_pt = next(p for p in points if p[0] == pan_first)
            last_pt = next(p for p in points if p[0] == pan_last)
            results.append({
                "tag_id": tid,
                "fov_deg": round(fov_deg, 1),
                "pan_first": round(pan_first, 1),
                "pan_last": round(pan_last, 1),
                "az_first": round(first_pt[1], 1),
                "az_last": round(last_pt[1], 1),
                "x_first": round(first_pt[2], 0),
                "x_last": round(last_pt[2], 0),
                "tag_width_px_avg": avg_width_px,
                "visible_count": len(points),
                "img_width": w,
            })
        with app.app_context():
            socketio.emit("fov_calibration_status", {"running": False, "msg": "Done"})
            socketio.emit("fov_calibration_result", {"ok": True, "results": results, "distance_ft": 8})
    except Exception as e:
        with app.app_context():
            socketio.emit("fov_calibration_status", {"running": False, "msg": "Error: " + str(e)})
            socketio.emit("fov_calibration_result", {"ok": False, "error": str(e)})
    finally:
        _fov_calibration_running = False


# ---- Routes ----
@app.route("/api/restart", methods=["POST"])
def api_restart():
    """Restart: spawn new process, then exit. More reliable than execv when run from IDE/sandbox."""
    def _restart():
        time.sleep(2)
        import subprocess
        cwd = os.path.dirname(os.path.abspath(__file__))
        subprocess.Popen(
            [sys.executable, os.path.abspath(__file__)] + sys.argv[1:],
            cwd=cwd,
            start_new_session=True,
            stdout=open("/tmp/gimbal_app.log", "a"),
            stderr=subprocess.STDOUT,
        )
        os._exit(0)
    threading.Thread(target=_restart, daemon=False).start()
    return jsonify({"ok": True, "msg": "Restarting in 2s..."})


@app.route("/api/serial_ports")
def api_serial_ports():
    return jsonify(list_serial_ports())


@app.route("/api/serial_diagnostic")
def api_serial_diagnostic():
    """Diagnostic info for serial connection issues."""
    ports = list_serial_ports()
    has_usb = any("/dev/ttyUSB" in (p.get("device") or "") for p in ports)
    tip = None
    if sys.platform == "linux" and not has_usb:
        tip = "No /dev/ttyUSB0 found. If gimbal is USB (CP210x): run 'sudo modprobe cp210x' then unplug/replug gimbal."
    return jsonify({"ports": ports, "has_usb_serial": has_usb, "tip": tip})


@app.route("/api/video_devices")
def api_video_devices():
    return jsonify(list_video_devices())


@app.route("/api/camera_test")
def api_camera_test():
    """Test if camera can be opened."""
    devices = list_video_devices()
    results = []
    for dev in devices:
        try:
            cap = SimpleCamera(device=dev)
            ok = cap.open()
            if ok:
                r, _ = cap.read()
                cap.close()
                results.append({"device": dev, "ok": True})
            else:
                results.append({"device": dev, "ok": False})
        except Exception as e:
            results.append({"device": dev, "ok": False, "error": str(e)})
    return jsonify({"devices": devices, "results": results})


def _mjpeg_frame(jpeg_bytes: bytes) -> bytes:
    """Format one JPEG as MJPEG multipart frame."""
    return (
        b"--frame\r\n"
        b"Content-Type: image/jpeg\r\n"
        b"Content-Length: " + str(len(jpeg_bytes)).encode() + b"\r\n\r\n" + jpeg_bytes + b"\r\n"
    )


def _placeholder_jpeg() -> bytes:
    """Placeholder JPEG when no frame available."""
    import cv2
    import numpy as np
    img = np.zeros((180, 320, 3), dtype=np.uint8)
    img[:] = (50, 50, 50)
    cv2.putText(img, "No camera", (80, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (150, 150, 150), 2)
    cv2.putText(img, "Click Start camera", (50, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 120, 120), 1)
    _, jpeg = cv2.imencode(".jpg", img)
    return jpeg.tobytes()


@app.route("/api/frame")
def api_frame():
    """Single JPEG snapshot - reliable fallback when MJPEG streaming fails."""
    if _cam_running and _last_jpeg_bytes:
        return Response(_last_jpeg_bytes, mimetype="image/jpeg")
    return Response(_placeholder_jpeg(), mimetype="image/jpeg")


@app.route("/api/stream")
def api_stream():
    """MJPEG stream - browser img src can display this directly."""
    def generate():
        while True:
            if not _cam_running:
                yield _mjpeg_frame(_placeholder_jpeg())
            elif _last_jpeg_bytes:
                yield _mjpeg_frame(_last_jpeg_bytes)
            else:
                yield _mjpeg_frame(_placeholder_jpeg())
            time.sleep(1.0 / STREAM_FPS)

    return Response(
        stream_with_context(generate()),
        mimetype="multipart/x-mixed-replace; boundary=frame",
        headers={
            "Cache-Control": "no-cache, no-store, must-revalidate",
            "Pragma": "no-cache",
            "X-Accel-Buffering": "no",
        },
    )


@app.route("/api/map")
def api_map():
    tag_list = [{"tag_id": k, "azimuth": v} for k, v in _tag_azimuths.items()]
    return jsonify({"tags": tag_list, "entries_count": len(_map_entries)})


@app.route("/")
def index():
    r = send_from_directory("static", "index.html")
    r.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
    return r


# ---- SocketIO ----
@socketio.on("connect")
def on_connect():
    _emit_log("info", "Client connected")
    emit("state", {"connected": _connected, "port": _port_name})
    if _last_imu:
        emit("imu", _last_imu)
    emit("pan_tilt", {"pan": _current_pan, "tilt": _current_tilt})
    emit("map", {"tags": [{"tag_id": k, "azimuth": v} for k, v in _tag_azimuths.items()]})
    emit("camera_config", {"fov_h": round(CAMERA_H_FOV_DEG, 1)})
    if _last_tags and _cam_running:
        emit("tags_live", {"tags": _last_tags, "fps": "--"})
    res_fps = _camera_res_fps()
    emit("camera_state", {"connected": _cam_running, "width": res_fps[0], "height": res_fps[1], "fps": res_fps[2]})
    emit("log_backlog", {"logs": _log_buffer})


@socketio.on("connect_serial")
def on_connect_serial(data):
    port = data.get("port", "COM3")
    baud = int(data.get("baud", 921600))
    ser, fw_info = probe_port_for_gimbal(port, baud)
    if ser is not None and fw_info is not None:
        ok = connect_serial(port, baud, existing_serial=ser)
        emit("connect_serial_result", {"ok": ok, "port": port})
        if fw_info:
            emit("fw_info", fw_info)
    else:
        emit("connect_serial_result", {"ok": False, "port": port})
        emit("state", {"connected": False, "port": "", "error": "Gimbal not found (model_id != 99) or port busy"})


@socketio.on("auto_connect")
def on_auto_connect(data):
    baud = int(data.get("baud", 921600)) if data else 921600
    ports = [p["device"] for p in list_serial_ports() if p.get("device")]
    _emit_log("info", "Auto-connecting serial (baud={}, ports={})".format(baud, ports or "none"))
    if not ports and sys.platform == "linux":
        _emit_log("warn", "No serial ports. USB gimbal? Try: sudo modprobe cp210x")
    ok, port, fw_info = auto_connect_serial(baud)
    if ok:
        _emit_log("info", "Serial connected: {}".format(port))
    else:
        if not any("/dev/ttyUSB" in p for p in ports) and sys.platform == "linux":
            _emit_log("warn", "No /dev/ttyUSB0? Load driver: sudo modprobe cp210x")
        _emit_log("warn", "Serial auto-connect failed")
    emit("auto_connect_result", {"ok": ok, "port": port, "fw_info": fw_info})


@socketio.on("disconnect_serial")
def on_disconnect_serial():
    disconnect_serial()
    emit("state", {"connected": False, "port": ""})


@socketio.on("set_pan_tilt")
def on_set_pan_tilt(data):
    pan = float(data.get("pan", 0))
    tilt = float(data.get("tilt", 42))
    ok = send_pan_tilt(pan, tilt)
    emit("set_pan_tilt_result", {"ok": ok})
    # When gimbal disconnected, _current_pan/tilt stay old; emit requested values so map updates
    pan_out = _current_pan if ok else pan
    tilt_out = _current_tilt if ok else tilt
    emit("pan_tilt", {"pan": pan_out, "tilt": tilt_out})


@socketio.on("start_camera")
def on_start_camera(data):
    dev = (data or {}).get("device") or _cam_device
    _emit_log("info", "Starting camera (device={})".format(dev or "auto"))
    result = start_camera(dev)
    ok = result[0] if isinstance(result, tuple) else result
    err = result[1] if isinstance(result, tuple) and len(result) > 1 else None
    if ok:
        w, h, fps = _camera_res_fps()
        _emit_log("info", "Camera started successfully")
        emit("camera_result", {"ok": True, "error": None, "action": "started", "width": w, "height": h, "fps": fps})
    else:
        _emit_log("error", "Camera failed: {}".format(err or "unknown"))
        emit("camera_result", {"ok": False, "error": err, "action": None})


@socketio.on("stop_camera")
def on_stop_camera():
    _emit_log("info", "Camera stopped")
    stop_camera()
    emit("camera_result", {"ok": True, "action": "stopped"})


@socketio.on("scan_map")
def on_scan_map(data):
    pan_start = float(data.get("pan_start", -180))
    pan_end = float(data.get("pan_end", 180))
    step = float(data.get("step", 10))
    entries = scan_map(pan_start, pan_end, step)
    emit("map", {"tags": [{"tag_id": k, "azimuth": v} for k, v in _tag_azimuths.items()], "entries_count": len(entries)})


@socketio.on("start_fov_calibration")
def on_start_fov_calibration(data):
    """Start FOV calibration in a background thread: pan -50° to +50°, record tag center_x at each step, compute FOV from left/right edges."""
    thread = threading.Thread(target=_run_fov_calibration)
    thread.daemon = True
    thread.start()
    emit("fov_calibration_status", {"running": True, "msg": "Calibration started"})


@socketio.on("set_distance_scale")
def on_set_distance_scale(data):
    """Set distance scale: displayed_distance *= scale. E.g. if tag is 8 ft but shows 7.2, send scale = 8/7.2."""
    global _distance_scale
    try:
        s = float(data.get("scale", 1.0))
        if 0.1 <= s <= 3.0:
            _distance_scale = s
            emit("distance_scale_result", {"ok": True, "scale": _distance_scale})
        else:
            emit("distance_scale_result", {"ok": False, "error": "Scale must be between 0.1 and 3.0"})
    except (TypeError, ValueError) as e:
        emit("distance_scale_result", {"ok": False, "error": str(e)})


if __name__ == "__main__":
    port = 5001
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            pass
    _emit_log("info", "Server starting on http://localhost:{}".format(port))
    print("Gimbal AprilTag Vision: http://localhost:{}".format(port))
    print("Connect gimbal serial + camera, then open the page.")
    socketio.run(app, host="0.0.0.0", port=port, debug=False, allow_unsafe_werkzeug=True)
