# Gimbal binary protocol (GET_IMU=126, PAN_TILT_ABS=133, IMU=1002)
# Reference: gimble_conrol/compass_web/protocol_imu.py
# FW discovery: GIMBAL_SERIAL_CONNECTION_REFERENCE.md

import platform
import re
import struct
from typing import Optional, Tuple

STX = 0x02
ETX = 0x03
MAX_PAYLOAD = 251

CMD_GET_IMU = 126
CMD_PAN_TILT_ABS = 133
RSP_IMU = 1002

CMD_GET_FW_INFO = 610
RSP_FW_INFO = 2610
ACK_RECEIVED = 1
MODEL_ID_TARGET = 99
FW_INFO_PAYLOAD_WITH_MODEL = 70
FW_INFO_PAYLOAD_LEGACY = 65  # active_slot(1), version_a(32), version_b(32)


def crc8(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (0x07 ^ (crc << 1)) if (crc & 0x80) else (crc << 1)
        crc &= 0xFF
    return crc


def build_frame(seq: int, type_id: int, payload: Optional[bytes] = None) -> bytes:
    payload = payload or b""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    length = 4 + len(payload)
    body = struct.pack("<BHH", length, seq & 0xFFFF, type_id & 0xFFFF) + payload
    checksum = crc8(body)
    return bytes([STX]) + body + bytes([checksum, ETX])


def parse_frame(data: bytes) -> Optional[Tuple[int, int, bytes]]:
    if len(data) < 8 or data[0] != STX:
        return None
    length = data[1]
    if length < 4 or length > MAX_PAYLOAD:
        return None
    frame_size = 4 + length
    if len(data) < frame_size:
        return None
    if data[frame_size - 1] != ETX:
        return None
    body = data[1 : frame_size - 2]
    if crc8(body) != data[frame_size - 2]:
        return None
    seq = struct.unpack_from("<H", body, 1)[0]
    type_id = struct.unpack_from("<H", body, 3)[0]
    payload = body[5 : 5 + (length - 4)] if length > 4 else b""
    return (seq, type_id, payload)


def build_pan_tilt_abs(seq: int, pan: float, tilt: float, spd: int = 3400, acc: int = 100) -> bytes:
    payload = struct.pack("<ffHH", pan, tilt, spd, acc)
    return build_frame(seq, CMD_PAN_TILT_ABS, payload)


def decode_imu(payload: bytes) -> Optional[dict]:
    if len(payload) < 50:
        return None
    return {
        "roll": struct.unpack_from("<f", payload, 0)[0],
        "pitch": struct.unpack_from("<f", payload, 4)[0],
        "yaw": struct.unpack_from("<f", payload, 8)[0],
        "ax": struct.unpack_from("<f", payload, 12)[0],
        "ay": struct.unpack_from("<f", payload, 16)[0],
        "az": struct.unpack_from("<f", payload, 20)[0],
        "gx": struct.unpack_from("<f", payload, 24)[0],
        "gy": struct.unpack_from("<f", payload, 28)[0],
        "gz": struct.unpack_from("<f", payload, 32)[0],
        "mx": struct.unpack_from("<h", payload, 36)[0],
        "my": struct.unpack_from("<h", payload, 38)[0],
        "mz": struct.unpack_from("<h", payload, 40)[0],
        "temp": struct.unpack_from("<f", payload, 46)[0],
    }


def decode_fw_info(payload: bytes) -> Optional[dict]:
    """Decode FW_INFO. Supports 70-byte (slot,serial,model_id,ver_a,ver_b) or 65-byte (slot,ver_a,ver_b)."""
    try:
        if len(payload) >= FW_INFO_PAYLOAD_WITH_MODEL:
            serial_val = struct.unpack_from("<I", payload, 1)[0]
            model_id = payload[5]
            version_a = payload[6:38].rstrip(b"\x00").decode("utf-8", errors="replace")
            version_b = payload[38:70].rstrip(b"\x00").decode("utf-8", errors="replace")
            return {
                "active_slot": payload[0],
                "serial": serial_val,
                "model_id": model_id,
                "version_a": version_a,
                "version_b": version_b,
            }
        if len(payload) >= FW_INFO_PAYLOAD_LEGACY:
            version_a = payload[1:33].rstrip(b"\x00").decode("utf-8", errors="replace")
            version_b = payload[33:65].rstrip(b"\x00").decode("utf-8", errors="replace")
            return {
                "active_slot": payload[0],
                "serial": 0,
                "model_id": None,
                "version_a": version_a,
                "version_b": version_b,
            }
    except (struct.error, UnicodeDecodeError):
        pass
    return None


def serial_port_name(port: str) -> str:
    """On Windows, use \\\\.\\COMn for COM10+ and driver compatibility."""
    port = (port or "").strip()
    if not port:
        return port
    if platform.system() == "Windows":
        pu = port.upper()
        if pu.startswith("COM") and not pu.startswith("\\\\"):
            return "\\\\.\\" + pu
    return port


def com_sort_key(port: str) -> tuple:
    """Sort COM ports numerically (COM3, COM4, COM9, COM13)."""
    m = re.match(r"^COM(\d+)$", (port or "").upper())
    return (int(m.group(1)), port) if m else (99999, port)
