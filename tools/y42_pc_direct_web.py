#!/usr/bin/env python3
"""Full local web debugger for direct PC -> USB-RS485 -> Y42 motors.

The ESP32 debugger's motor-facing protocol is reused here, but the browser
talks to this process instead of to an ESP32 web server. GPIO pneumatic
outputs are intentionally not emulated: a PC/RS485 adapter has no GPIO pins.
"""

from __future__ import annotations

import copy
import json
import math
import statistics
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import y42_pc_direct as direct


HOST = "127.0.0.1"
PORT = 8787
MOTOR_IDS = (1, 2, 3, 4)
MOTOR_LABELS = ("左上", "右上", "右下", "左下")
PULL_DIRECTIONS = (0, 1, 0, 1)
HARD_CURRENT = 3000
HARD_TEMPERATURE = 55
CALIBRATION_DEFAULT_STRONG_CURRENT = 1500
CALIBRATION_DEFAULT_WEAK_CURRENT = 300
TRAJECTORY_DEFAULT_CURRENT = 2000
TRAJECTORY_START_RPM = 45.0
TRAJECTORY_START_TICK = 0.02
TRAJECTORY_START_SETTLE_SECONDS = 0.35
TRAJECTORY_POSITION_TOLERANCE = 8.0
# A closed-loop driver can stop a few degrees short when the cable tension
# settles.  Use this bounded fallback only after every motor has stopped; it
# prevents an endless wait while keeping a large position error as a hard
# failure.
TRAJECTORY_NEAR_POSITION_TOLERANCE = 18.0
TRAJECTORY_NEAR_SETTLE_SECONDS = 0.80
TRAJECTORY_FINAL_REASSERT_INTERVAL = 0.35
TRAJECTORY_LOG_INTERVAL = 0.10
TRAJECTORY_FEEDBACK_FILTER_ALPHA = 0.45
# One 0xAA frame contains all four 0xCB position records.  At 115200 baud a
# 61-byte frame occupies about 5.3 ms, so a 10 ms clock leaves room for the
# half-duplex turn-around and a short safety query without sending four
# separate, phase-shifted motor commands.
TRAJECTORY_HZ = 100.0
TRAJECTORY_INTERVAL = 1.0 / TRAJECTORY_HZ
TRAJECTORY_MAX_STEP_MM = 3.5
TRAJECTORY_PRETENSION_MM = 1.0
TRAJECTORY_MAX_RPM = 3000.0
TRAJECTORY_FEEDFORWARD = 1.25
# Feedback queries are sampled on a separate, lower-rate cadence.  One motor
# position is read every five command ticks, so a complete four-motor feedback
# path arrives about every 200 ms without replacing the 100 Hz AA command
# clock with four phase-shifted motor writes.
TRAJECTORY_POSITION_SAMPLE_EVERY_FRAMES = 5
TRAJECTORY_TARGET_TRACE_SAMPLE_EVERY_FRAMES = 2
TRAJECTORY_SAFETY_INTERVAL = 0.5
TRAJECTORY_TRACE_MAX_POINTS = 2500
MAX_SLOPE_SPREAD = 3.7
MAX_CROSS_MOTOR_SLOPE_RATIO = 1.08
DIRECTION_TEST_RPM = 8.0
DIRECTION_TEST_TIMEOUT = 8.0
DIRECTION_TEST_DEGREES = 45.0
DIRECTION_TEST_TOLERANCE = 3.0
DIRECTION_TEST_MAX_TRAVEL = 65.0
CALIBRATION_FILE = Path(__file__).with_name("y42_pc_calibration.json")
PROCESS_LOG_FILE = Path(__file__).with_name("y42_pc_process_log.jsonl")

SERIAL_LOCK = threading.Lock()
STATE_LOCK = threading.Lock()
PREPARE_GUARD = threading.Lock()
STATUS_POLL_LOCK = threading.Lock()
STATUS_CACHE_LOCK = threading.Lock()
STOP_EVENT = threading.Event()
AUTO_THREAD: threading.Thread | None = None
TRAJECTORY_SPEC: dict | None = None
STATUS_CACHE = {"at": 0.0, "port": "", "baud": 0, "recovered": False, "result": None}
PROCESS_LOG_LOCK = threading.Lock()
STATUS_SUCCESS_CACHE_SECONDS = 0.25
STATUS_FAILURE_CACHE_SECONDS = 1.5


def blank_calibration() -> dict:
    return {
        "valid": False,
        "side": 150.0,
        "ball": 40.0,
        "effective_side": 150.0,
        "workspace_width": 110.0,
        "workspace_height": 150.0,
        "geometry_version": 2,
        "slope": [0.0, 0.0, 0.0, 0.0],
        "intercept": [0.0, 0.0, 0.0, 0.0],
        "r2": [0.0, 0.0, 0.0, 0.0],
        "groups": [[0.0, 0.0, 0.0] for _ in MOTOR_IDS],
        "spread": [0.0, 0.0, 0.0, 0.0],
        "samples": [],
        "direction_reverse": [False, False, False, False],
        "encoder_positive_direction": [],
        "quality_error": "",
        "strong_current": CALIBRATION_DEFAULT_STRONG_CURRENT,
        "weak_current": CALIBRATION_DEFAULT_WEAK_CURRENT,
        "trajectory_current": TRAJECTORY_DEFAULT_CURRENT,
        "slope_trim_percent": [0.0, 0.0, 0.0, 0.0],
    }


def append_process_log(event: str, **fields: object) -> None:
    """Append a compact JSONL record for later trajectory/calibration analysis."""
    record = {"ts": round(time.time(), 3), "monotonic": round(time.monotonic(), 3), "event": event}
    record.update(fields)
    try:
        line = json.dumps(record, ensure_ascii=False, separators=(",", ":"), default=str) + "\n"
        with PROCESS_LOG_LOCK:
            with PROCESS_LOG_FILE.open("a", encoding="utf-8", newline="") as handle:
                handle.write(line)
        with STATE_LOCK:
            STATE["process_log_count"] = int(STATE.get("process_log_count", 0)) + 1
            STATE["last_process_event"] = event
    except OSError:
        # Logging must never interrupt a motor safety path.
        return


def recent_process_log(limit: int = 200) -> list[dict]:
    try:
        with PROCESS_LOG_LOCK:
            lines = PROCESS_LOG_FILE.read_text(encoding="utf-8").splitlines()[-max(1, min(limit, 1000)):]
        return [json.loads(line) for line in lines if line.strip()]
    except (OSError, ValueError, TypeError):
        return []


def persist_calibration(calibration: dict) -> None:
    """Atomically save calibration/settings so a browser refresh cannot see a half-written file."""
    temporary = CALIBRATION_FILE.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(calibration, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(CALIBRATION_FILE)


def normalized_slope_trims(calibration: dict | None = None) -> list[float]:
    data = calibration if calibration is not None else calibration_data()
    raw = data.get("slope_trim_percent", [])
    try:
        values = [float(value) for value in raw]
    except (TypeError, ValueError):
        values = []
    if len(values) != len(MOTOR_IDS) or any(not math.isfinite(value) for value in values):
        return [0.0 for _ in MOTOR_IDS]
    return [max(-20.0, min(20.0, value)) for value in values]


def effective_slopes(calibration: dict | None = None) -> list[float]:
    data = calibration if calibration is not None else calibration_data()
    try:
        raw = [float(value) for value in data["slope"]]
    except (KeyError, TypeError, ValueError):
        return [0.0 for _ in MOTOR_IDS]
    trims = normalized_slope_trims(data)
    return [value * (1.0 + trim / 100.0) for value, trim in zip(raw, trims)]


def trajectory_current_limit(calibration: dict | None = None) -> int:
    data = calibration if calibration is not None else calibration_data()
    try:
        value = int(data.get("trajectory_current", TRAJECTORY_DEFAULT_CURRENT))
    except (TypeError, ValueError):
        value = TRAJECTORY_DEFAULT_CURRENT
    return max(300, min(HARD_CURRENT, value))


def slope_consistency_error(slopes: object) -> str | None:
    if not isinstance(slopes, list) or len(slopes) != len(MOTOR_IDS):
        return "缺少四台电机完整绳长斜率"
    try:
        values = [float(value) for value in slopes]
    except (TypeError, ValueError):
        return "绳长斜率包含无效数字"
    magnitudes = [abs(value) for value in values]
    minimum = min(magnitudes, default=0.0)
    maximum = max(magnitudes, default=0.0)
    if minimum <= 0.05:
        return "四台绳长斜率中存在接近 0 的结果，不能使用"
    ratio = maximum / minimum
    if ratio > MAX_CROSS_MOTOR_SLOPE_RATIO:
        details = "、".join(f"M{index + 1}={value:.4f}°/mm" for index, value in enumerate(values))
        return (
            f"四台绳长斜率不一致：{details}；最大/最小={ratio:.1%}，"
            f"超过 {MAX_CROSS_MOTOR_SLOPE_RATIO - 1:.0%} 允许差异"
        )
    return None


STATE = {
    "auto": "idle",
    "message": "未校准；可先读取状态或做单电机低速测试",
    "corner": 0,
    "point": 0,
    "points": 0,
    "repeat": False,
    "port": None,
    "baud": direct.DEFAULT_BAUD,
    "control_unlocked": False,
    "telemetry": {},
    "direction_ready": {},
    "direction_motor": None,
    "trajectory_hz_target": TRAJECTORY_HZ,
    "trajectory_hz_actual": 0.0,
    "trajectory_frame_bytes": 61,
    "trajectory_frames": 0,
    "trajectory_target_path": [],
    "trajectory_actual_path": [],
    "trajectory_actual_feedback_path": [],
    "trajectory_actual_error_mm": None,
    "process_log_file": str(PROCESS_LOG_FILE),
    "process_log_count": 0,
    "last_process_event": "",
    "calibration": blank_calibration(),
}


def load_calibration() -> None:
    try:
        data = json.loads(CALIBRATION_FILE.read_text(encoding="utf-8"))
        calibration = blank_calibration()
        calibration.update(data)
        raw_reverse = calibration.get("direction_reverse")
        if not isinstance(raw_reverse, list) or len(raw_reverse) != len(MOTOR_IDS):
            calibration["direction_reverse"] = [False for _ in MOTOR_IDS]
        else:
            calibration["direction_reverse"] = [bool(value) for value in raw_reverse]
        raw_position_direction = calibration.get("encoder_positive_direction")
        try:
            position_direction = [int(value) for value in raw_position_direction]
        except (TypeError, ValueError):
            position_direction = []
        calibration["encoder_positive_direction"] = (
            position_direction
            if len(position_direction) == len(MOTOR_IDS) and all(value in (0, 1) for value in position_direction)
            else []
        )
        raw_trims = calibration.get("slope_trim_percent", [])
        try:
            trims = [float(value) for value in raw_trims]
        except (TypeError, ValueError):
            trims = []
        calibration["slope_trim_percent"] = (
            [max(-20.0, min(20.0, value)) for value in trims]
            if len(trims) == len(MOTOR_IDS) and all(math.isfinite(value) for value in trims)
            else [0.0 for _ in MOTOR_IDS]
        )
        for key, default, minimum, maximum in (
            ("strong_current", CALIBRATION_DEFAULT_STRONG_CURRENT, 50, HARD_CURRENT),
            ("weak_current", CALIBRATION_DEFAULT_WEAK_CURRENT, 20, HARD_CURRENT),
            ("trajectory_current", TRAJECTORY_DEFAULT_CURRENT, 300, HARD_CURRENT),
        ):
            try:
                value = int(calibration.get(key, default))
            except (TypeError, ValueError):
                value = default
            calibration[key] = max(minimum, min(maximum, value))
        geometry_error = None
        if calibration.get("valid") and (
            data.get("geometry_version") != 2
            or not isinstance(data.get("workspace_width"), (int, float))
            or not isinstance(data.get("workspace_height"), (int, float))
        ):
            geometry_error = "旧方形校准参数已作废：请按上下 150mm、球径 40mm 重新校准"
            calibration["valid"] = False
        quality_error = geometry_error or (
            slope_consistency_error(effective_slopes(calibration)) if calibration.get("valid") else None
        )
        if quality_error:
            calibration["valid"] = False
            calibration["quality_error"] = quality_error
        else:
            calibration["quality_error"] = ""
        if (calibration.get("valid") and len(calibration.get("slope", [])) == 4) or raw_reverse is not None:
            with STATE_LOCK:
                STATE["calibration"] = calibration
                STATE["message"] = f"已拒绝本地校准参数：{quality_error}" if quality_error else "已加载电脑本地校准参数"
    except (OSError, ValueError, TypeError):
        return


load_calibration()


def json_bytes(value: object) -> bytes:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode("utf-8")


def serial_port_label(port: object) -> dict[str, str]:
    device = str(getattr(port, "device", ""))
    description = str(getattr(port, "description", "") or "")
    return {"device": device, "label": f"{device} - {description}" if description else device}


def telemetry_dict(telemetry: direct.Telemetry) -> dict:
    return {
        "id": telemetry.address,
        "voltage": telemetry.voltage,
        "bus_current": telemetry.bus_current,
        "phase_current": telemetry.phase_current,
        "rpm": telemetry.rpm,
        "position": telemetry.position,
        "temperature": telemetry.temperature,
        "flags": telemetry.flags,
        "flags_text": direct.decode_flags(telemetry.flags) if telemetry.flags is not None else "未响应",
        "online": telemetry.flags is not None,
    }


def state_snapshot() -> dict:
    with STATE_LOCK:
        return copy.deepcopy(STATE)


def set_state(**updates: object) -> None:
    with STATE_LOCK:
        STATE.update(updates)


def update_telemetry(telemetries: list[direct.Telemetry]) -> None:
    with STATE_LOCK:
        previous = STATE.get("telemetry", {})
        merged: dict[str, dict] = {}
        # A lost field must not make the floating dashboard blink to "--".
        # Keep the last known numeric sample, while leaving flags/online false
        # when the current status frame itself was not received.
        value_fields = ("voltage", "bus_current", "phase_current", "rpm", "position", "temperature")
        for telemetry in telemetries:
            current = telemetry_dict(telemetry)
            old = previous.get(str(telemetry.address), {}) if isinstance(previous, dict) else {}
            for field in value_fields:
                if current.get(field) is None and old.get(field) is not None:
                    current[field] = old[field]
            merged[str(telemetry.address)] = current
        STATE["telemetry"] = merged


def active_auto() -> bool:
    return AUTO_THREAD is not None and AUTO_THREAD.is_alive()


def valid_reply(reply: bytes, address: int, command: int, minimum: int = 4) -> bool:
    return direct.valid_reply(reply, address, command, minimum)


def command_result(address: int, command: int, tx: bytes, rx: bytes) -> dict:
    accepted = direct.command_accepted(rx, address, command)
    return {
        "id": address,
        "ok": accepted or bool(rx),
        "accepted": accepted,
        "tx": direct.hex_bytes(tx),
        "rx": direct.hex_bytes(rx),
    }


def firmware_status_result(address: int, tx: bytes, rx: bytes) -> dict:
    valid = valid_reply(rx, address, 0x1A, 4)
    result = {
        "id": address,
        "label": MOTOR_LABELS[address - 1],
        "ok": valid,
        "tx": direct.hex_bytes(tx),
        "rx": direct.hex_bytes(rx),
        "firmware": None,
        "firmware_code": None,
    }
    if valid:
        firmware_code = (rx[2] >> 1) & 0x01
        result["firmware_code"] = firmware_code
        result["firmware"] = "Emm" if firmware_code else "X"
        result["flags"] = rx[2]
    return result


def read_firmware_status(connection: object, address: int) -> dict:
    config_tx = direct.frame(address, 0x42, 0x6C, direct.FRAME_END)
    config_rx = direct.transact(connection, config_tx, first_byte_timeout=0.3)
    result = {
        "id": address,
        "label": MOTOR_LABELS[address - 1],
        "ok": valid_reply(config_rx, address, 0x42, 5),
        "tx": direct.hex_bytes(config_tx),
        "rx": direct.hex_bytes(config_rx),
        "firmware": None,
        "firmware_code": None,
        "detected_by": "0x42 length",
    }
    if result["ok"]:
        if config_rx[2] == 0x25:
            result["firmware_code"] = 0
            result["firmware"] = "X"
            result["detected_by"] = "0x42 length"
            return result
        elif config_rx[2] == 0x21:
            result["firmware_code"] = 1
            result["firmware"] = "Emm"
            result["detected_by"] = "0x42 length"
            return result

    # 0x1A's FwType bit is retained only as a compatibility fallback.  Some
    # Y42 V2.x units report that bit as X even while the active firmware is
    # still Emm, which would make C5/F5 torque commands fail with EE.
    option_tx = direct.frame(address, 0x1A, direct.FRAME_END)
    option_rx = direct.transact(connection, option_tx, first_byte_timeout=0.3)
    fallback = firmware_status_result(address, option_tx, option_rx)
    fallback["fallback_0x42_rx"] = direct.hex_bytes(config_rx)
    if fallback["ok"]:
        fallback["detected_by"] = "0x1A fallback (0x42 unavailable)"
        fallback["message"] = "0x42 配置块不可用，以下固件类型仅作兼容性推断"
        return fallback
    result["message"] = f"无法可靠读取固件类型；0x42 RX={direct.hex_bytes(config_rx)}，0x1A RX={direct.hex_bytes(option_rx)}"
    return result


def firmware_switch_frame(address: int, firmware_code: int = 0, store: bool = True) -> bytes:
    if firmware_code not in (0, 1, 2):
        raise ValueError("固件类型必须是 0=X、1=Emm、2=Emm 狂暴模式")
    return direct.frame(address, 0xD5, 0x69, 0x01 if store else 0x00, firmware_code, direct.FRAME_END)


def firmware_restart_frame(address: int) -> bytes:
    return direct.frame(address, 0x08, 0x97, direct.FRAME_END)


def key_lock_frame(address: int, locked: bool = True, store: bool = True) -> bytes:
    return direct.frame(
        address,
        0xD0,
        0xB3,
        0x01 if store else 0x00,
        0x01 if locked else 0x00,
        direct.FRAME_END,
    )


def key_lock_status(connection: object, address: int) -> dict:
    tx = direct.frame(address, 0x1A, direct.FRAME_END)
    rx = b""
    valid = False
    # The D0 write ACK can arrive before the controller has finished
    # persisting the setting.  A single immediate 0x1A read is therefore
    # occasionally empty on an otherwise healthy RS485 link.
    for attempt in range(4):
        if attempt:
            time.sleep(0.10)
        rx = direct.transact(connection, tx, first_byte_timeout=0.40)
        valid = valid_reply(rx, address, 0x1A, 4)
        if valid:
            break
    return {
        "id": address,
        "ok": valid,
        "tx": direct.hex_bytes(tx),
        "rx": direct.hex_bytes(rx),
        # Y42 V2 returns a reserved/status prefix before the option byte;
        # use the byte immediately before 0x6B so both 4- and 5-byte replies
        # are decoded consistently.
        "locked": bool(rx[-2] & 0x20) if valid else None,
    }


def lock_keys_all(connection: object) -> tuple[bool, list[dict]]:
    results = []
    for address in MOTOR_IDS:
        tx = key_lock_frame(address, locked=True, store=True)
        rx = direct.transact(connection, tx, first_byte_timeout=0.3)
        accepted = direct.command_accepted(rx, address, 0xD0)
        verified = key_lock_status(connection, address)
        results.append({
            "id": address,
            "tx": direct.hex_bytes(tx),
            "rx": direct.hex_bytes(rx),
            "accepted": accepted,
            "verify": verified,
        })
    return all(item["accepted"] and item["verify"].get("locked") is True for item in results), results


def switch_all_to_x(connection: object) -> dict:
    stop_results = stop_all(connection)
    if not all(item["accepted"] for item in stop_results):
        return {"ok": False, "message": "切换 X 固件前四台急停未全部收到 ACK", "stop_results": stop_results}
    results = []
    for address in MOTOR_IDS:
        tx = firmware_switch_frame(address, 0, store=True)
        rx = direct.transact(connection, tx, first_byte_timeout=0.3)
        results.append(command_result(address, 0xD5, tx, rx))
    restart_results = []
    for address in MOTOR_IDS:
        tx = firmware_restart_frame(address)
        rx = direct.transact(connection, tx, first_byte_timeout=0.5)
        restart_results.append(command_result(address, 0x08, tx, rx))
        time.sleep(0.25)
    time.sleep(0.5)
    status = [read_firmware_status(connection, address) for address in MOTOR_IDS]
    accepted = all(item["accepted"] for item in results)
    restarted = all(item["accepted"] for item in restart_results)
    verified = all(item.get("ok") and item.get("firmware_code") == 0 for item in status)
    return {
        "ok": accepted and restarted and verified,
        "accepted": accepted,
        "restarted": restarted,
        "verified": verified,
        "results": results,
        "restart_results": restart_results,
        "status": status,
        "message": "已切换并复核四台 X 固件" if verified else "X 固件切换后复核未全部通过",
    }


def command_frame(address: int, action: str, data: dict) -> bytes:
    if action == "enable":
        return direct.frame(address, 0xF3, 0xAB, 0x01, 0x00, direct.FRAME_END)
    if action == "disable":
        return direct.frame(address, 0xF3, 0xAB, 0x00, 0x00, direct.FRAME_END)
    if action == "stop":
        return direct.frame(address, 0xFE, 0x98, 0x00, direct.FRAME_END)
    if action == "clear":
        return direct.frame(address, 0x0A, 0x6D, direct.FRAME_END)
    if action == "release":
        return direct.frame(address, 0x0E, 0x52, direct.FRAME_END)
    if action == "abort-home":
        return direct.frame(address, 0x9C, 0x48, direct.FRAME_END)
    if action == "set-home":
        return direct.frame(address, 0x93, 0x88, 0x01, direct.FRAME_END)
    if action == "home":
        mode = int(data.get("mode", 0))
        if not 0 <= mode <= 5:
            raise ValueError("回零模式必须是 0..5")
        return direct.frame(address, 0x9A, mode, 0x00, direct.FRAME_END)
    if action == "speed":
        direction = int(data.get("direction", 0))
        rpm = float(data.get("rpm", 0))
        acceleration = int(data.get("acceleration", 100))
        if direction not in (0, 1) or not 0 <= rpm <= 3000 or not 0 <= acceleration <= 65535:
            raise ValueError("速度参数超出范围：方向 0/1，RPM 0..3000，加速度 0..65535")
        return direct.speed_frame(address, direction, rpm, acceleration)
    if action == "torque":
        direction = int(data.get("direction", 0))
        current = int(data.get("current", 100))
        slope = int(data.get("slope", 2000))
        max_rpm = float(data.get("max_rpm", 10))
        return direct.torque_frame(address, direction, current, slope, max_rpm)
    if action == "position":
        degrees = float(data.get("degrees", 0))
        if degrees < 0 or degrees > 429496729.5:
            raise ValueError("角度必须在 0..429496729.5 度")
        return direct.position_frame(
            address,
            int(data.get("direction", 0)),
            float(data.get("rpm", 5)),
            int(data.get("acceleration", 100)),
            degrees,
            int(data.get("mode", 2)),
        )
    raise ValueError(f"未知电机操作: {action}")


def enable_address(connection: object, address: int) -> tuple[bool, bytes, bytes]:
    status = direct.status_query(connection, address)
    if valid_reply(status, address, 0x3A, 4) and (status[2] & 0x01):
        return True, direct.frame(address, 0x3A, direct.FRAME_END), status
    tx = direct.frame(address, 0xF3, 0xAB, 0x01, 0x00, direct.FRAME_END)
    reply = direct.transact(connection, tx, first_byte_timeout=0.3)
    return direct.command_accepted(reply, address, 0xF3), tx, reply


def torque_command(connection: object, address: int, direction: int, current: int) -> tuple[bool, bytes, bytes]:
    """Send an X-firmware torque command with a tolerant ACK/retry path."""

    tx = direct.torque_frame(address, direction, current, 2000, 1000)
    reply = b""
    for attempt in range(3):
        reply = direct.transact(connection, tx, first_byte_timeout=0.3)
        if direct.command_accepted(reply, address, 0xC5):
            return True, tx, reply
        if attempt < 2:
            time.sleep(0.025)
    return False, tx, reply


def stop_accepted(reply: bytes, address: int) -> bool:
    """Accept both generic FE 02 and the Y42 V2 FE E2 stop reply."""

    return direct.command_accepted(reply, address, 0xFE) or (
        valid_reply(reply, address, 0xFE, 4) and reply[2] == 0xE2
    )


def stop_all(connection: object) -> list[dict]:
    results = []
    for address in MOTOR_IDS:
        tx = direct.frame(address, 0xFE, 0x98, 0x00, direct.FRAME_END)
        rx = direct.transact(connection, tx)
        result = command_result(address, 0xFE, tx, rx)
        # Y42 V2 commonly acknowledges the stop frame with FE E2 rather
        # than the generic command ACK byte FE 02.
        result["accepted"] = stop_accepted(rx, address)
        result["ok"] = result["accepted"] or bool(rx)
        results.append(result)
    return results


def disable_all(connection: object) -> list[dict]:
    results = stop_all(connection)
    for address in MOTOR_IDS:
        tx = command_frame(address, "disable", {})
        results.append(command_result(address, 0xF3, tx, direct.transact(connection, tx)))
    return results


def disable_one(connection: object, address: int) -> bool:
    stop_tx = command_frame(address, "stop", {})
    stop_rx = direct.transact(connection, stop_tx)
    disable_tx = command_frame(address, "disable", {})
    disable_rx = direct.transact(connection, disable_tx)
    return stop_accepted(stop_rx, address) and direct.command_accepted(disable_rx, address, 0xF3)


def read_all(
    connection: object,
    *,
    first_byte_timeout: float = 0.06,
    inter_byte_quiet: float = 0.004,
) -> list[direct.Telemetry]:
    values = [
        direct.read_telemetry(
            connection,
            address,
            first_byte_timeout=first_byte_timeout,
            inter_byte_quiet=inter_byte_quiet,
        )
        for address in MOTOR_IDS
    ]
    update_telemetry(values)
    return values


def telemetry_ready_for_calibration(values: list[direct.Telemetry]) -> bool:
    """Require the fields used by calibration to be present for every motor."""

    return len(values) == len(MOTOR_IDS) and all(
        t.position is not None
        and t.rpm is not None
        and t.phase_current is not None
        and t.temperature is not None
        for t in values
    )


def read_all_reliable(
    connection: object,
    attempts: int = 2,
    *,
    first_byte_timeout: float = 0.06,
    inter_byte_quiet: float = 0.004,
) -> list[direct.Telemetry]:
    """Retry a complete telemetry pass when a USB-RS485 frame is lost."""

    last: list[direct.Telemetry] = []
    for attempt in range(attempts):
        last = read_all(
            connection,
            first_byte_timeout=first_byte_timeout,
            inter_byte_quiet=inter_byte_quiet,
        )
        if telemetry_ready_for_calibration(last):
            return last
        if attempt + 1 < attempts:
            time.sleep(0.05)
    return last


def read_link_health(connection: object, *, first_byte_timeout: float = 0.08) -> list[direct.Telemetry]:
    """Probe only the four flag registers before requesting full telemetry.

    When the USB adapter is present but the motor bus is unplugged, asking for
    28 telemetry fields per status refresh makes the web request look frozen.
    A four-frame health probe gives a fast, unambiguous link result first.
    """

    values: list[direct.Telemetry] = []
    for address in MOTOR_IDS:
        telemetry = direct.Telemetry(address)
        reply = direct.transact(
            connection,
            direct.frame(address, 0x3A, direct.FRAME_END),
            first_byte_timeout=first_byte_timeout,
            inter_byte_quiet=0.002,
        )
        if valid_reply(reply, address, 0x3A, 4):
            telemetry.flags = reply[2]
        values.append(telemetry)
    return values


def read_all_with_reconnect(
    port: str,
    baud: int,
    attempts: int = 3,
    *,
    first_byte_timeout: float = 0.06,
    inter_byte_quiet: float = 0.004,
    lock_timeout: float | None = None,
) -> tuple[list[direct.Telemetry], bool, str]:
    """Read all four motors, reopening the USB-RS485 handle after a bad pass.

    A pyserial object can remain open after a USB adapter has reset.  Reusing
    that stale handle makes every following query look like an offline motor,
    so status polling gets a bounded fresh-handle retry path too.
    """

    last: list[direct.Telemetry] = []
    last_error = ""
    for attempt in range(1, attempts + 1):
        attempt_error = ""
        acquired = SERIAL_LOCK.acquire() if lock_timeout is None else SERIAL_LOCK.acquire(timeout=lock_timeout)
        if not acquired:
            return last, False, "串口正被其他自动流程占用，状态读取已快速返回"
        try:
            with direct.open_serial(port, baud) as connection:
                # The real adapter's measured response latency is about
                # 31..44 ms, so a 35 ms probe falsely reports M1 offline.
                # Keep the probe short for a dead bus, but above the observed
                # worst-case latency for a live motor.
                health_timeout = max(0.08, min(first_byte_timeout, 0.12))
                health = read_link_health(connection, first_byte_timeout=health_timeout)
                responsive = sum(telemetry.flags is not None for telemetry in health)
                if responsive == 0:
                    last = health
                    update_telemetry(last)
                    attempt_error = "串口已打开，但 ID 1..4 均无 0x3A 状态回包"
                else:
                    last = read_all_reliable(
                        connection,
                        attempts=2,
                        first_byte_timeout=first_byte_timeout,
                        inter_byte_quiet=inter_byte_quiet,
                    )
                    if responsive < len(MOTOR_IDS):
                        attempt_error = f"健康查询只有 {responsive}/{len(MOTOR_IDS)} 台有回包"
        except Exception as exc:
            attempt_error = str(exc)
        finally:
            SERIAL_LOCK.release()
        if telemetry_ready_for_calibration(last):
            schedule_prepare_if_needed(port, baud)
            return last, True, ""
        last_error = attempt_error or "四台遥测字段不完整"
        if attempt < attempts:
            time.sleep(0.08 * attempt)
    return last, False, last_error or "RS485 无有效回包"


def _cached_status_result(port: str, baud: int, *, message: str | None = None) -> dict:
    with STATUS_CACHE_LOCK:
        cached = copy.deepcopy(STATUS_CACHE.get("result"))
        cached_port = STATUS_CACHE.get("port")
        cached_baud = STATUS_CACHE.get("baud")
    if cached is not None and cached_port == port and cached_baud == baud:
        if message:
            cached["message"] = message
        return cached
    snapshot = state_snapshot()
    return {
        "ok": True,
        "action": "status",
        "motors": list(snapshot.get("telemetry", {}).values()),
        "auto": snapshot.get("auto", "idle"),
        "reconnected": False,
        "message": message or "正在建立串口状态连接",
    }


def status_for_web(port: str, baud: int) -> dict:
    """Return a bounded status response without piling up serial requests."""

    now = time.monotonic()
    with STATUS_CACHE_LOCK:
        age = now - float(STATUS_CACHE.get("at", 0.0))
        cached_port = STATUS_CACHE.get("port")
        cached_baud = STATUS_CACHE.get("baud")
        cached_recovered = bool(STATUS_CACHE.get("recovered"))
        has_cache = STATUS_CACHE.get("result") is not None and cached_port == port and cached_baud == baud
    cache_window = STATUS_SUCCESS_CACHE_SECONDS if cached_recovered else STATUS_FAILURE_CACHE_SECONDS
    if has_cache and age < cache_window:
        return _cached_status_result(port, baud)
    if not STATUS_POLL_LOCK.acquire(blocking=False):
        return _cached_status_result(port, baud, message="状态读取正在进行，已显示上一次结果；不会叠加串口请求")
    try:
        # A second browser request may have filled the cache while this one
        # was waiting for the single-flight guard.
        now = time.monotonic()
        with STATUS_CACHE_LOCK:
            age = now - float(STATUS_CACHE.get("at", 0.0))
            cached_port = STATUS_CACHE.get("port")
            cached_baud = STATUS_CACHE.get("baud")
            cached_recovered = bool(STATUS_CACHE.get("recovered"))
            has_cache = STATUS_CACHE.get("result") is not None and cached_port == port and cached_baud == baud
        cache_window = STATUS_SUCCESS_CACHE_SECONDS if cached_recovered else STATUS_FAILURE_CACHE_SECONDS
        if has_cache and age < cache_window:
            return _cached_status_result(port, baud)

        values, recovered, reason = read_all_with_reconnect(
            port,
            baud,
            attempts=1,
            first_byte_timeout=0.08,
            inter_byte_quiet=0.002,
            lock_timeout=0.05,
        )
        deferred = ("串口正被其他自动流程占用" in reason)
        if not recovered and not deferred:
            set_state(message="状态读取失败：" + reason)
        snapshot = state_snapshot()
        result = {
            "ok": True,
            "action": "status",
            "motors": list(snapshot["telemetry"].values()),
            "auto": snapshot["auto"],
            "reconnected": recovered,
            "status_deferred": deferred,
            "message": "状态读取暂缓：另一个串口操作正在进行，保留上次状态" if deferred else ("四台遥测已自动重连恢复" if recovered else "自动重连未恢复：" + reason),
        }
        with STATUS_CACHE_LOCK:
            STATUS_CACHE.update({"at": time.monotonic(), "port": port, "baud": baud, "recovered": recovered, "result": copy.deepcopy(result)})
        return result
    except Exception as exc:
        reason = str(exc)
        set_state(message="状态读取异常：" + reason)
        result = _cached_status_result(port, baud, message="状态读取异常：" + reason)
        with STATUS_CACHE_LOCK:
            STATUS_CACHE.update({"at": time.monotonic(), "port": port, "baud": baud, "recovered": False, "result": copy.deepcopy(result)})
        return result
    finally:
        STATUS_POLL_LOCK.release()


def disable_all_with_reconnect(port: str, baud: int, attempts: int = 3, announce: bool = True) -> dict:
    """Best-effort safety shutdown that retries with a new serial handle.

    It is intentionally used after a motion/telemetry failure.  We never
    resume motion automatically after a link fault; the reconnect is only to
    send and verify STOP + DISABLE for all four motors.
    """

    last_results: list[dict] = []
    last_error = ""
    for attempt in range(1, attempts + 1):
        if announce:
            set_state(message=f"通讯异常：正在重连并失能四台电机（{attempt}/{attempts}）")
        try:
            with SERIAL_LOCK, direct.open_serial(port, baud) as connection:
                last_results = disable_all(connection)
            if last_results and all(item.get("accepted") for item in last_results):
                return {"ok": True, "attempts": attempt, "results": last_results}
            last_error = "四台 STOP/DISABLE 未全部收到 ACK"
        except Exception as exc:
            last_error = str(exc)
        if attempt < attempts:
            time.sleep(0.12 * attempt)
    return {
        "ok": False,
        "attempts": attempts,
        "results": last_results,
        "error": last_error or "RS485 重连失败",
    }


def stability_test(port: str, baud: int, rounds: int) -> dict:
    if not 1 <= rounds <= 100:
        raise ValueError("稳定性测试轮数必须是 1..100")
    rows = {address: {"valid": 0, "timeout": 0, "bad": 0, "latencies_ms": [], "bad_frames": []} for address in MOTOR_IDS}
    started = time.perf_counter()
    with SERIAL_LOCK, direct.open_serial(port, baud) as connection:
        for _ in range(rounds):
            for address in MOTOR_IDS:
                t0 = time.perf_counter()
                reply = direct.status_query(connection, address)
                latency = (time.perf_counter() - t0) * 1000.0
                row = rows[address]
                if valid_reply(reply, address, 0x3A, 4):
                    row["valid"] += 1
                    row["latencies_ms"].append(latency)
                elif reply:
                    row["bad"] += 1
                    if len(row["bad_frames"]) < 5:
                        row["bad_frames"].append(direct.hex_bytes(reply))
                else:
                    row["timeout"] += 1
            time.sleep(0.02)
    motors = {}
    for address, row in rows.items():
        latencies = row["latencies_ms"]
        motors[str(address)] = {
            "label": MOTOR_LABELS[address - 1],
            "valid": row["valid"],
            "timeout": row["timeout"],
            "bad": row["bad"],
            "success_rate": round(row["valid"] / rounds * 100.0, 1),
            "latency_ms": {
                "min": round(min(latencies), 2) if latencies else None,
                "avg": round(statistics.mean(latencies), 2) if latencies else None,
                "max": round(max(latencies), 2) if latencies else None,
            },
            "bad_frames": row["bad_frames"],
        }
    return {"ok": True, "action": "stability", "port": port, "baud": baud, "rounds": rounds, "elapsed_s": round(time.perf_counter() - started, 3), "motors": motors, "note": "只发送 0x3A 状态查询，不发送使能、速度、力矩或位置指令"}


def baud_sweep(port: str, bauds: list[int]) -> dict:
    results = []
    with SERIAL_LOCK:
        for baud in bauds:
            row = {"baud": baud, "valid": [], "replies": {}}
            try:
                with direct.open_serial(port, baud) as connection:
                    for address in MOTOR_IDS:
                        reply = direct.transact(connection, direct.frame(address, 0x3A, direct.FRAME_END), first_byte_timeout=0.3)
                        if valid_reply(reply, address, 0x3A, 4):
                            row["valid"].append(address)
                        if reply:
                            row["replies"][str(address)] = direct.hex_bytes(reply)
            except Exception as exc:
                row["error"] = str(exc)
            results.append(row)
    return {"ok": True, "action": "baud-sweep", "port": port, "results": results, "note": "每个波特率只向 ID 1..4 发送一次 0x3A 状态查询，不发送运动指令"}


def online(telemetry: direct.Telemetry) -> bool:
    return telemetry.position is not None and telemetry.rpm is not None and telemetry.flags is not None


def safety_error(values: list[direct.Telemetry], current_limit: int = HARD_CURRENT) -> str | None:
    for index, telemetry in enumerate(values):
        if not online(telemetry):
            return f"电机 M{index + 1}/ID{telemetry.address} 通讯中断"
        if telemetry.temperature is not None and telemetry.temperature >= HARD_TEMPERATURE:
            return f"电机 M{index + 1} 温度达到 {telemetry.temperature}°C"
        if telemetry.phase_current is not None and telemetry.phase_current > current_limit:
            return f"电机 M{index + 1} 电流超过 {current_limit}mA"
        if telemetry.flags is not None and telemetry.flags & 0x08:
            return f"电机 M{index + 1} 触发保护"
    return None


def query_position_field(connection: object, index: int) -> tuple[bool, float | None, str]:
    address = MOTOR_IDS[index]
    command = 0x36
    reply = b""
    for attempt in range(2):
        reply = direct.transact(
            connection,
            direct.frame(address, command, direct.FRAME_END),
            first_byte_timeout=0.08,
            inter_byte_quiet=0.004,
        )
        if valid_reply(reply, address, command, 8):
            break
        if attempt == 0:
            time.sleep(0.003)
    if not valid_reply(reply, address, command, 8):
        return False, None, f"轨迹中电机 M{index + 1} 位置连续两次无有效回包"
    raw = direct.read_u32(reply, 3) / 10.0
    position = -raw if reply[2] else raw
    with STATE_LOCK:
        current = STATE["telemetry"].setdefault(str(address), {"id": address})
        current["position"] = position
        current["online"] = True
    return True, position, ""


def query_safety_field(connection: object, index: int, field: int) -> tuple[bool, str]:
    address = MOTOR_IDS[index]
    command = (0x27, 0x39, 0x3A)[field]
    # Keep the low-rate safety poll inside the spare part of the 10 ms bus
    # budget. The trajectory itself remains one 0xAA write per tick.
    reply = b""
    for attempt in range(2):
        reply = direct.transact(
            connection,
            direct.frame(address, command, direct.FRAME_END),
            first_byte_timeout=0.08,
            inter_byte_quiet=0.004,
        )
        minimum = 5 if field < 2 else 4
        if valid_reply(reply, address, command, minimum):
            break
        if attempt == 0:
            time.sleep(0.003)
    minimum = 5 if field < 2 else 4
    if not valid_reply(reply, address, command, minimum):
        return False, f"轨迹中电机 M{index + 1} 安全回读连续两次无有效回包"
    with STATE_LOCK:
        current = STATE["telemetry"].setdefault(str(address), {"id": address})
        if field == 0:
            current["phase_current"] = direct.read_u16(reply, 2)
            if current["phase_current"] > HARD_CURRENT:
                return False, f"轨迹中电机 M{index + 1} 电流超过 {HARD_CURRENT}mA"
        elif field == 1:
            current["temperature"] = reply[3]
            if current["temperature"] >= HARD_TEMPERATURE:
                return False, f"轨迹中电机 M{index + 1} 温度达到 {current['temperature']}°C"
        else:
            current["flags"] = reply[2]
            current["flags_text"] = direct.decode_flags(reply[2])
            if reply[2] & 0x08:
                return False, f"轨迹中电机 M{index + 1} 触发保护"
    return True, ""


def calibration_data() -> dict:
    with STATE_LOCK:
        return copy.deepcopy(STATE["calibration"])


def save_calibration_settings(data: dict) -> dict:
    """Persist geometry/current/slope trims without starting a motor action."""
    calibration = calibration_data()
    try:
        side = float(data.get("side", calibration.get("side", 150.0)))
        ball = float(data.get("ball", calibration.get("ball", 40.0)))
        strong = int(data.get("strong", calibration.get("strong_current", CALIBRATION_DEFAULT_STRONG_CURRENT)))
        weak = int(data.get("weak", calibration.get("weak_current", CALIBRATION_DEFAULT_WEAK_CURRENT)))
        trajectory_current = int(data.get("trajectory_current", calibration.get("trajectory_current", TRAJECTORY_DEFAULT_CURRENT)))
        raw_trims = data.get("slope_trim_percent", calibration.get("slope_trim_percent", [0.0] * len(MOTOR_IDS)))
        trims = [float(value) for value in raw_trims]
    except (TypeError, ValueError) as exc:
        raise ValueError("校准参数必须是有效数字") from exc
    width = side - ball
    if not 50.0 <= side <= 1000.0 or not 1.0 <= ball < side or width < 50.0:
        raise ValueError("几何参数范围无效：上下中心距 50..1000mm、小球直径 1..上下中心距、左右工作区至少 50mm")
    if not 50 <= strong <= HARD_CURRENT or not 20 <= weak < strong:
        raise ValueError("校准电流范围无效：强电流 50..3000mA，张力电流 20..强电流")
    if not 300 <= trajectory_current <= HARD_CURRENT:
        raise ValueError("轨迹位置电流上限必须在 300..3000mA")
    if len(trims) != len(MOTOR_IDS) or any(not math.isfinite(value) or not -20.0 <= value <= 20.0 for value in trims):
        raise ValueError("四台斜率微调必须分别填写 -20%..+20%")
    old_side = float(calibration.get("side", side))
    old_ball = float(calibration.get("ball", ball))
    geometry_changed = abs(old_side - side) > 0.0001 or abs(old_ball - ball) > 0.0001
    effective = [value * (1.0 + trim / 100.0) for value, trim in zip(calibration.get("slope", []), trims)]
    if calibration.get("valid") and slope_consistency_error(effective):
        raise ValueError("斜率微调后四台系数差异过大，请把微调控制在相近范围内")
    calibration.update(
        {
            "side": side,
            "ball": ball,
            "effective_side": side,
            "workspace_width": width,
            "workspace_height": side,
            "geometry_version": 2,
            "strong_current": strong,
            "weak_current": weak,
            "trajectory_current": trajectory_current,
            "slope_trim_percent": trims,
        }
    )
    if geometry_changed and calibration.get("valid"):
        calibration["valid"] = False
        calibration["quality_error"] = "几何尺寸已修改，请用新尺寸重新完成四角校准"
    with STATE_LOCK:
        STATE["calibration"] = calibration
        STATE["message"] = "校准参数已保存；几何尺寸变化时需要重新做四角校准" if geometry_changed else "校准参数已保存"
    persist_calibration(calibration)
    append_process_log(
        "calibration_settings_saved",
        side=side,
        ball=ball,
        strong_current=strong,
        weak_current=weak,
        trajectory_current=trajectory_current,
        slope_trim_percent=trims,
        geometry_changed=geometry_changed,
    )
    return {
        "ok": True,
        "action": "calibration-settings",
        "calibration": calibration,
        "message": "校准参数已保存" + ("；几何尺寸已变化，请重新校准" if geometry_changed else ""),
    }


def direction_reversal_flags(calibration: dict | None = None) -> tuple[bool, ...]:
    data = calibration if calibration is not None else calibration_data()
    raw = data.get("direction_reverse", [])
    if not isinstance(raw, list) or len(raw) != len(MOTOR_IDS):
        return tuple(False for _ in MOTOR_IDS)
    return tuple(bool(value) for value in raw)


def pull_directions() -> tuple[int, ...]:
    return tuple(
        int(base) ^ int(reversed_direction)
        for base, reversed_direction in zip(PULL_DIRECTIONS, direction_reversal_flags())
    )


def encoder_positive_directions(calibration: dict | None = None) -> tuple[int, ...]:
    """Return the driver direction bit that increases each encoder position.

    Prefer the mapping learned by the direction test.  Older calibration files
    do not contain it, so derive it from the fitted rope-length slope: the
    pull direction shortens the rope, hence its encoder sign is opposite to
    the slope sign.
    """
    data = calibration if calibration is not None else calibration_data()
    raw = data.get("encoder_positive_direction")
    if isinstance(raw, list) and len(raw) == len(MOTOR_IDS):
        try:
            values = tuple(int(value) for value in raw)
            if all(value in (0, 1) for value in values):
                return values
        except (TypeError, ValueError):
            pass
    slopes = data.get("slope", [])
    pull = pull_directions() if calibration is None else tuple(
        int(base) ^ int(reversed_direction)
        for base, reversed_direction in zip(PULL_DIRECTIONS, direction_reversal_flags(data))
    )
    result = []
    for index, direction in enumerate(pull):
        try:
            slope = float(slopes[index])
        except (IndexError, TypeError, ValueError):
            slope = 0.0
        if abs(slope) < 0.05:
            result.append(direction)
            continue
        encoder_sign_for_pull = -1 if slope > 0 else 1
        result.append(direction if encoder_sign_for_pull > 0 else direction ^ 1)
    return tuple(result)


def position_direction_for_delta(index: int, delta: float) -> int:
    positive_direction = encoder_positive_directions()[index]
    return positive_direction if delta >= 0 else positive_direction ^ 1


def absolute_position_direction(index: int, target: float) -> int:
    """Encode an absolute target in the installed driver's coordinate frame.

    The calibration position is the signed encoder coordinate, while X
    position mode 1/AA-CB interprets ``direction + abs(angle)`` in the
    driver's coordinate frame.  For a motor whose encoder-positive driver bit
    is 1 (the saved M3/M4 mapping), that coordinate is mirrored.
    """

    positive_direction = encoder_positive_directions()[index]
    driver_target = target if positive_direction == 0 else -target
    return 1 if driver_target < 0.0 else 0


def workspace_dimensions(calibration: dict | None = None) -> tuple[float, float]:
    """Return the calibrated ball-center rectangle as (width, height).

    Older calibration files only contain ``effective_side`` and are kept
    readable as square workspaces until the user recalibrates with the real
    rectangular geometry.
    """

    data = calibration if calibration is not None else calibration_data()
    try:
        width = float(data.get("workspace_width"))
        height = float(data.get("workspace_height"))
        if width > 0.0 and height > 0.0:
            return width, height
    except (TypeError, ValueError):
        pass
    side = float(data.get("effective_side", 140.0))
    return side, side


def anchors(width: float, height: float) -> tuple[tuple[float, ...], tuple[float, ...]]:
    return (0.0, width, width, 0.0), (height, height, 0.0, 0.0)


def motor_degrees(index: int, x: float, y: float) -> float:
    calibration = calibration_data()
    width, height = workspace_dimensions(calibration)
    ax, ay = anchors(width, height)
    length = max(0.0, math.hypot(x - ax[index], y - ay[index]) - TRAJECTORY_PRETENSION_MM)
    slopes = effective_slopes(calibration)
    return float(calibration["intercept"][index]) + slopes[index] * length


def position_to_xy(positions: list[float]) -> tuple[float, float, float] | None:
    """Estimate the ball center from four signed encoder positions.

    Calibration converts each encoder position back to a rope length. Three
    linearized circle-difference equations then give an overdetermined XY
    estimate; the returned RMS is useful for showing whether the live path is
    internally consistent.
    """

    if len(positions) != len(MOTOR_IDS):
        return None
    calibration = calibration_data()
    try:
        slopes = effective_slopes(calibration)
        intercepts = [float(value) for value in calibration["intercept"]]
        lengths = [
            (float(position) - intercepts[index]) / slopes[index] + TRAJECTORY_PRETENSION_MM
            for index, position in enumerate(positions)
        ]
    except (KeyError, IndexError, TypeError, ValueError, ZeroDivisionError):
        return None
    if any(not math.isfinite(length) or length <= 0.0 for length in lengths):
        return None
    width, height = workspace_dimensions(calibration)
    ax, ay = anchors(width, height)
    ref_x, ref_y, ref_length = ax[0], ay[0], lengths[0]
    rows = []
    values = []
    for index in range(1, len(MOTOR_IDS)):
        rows.append((2.0 * (ax[index] - ref_x), 2.0 * (ay[index] - ref_y)))
        values.append(
            ref_length * ref_length
            - lengths[index] * lengths[index]
            + ax[index] * ax[index]
            + ay[index] * ay[index]
            - ref_x * ref_x
            - ref_y * ref_y
        )
    ata00 = sum(row[0] * row[0] for row in rows)
    ata01 = sum(row[0] * row[1] for row in rows)
    ata11 = sum(row[1] * row[1] for row in rows)
    atb0 = sum(row[0] * value for row, value in zip(rows, values))
    atb1 = sum(row[1] * value for row, value in zip(rows, values))
    determinant = ata00 * ata11 - ata01 * ata01
    if abs(determinant) < 0.000001:
        return None
    x = (atb0 * ata11 - atb1 * ata01) / determinant
    y = (ata00 * atb1 - ata01 * atb0) / determinant
    residuals = [math.hypot(x - ax[index], y - ay[index]) - lengths[index] for index in range(4)]
    rms = math.sqrt(sum(value * value for value in residuals) / len(residuals))
    return x, y, rms


def append_trajectory_trace(name: str, point: tuple[float, float]) -> None:
    with STATE_LOCK:
        trace = list(STATE.get(name, []))
        trace.append([round(float(point[0]), 2), round(float(point[1]), 2)])
        if len(trace) > TRAJECTORY_TRACE_MAX_POINTS:
            trace = trace[::2]
        STATE[name] = trace


def limited_position_frame(address: int, direction: int, degrees: float) -> bytes:
    """Build an absolute target using the installed motor's calibrated sign."""

    packet = bytearray((address, 0xCD, direction))
    packet += direct.u16(1000) + direct.u16(1000) + direct.u16(4000)
    packet += direct.u32(round(abs(degrees) * 10.0))
    packet += bytes((0x01, 0x01)) + direct.u16(2000) + bytes((direct.FRAME_END,))
    return bytes(packet)


def four_motor_position_frame(
    targets: list[float], rpms: list[float] | None = None, current: int | None = None
) -> bytes:
    """Build one 0xAA absolute-position frame for M1..M4 together."""

    if len(targets) != len(MOTOR_IDS):
        raise ValueError("四电机合帧必须包含 4 个目标位置")
    speeds = rpms if rpms is not None else [1000.0 for _ in MOTOR_IDS]
    if len(speeds) != len(MOTOR_IDS):
        raise ValueError("四电机合帧必须包含 4 个目标转速")
    current_limit = trajectory_current_limit() if current is None else max(300, min(HARD_CURRENT, int(current)))
    packet = bytearray((0x00, 0xAA, 0x00, 0x00))
    for index, address in enumerate(MOTOR_IDS):
        rpm = max(1.0, min(TRAJECTORY_MAX_RPM, float(speeds[index])))
        target = float(targets[index])
        packet += bytes((address, 0xCB, absolute_position_direction(index, target)))
        packet += direct.u16(round(rpm * 10.0)) + direct.u32(round(abs(target) * 10.0))
        packet += bytes((0x01, 0x00)) + direct.u16(current_limit) + bytes((direct.FRAME_END,))
    length = len(packet) - 4 + 3
    packet[2] = (length >> 8) & 0xFF
    packet[3] = length & 0xFF
    packet.append(direct.FRAME_END)
    if len(packet) != 61:
        raise RuntimeError(f"四电机 0xAA 合帧长度异常: {len(packet)} 字节")
    return bytes(packet)


def trajectory_frame(x: float, y: float, next_x: float, next_y: float, lookahead: float) -> bytes:
    """Build one 0xAA frame containing all four motor targets.

    The frame is written once per control tick.  Do not replace this with
    four 0xCB writes: that introduces up to one control tick of skew between
    the first and last motor.
    """
    targets = []
    speeds = []
    for index, address in enumerate(MOTOR_IDS):
        current_target = motor_degrees(index, x, y)
        target = motor_degrees(index, next_x, next_y)
        rpm = abs(target - current_target) / (lookahead * 6.0) if lookahead > 0.0001 else 1.0
        rpm = max(1.0, min(TRAJECTORY_MAX_RPM, rpm * TRAJECTORY_FEEDFORWARD))
        targets.append(target)
        speeds.append(rpm)
    return four_motor_position_frame(targets, speeds)


def send_fast(connection: object, packet: bytes) -> None:
    """Write one already-packed frame without waiting for an application ACK."""
    direction_mode = getattr(connection, "_y42_direction_mode", "auto")
    if direction_mode == "rts-high":
        connection.rts = True
    elif direction_mode == "rts-low":
        connection.rts = False
    connection.reset_input_buffer()
    connection.write(packet)
    connection.flush()
    if direction_mode in {"rts-high", "rts-low"}:
        baudrate = float(getattr(connection, "baudrate", direct.DEFAULT_BAUD) or direct.DEFAULT_BAUD)
        time.sleep(max(0.001, len(packet) * 10.0 / baudrate))
        connection.rts = direction_mode == "rts-low"
    deadline = time.monotonic() + 0.001
    while time.monotonic() < deadline:
        waiting = getattr(connection, "in_waiting", 0)
        if waiting:
            connection.read(waiting)
        else:
            time.sleep(0.0002)


def chaikin_path(vertices: list[tuple[float, float]], *, closed: bool, passes: int = 2) -> list[tuple[float, float]]:
    """Round polygon/polyline corners without overshooting the calibrated rectangle."""
    current = list(vertices)
    if len(current) < 2:
        return current
    for _ in range(max(1, passes)):
        if closed:
            unique = current[:-1] if current[0] == current[-1] else current
            rounded: list[tuple[float, float]] = []
            for index, point in enumerate(unique):
                following = unique[(index + 1) % len(unique)]
                rounded.append((point[0] * 0.75 + following[0] * 0.25, point[1] * 0.75 + following[1] * 0.25))
                rounded.append((point[0] * 0.25 + following[0] * 0.75, point[1] * 0.25 + following[1] * 0.75))
            current = rounded + [rounded[0]]
        else:
            rounded = [current[0]]
            for first, second in zip(current, current[1:]):
                rounded.extend(
                    (
                        (first[0] * 0.75 + second[0] * 0.25, first[1] * 0.75 + second[1] * 0.25),
                        (first[0] * 0.25 + second[0] * 0.75, first[1] * 0.25 + second[1] * 0.75),
                    )
                )
            rounded.append(current[-1])
            current = rounded
    return current


def path_with_cumulative(path: list[tuple[float, float]]) -> tuple[list[tuple[float, float]], list[float]]:
    cumulative = [0.0]
    for first, second in zip(path, path[1:]):
        cumulative.append(cumulative[-1] + math.hypot(second[0] - first[0], second[1] - first[1]))
    return path, cumulative


def sample_cumulative_path(path: list[tuple[float, float]], cumulative: list[float], u: float) -> tuple[float, float]:
    if not path:
        return 0.0, 0.0
    if len(path) == 1 or not cumulative or cumulative[-1] <= 0.0001:
        return tuple(path[0])
    target = max(0.0, min(1.0, u)) * cumulative[-1]
    high = 1
    while high < len(cumulative) and cumulative[high] < target:
        high += 1
    if high >= len(path):
        return tuple(path[-1])
    low = high - 1
    span = cumulative[high] - cumulative[low]
    fraction = (target - cumulative[low]) / span if span > 0.0001 else 0.0
    return (
        path[low][0] + (path[high][0] - path[low][0]) * fraction,
        path[low][1] + (path[high][1] - path[low][1]) * fraction,
    )


def sample_path(kind: str, u: float, spec: dict) -> tuple[float, float]:
    u = max(0.0, min(1.0, u))
    width = float(spec.get("width", spec.get("side", 140.0)))
    height = float(spec.get("height", spec.get("side", 140.0)))
    center_x = width / 2.0
    center_y = height / 2.0
    if kind == "circle":
        angle = 2.0 * math.pi * u
        radius = min(width, height) * 0.40
        return center_x + radius * math.cos(angle), center_y + radius * math.sin(angle)
    if kind in {"triangle", "rectangle", "cross", "custom"} and spec.get("path") and spec.get("cumulative"):
        return sample_cumulative_path(spec["path"], spec["cumulative"], u)
    if kind == "triangle":
        vertices = ((center_x, height * 0.10), (width * 0.90, height * 0.85), (width * 0.10, height * 0.85))
        q = min(2.999999, u * 3.0)
        segment = int(q)
        fraction = q - segment
        a = vertices[segment]
        b = vertices[(segment + 1) % 3]
        return a[0] + (b[0] - a[0]) * fraction, a[1] + (b[1] - a[1]) * fraction
    if kind in {"rectangle", "cross"}:
        if kind == "rectangle":
            vertices = (
                (width * 0.18, height * 0.18),
                (width * 0.82, height * 0.18),
                (width * 0.82, height * 0.82),
                (width * 0.18, height * 0.82),
                (width * 0.18, height * 0.18),
            )
        else:
            vertices = (
                (width * 0.18, height * 0.18),
                (width * 0.82, height * 0.82),
                (center_x, center_y),
                (width * 0.82, height * 0.18),
                (width * 0.18, height * 0.82),
            )
        q = min(float(len(vertices) - 1) - 0.000001, u * (len(vertices) - 1))
        segment = int(q)
        fraction = q - segment
        a = vertices[segment]
        b = vertices[segment + 1]
        return a[0] + (b[0] - a[0]) * fraction, a[1] + (b[1] - a[1]) * fraction
    return sample_cumulative_path(spec["path"], spec["cumulative"], u)


def draw_targets_reached(values: list[direct.Telemetry], targets: list[float]) -> bool:
    return all(
        t.position is not None
        and t.rpm is not None
        and abs(t.position - target) <= TRAJECTORY_POSITION_TOLERANCE
        and abs(t.rpm) <= 15.0
        for t, target in zip(values, targets)
    )


def draw_targets_near_and_stopped(values: list[direct.Telemetry], targets: list[float]) -> bool:
    """Return true only for a small, bounded residual error with no motion.

    This is a hand-off condition, not a replacement for the normal 8°
    reached check.  It covers the common closed-loop case where one motor
    reports zero RPM a little short of the absolute target after the cable
    tension settles.  A large error still times out and triggers the normal
    safe shutdown.
    """
    return len(values) == len(targets) == len(MOTOR_IDS) and all(
        t.position is not None
        and t.rpm is not None
        and abs(t.position - target) <= TRAJECTORY_NEAR_POSITION_TOLERANCE
        and abs(t.rpm) <= 15.0
        for t, target in zip(values, targets)
    )


def max_target_error(values: list[direct.Telemetry], targets: list[float]) -> float | None:
    errors = [abs(t.position - target) for t, target in zip(values, targets) if t.position is not None]
    return max(errors) if len(errors) == len(targets) else None


def validate_trajectory_bounds(kind: str, spec: dict) -> tuple[float, float]:
    width = float(spec.get("width", spec.get("side", 140.0)))
    height = float(spec.get("height", spec.get("side", 140.0)))
    if width <= 0.0 or height <= 0.0:
        raise RuntimeError("轨迹工作区尺寸无效")
    for sample_index in range(101):
        x, y = sample_path(kind, sample_index / 100.0, spec)
        if not 0.0 <= x <= width or not 0.0 <= y <= height:
            raise RuntimeError(
                f"{kind_label(kind)}轨迹超出工作区 {width:.1f}×{height:.1f}mm：({x:.1f},{y:.1f})mm"
            )
    return width, height


def quintic_ease(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * value * (value * (value * 6.0 - 15.0) + 10.0)


def move_to_start_smooth(
    connection: object,
    current_positions: list[float],
    targets: list[float],
    label: str,
) -> None:
    """Ramp all four absolute targets together before checking the start point.

    A single large AA target makes the driver choose a sharp acceleration
    profile and is the main reason a following test shape can appear stuck at
    its start.  This sends a bounded, synchronized ramp instead.
    """
    if len(current_positions) != len(MOTOR_IDS) or len(targets) != len(MOTOR_IDS):
        raise RuntimeError("轨迹起点需要四台电机的完整位置")
    max_delta = max(abs(target - current) for current, target in zip(current_positions, targets))
    # Leave headroom for the quintic ramp's peak velocity.  The old formula
    # commanded the first frame at 140 RPM and could still undershoot the
    # later absolute target, which felt like a sharp tug followed by a catch-up.
    duration = max(1.00, min(12.0, max_delta / max(TRAJECTORY_START_RPM * 6.0, 1.0) * 1.80))
    steps = max(4, int(math.ceil(duration / TRAJECTORY_START_TICK)))
    started = time.monotonic()
    append_process_log(
        "trajectory_start_ramp",
        shape=label,
        current_positions=current_positions,
        target_positions=targets,
        max_delta_deg=round(max_delta, 2),
        duration_s=round(duration, 3),
        steps=steps,
    )
    for step in range(1, steps + 1):
        if STOP_EVENT.is_set():
            raise RuntimeError("用户急停")
        phase = quintic_ease(step / steps)
        ramp_targets = [current + (target - current) * phase for current, target in zip(current_positions, targets)]
        ramp_rpm = TRAJECTORY_START_RPM * 1.10
        send_fast(connection, four_motor_position_frame(ramp_targets, [ramp_rpm] * len(MOTOR_IDS)))
        deadline = started + step * TRAJECTORY_START_TICK
        wait = deadline - time.monotonic()
        if wait > 0.0:
            time.sleep(wait)
    # Reassert the exact endpoint once, at the same bounded speed, then let
    # the normal feedback settle check decide whether it is really reached.
    send_fast(connection, four_motor_position_frame(targets, [TRAJECTORY_START_RPM] * len(MOTOR_IDS)))
    append_process_log("trajectory_start_ramp_complete", shape=label, elapsed_s=round(time.monotonic() - started, 3))


def compute_calibration(samples: list[list[float]], width: float, height: float, ball: float) -> tuple[dict | None, str]:
    ax, ay = anchors(width, height)
    previous = calibration_data()
    slope = []
    intercept = []
    r2 = []
    groups = []
    spread = []
    for motor in range(4):
        own_intercept = samples[motor][motor]
        local = []
        numerator = 0.0
        denominator = 0.0
        for corner in range(4):
            if corner == motor:
                continue
            length = math.hypot(ax[corner] - ax[motor], ay[corner] - ay[motor])
            value = (samples[corner][motor] - own_intercept) / length
            local.append(value)
            numerator += length * (samples[corner][motor] - own_intercept)
            denominator += length * length
        fitted = numerator / denominator if denominator else 0.0
        mean = sum(samples[corner][motor] for corner in range(4)) / 4.0
        ss_total = sum((samples[corner][motor] - mean) ** 2 for corner in range(4))
        ss_residual = sum(
            (
                samples[corner][motor]
                - (own_intercept + fitted * math.hypot(ax[corner] - ax[motor], ay[corner] - ay[motor]))
            )
            ** 2
            for corner in range(4)
        )
        score = 1.0 - ss_residual / ss_total if ss_total > 0.001 else 0.0
        max_spread = max((abs(value - fitted) for value in local), default=0.0)
        intercept.append(own_intercept)
        slope.append(fitted)
        r2.append(score)
        groups.append(local)
        spread.append(max_spread)
        signs_agree = all(value * local[0] > 0 for value in local[1:]) if local else False
        if abs(fitted) < 0.05 or not signs_agree or score < 0.85 or max_spread > MAX_SLOPE_SPREAD:
            return None, f"电机 M{motor + 1} 拟合质量不足：R²={score:.3f}，系数偏差={max_spread:.3f}°/mm"
    quality_error = slope_consistency_error(slope)
    if quality_error:
        return None, quality_error + "；请优先检查 M3/M4 绳路、绕线方向、编码器零点和角点采样，不保存本次校准"
    raw_slope = list(slope)
    common_slope_abs = sum(abs(value) for value in raw_slope) / len(raw_slope)
    # The four motors use one common angle-per-mm magnitude.  Their signs are
    # installation directions, so keep those signs while averaging only the
    # physical coefficient magnitude.  Encoder zero offsets remain individual.
    slope = [math.copysign(common_slope_abs, value) for value in raw_slope]
    common_r2 = []
    for motor in range(4):
        own_intercept = intercept[motor]
        mean = sum(samples[corner][motor] for corner in range(4)) / 4.0
        ss_total = sum((samples[corner][motor] - mean) ** 2 for corner in range(4))
        ss_residual = sum(
            (
                samples[corner][motor]
                - (own_intercept + slope[motor] * math.hypot(ax[corner] - ax[motor], ay[corner] - ay[motor]))
            )
            ** 2
            for corner in range(4)
        )
        common_r2.append(1.0 - ss_residual / ss_total if ss_total > 0.001 else 0.0)
    if any(score < 0.85 for score in common_r2):
        return None, f"公共斜率拟合质量不足：R²={min(common_r2):.3f}；请检查四角采样，不保存本次校准"
    r2 = common_r2
    return {
        "valid": True,
        "side": height,
        "ball": ball,
        "effective_side": height,
        "workspace_width": width,
        "workspace_height": height,
        "geometry_version": 2,
        "slope": slope,
        "raw_slope": raw_slope,
        "common_slope_abs": common_slope_abs,
        "intercept": intercept,
        "r2": r2,
        "groups": groups,
        "spread": spread,
        "samples": samples,
        "direction_reverse": list(direction_reversal_flags()),
        "encoder_positive_direction": list(
            encoder_positive_directions({"slope": slope, "direction_reverse": list(direction_reversal_flags())})
        ),
        "strong_current": int(previous.get("strong_current", CALIBRATION_DEFAULT_STRONG_CURRENT)),
        "weak_current": int(previous.get("weak_current", CALIBRATION_DEFAULT_WEAK_CURRENT)),
        "trajectory_current": int(previous.get("trajectory_current", TRAJECTORY_DEFAULT_CURRENT)),
        "slope_trim_percent": normalized_slope_trims(previous),
        "quality_error": "",
    }, ""


def read_motor_position(connection: object, address: int) -> float | None:
    reply = direct.transact(connection, direct.frame(address, 0x36, direct.FRAME_END))
    if not valid_reply(reply, address, 0x36, 8):
        return None
    value = direct.read_u32(reply, 3) / 10.0
    return -value if reply[2] else value


def relative_position_command(connection: object, address: int, direction: int, degrees: float = DIRECTION_TEST_DEGREES) -> tuple[bytes, bytes]:
    """Send a bounded relative move for the direction check.

    X firmware mode 2 means "relative to the current real-time position".
    It must receive the move amount (45 degrees), never the encoder's absolute
    position value.  Keeping this helper relative makes the test independent
    of the encoder's accumulated angle and avoids any reset move.
    """
    if degrees != DIRECTION_TEST_DEGREES:
        raise ValueError("正方向测试只允许发送 45° 相对位移")
    tx = direct.position_frame(address, direction, DIRECTION_TEST_RPM, 100, degrees, 2)
    reply = direct.transact(connection, tx, first_byte_timeout=0.3)
    if not direct.command_accepted(reply, address, 0xFD):
        detail = direct.hex_bytes(reply) if reply else "超时无回包"
        raise RuntimeError(f"电机 ID {address} 位置命令未确认，RX={detail}")
    return tx, reply


def wait_motor_travel(
    connection: object,
    address: int,
    origin: float,
    timeout: float = DIRECTION_TEST_TIMEOUT,
) -> float:
    """Wait for 45° of actual encoder travel without assuming its sign.

    The driver direction bit (CW/CCW) is not guaranteed to match the encoder
    position sign after a motor is installed with a different cable/winding
    orientation.  Direction confirmation is a physical check, so both +45°
    and -45° are valid observations; only the travel magnitude is bounded.
    """
    started = time.monotonic()
    last: float | None = None
    while time.monotonic() - started < timeout:
        if STOP_EVENT.is_set():
            raise RuntimeError("用户急停")
        for field in range(3):
            ok, reason = query_safety_field(connection, MOTOR_IDS.index(address), field)
            if not ok:
                raise RuntimeError(reason)
        current = read_motor_position(connection, address)
        if current is not None:
            last = current
            travel = abs(current - origin)
            if travel > DIRECTION_TEST_MAX_TRAVEL:
                raise RuntimeError(
                    f"电机 ID {address} 单次位移已达 {travel:.1f}°，超过 {DIRECTION_TEST_DEGREES:.0f}°测试上限，已停止"
                )
            if abs(travel - DIRECTION_TEST_DEGREES) <= DIRECTION_TEST_TOLERANCE or travel >= DIRECTION_TEST_DEGREES - DIRECTION_TEST_TOLERANCE:
                return current
        time.sleep(0.08)
    suffix = "" if last is None else f"，最后位置 {last:.1f}°"
    raise RuntimeError(
        f"电机 ID {address} 未在 {timeout:.0f} 秒内完成 {DIRECTION_TEST_DEGREES:.0f}° 实际位移{suffix}"
    )


def direction_test_worker(address: int) -> None:
    index = MOTOR_IDS.index(address)
    direction = pull_directions()[index]
    error: str | None = None
    try:
        set_state(auto="direction-test", direction_motor=address, message=f"M{index + 1} 正方向测试：检查 X 固件并读取原位置")
        with SERIAL_LOCK, direct.open_serial(current_port(), current_baud()) as connection:
            cleared = disable_all(connection)
            if not all(item["accepted"] for item in cleared):
                raise RuntimeError("方向测试启动前四台电机未能全部失能，已取消运动")
            firmware = read_firmware_status(connection, address)
            if firmware.get("firmware") != "X":
                raise RuntimeError(f"M{index + 1}/ID{address} 不是可确认的 X 固件，已取消运动")
            enabled, _, _ = enable_address(connection, address)
            if not enabled:
                raise RuntimeError(f"M{index + 1}/ID{address} 自动使能失败")
            original = read_motor_position(connection, address)
            if original is None:
                raise RuntimeError(f"M{index + 1}/ID{address} 读取当前位置失败，已取消运动")
            set_state(message=f"M{index + 1} 正方向测试：驱动器方向 {direction}，仅发送 {DIRECTION_TEST_DEGREES:.0f}° 相对位移")
            relative_position_command(connection, address, direction)
            observed = wait_motor_travel(connection, address, original)
            delta = observed - original
            set_state(message=f"M{index + 1} 已完成 {DIRECTION_TEST_DEGREES:.0f}° 测试（实际位置变化 {delta:+.1f}°），保持当前位置并失能")
            if not disable_one(connection, address):
                raise RuntimeError(f"M{index + 1}/ID{address} 测试后失能未收到 ACK")
            with STATE_LOCK:
                ready = copy.deepcopy(STATE.get("direction_ready", {}))
                ready[str(address)] = {
                    "id": address,
                    "direction": direction,
                    "original": round(original, 1),
                    "target": round(observed, 1),
                    "delta": round(delta, 1),
                    "completed": True,
                }
                STATE["direction_ready"] = ready
            set_state(auto="idle", direction_motor=None, message=f"M{index + 1} 已转 45° 并停在当前位置；请选择方向后保存")
    except Exception as exc:
        error = str(exc)
        set_state(auto="error", direction_motor=address, message=f"M{index + 1} 正方向测试失败：{error}")
    finally:
        if error or STOP_EVENT.is_set():
            recovery = disable_all_with_reconnect(current_port(), current_baud(), announce=True)
            if not recovery["ok"]:
                set_state(
                    auto="error",
                    message=f"M{index + 1} 测试异常，RS485 重连后仍无法确认四台失能；请立即切断电机动力或使用硬件急停（{recovery['error']}）",
                )
        if STOP_EVENT.is_set() and not error:
            set_state(auto="idle", message="已停止，电机已发送急停")
        STOP_EVENT.clear()


def save_direction_choice(address: int, reverse: bool) -> dict:
    with STATE_LOCK:
        ready = copy.deepcopy(STATE.get("direction_ready", {})).get(str(address))
        calibration = copy.deepcopy(STATE["calibration"])
    if not ready or not ready.get("completed"):
        raise RuntimeError(f"请先完成 M{MOTOR_IDS.index(address) + 1} 的 45°正方向测试")
    reversals = list(direction_reversal_flags(calibration))
    if reverse:
        reversals[MOTOR_IDS.index(address)] = not reversals[MOTOR_IDS.index(address)]
    calibration["direction_reverse"] = reversals
    position_directions = list(encoder_positive_directions(calibration))
    try:
        tested_direction = int(ready.get("direction"))
        observed_delta = float(ready.get("delta"))
        if tested_direction in (0, 1) and abs(observed_delta) >= 1.0:
            position_directions[MOTOR_IDS.index(address)] = tested_direction if observed_delta > 0 else tested_direction ^ 1
    except (TypeError, ValueError):
        pass
    calibration["encoder_positive_direction"] = position_directions
    with STATE_LOCK:
        STATE["calibration"] = calibration
        STATE["direction_ready"] = {
            key: value for key, value in STATE.get("direction_ready", {}).items() if key != str(address)
        }
    chosen = pull_directions()[MOTOR_IDS.index(address)]
    persist_calibration(calibration)
    append_process_log("direction_saved", motor=address, reverse=reverse, direction=chosen)
    return {
        "ok": True,
        "action": "direction-save",
        "id": address,
        "reverse": reverse,
        "direction": chosen,
        "message": f"M{MOTOR_IDS.index(address) + 1} 正方向已保存为驱动器方向 {chosen}",
    }


def smooth_calibration_release(connection: object, active_index: int, strong: int, weak: int) -> None:
    """Equalize the last corner's tension before the final disable.

    The last corner motor is still carrying the strong calibration current
    when the fit completes. A direct STOP/DISABLE releases that stored cable
    tension abruptly and can throw the ball. Ramp only that motor down while
    the other three remain at the weak tension, then hold all four at the
    same weak current briefly before the normal safety shutdown.
    """

    levels = []
    for factor in (0.60, 0.35, 0.20):
        level = max(weak, min(strong, int(round(strong * factor))))
        if not levels or level < levels[-1]:
            levels.append(level)
    if not levels or levels[-1] != weak:
        levels.append(weak)
    direction = pull_directions()[active_index]
    for level in levels:
        accepted, _, reply = torque_command(connection, MOTOR_IDS[active_index], direction, level)
        if not accepted:
            detail = direct.hex_bytes(reply) if reply else "超时无回包"
            raise RuntimeError(f"校准完成卸力：M{active_index + 1} 未确认 {level}mA，RX={detail}")
        values = read_all_reliable(connection, attempts=2)
        error = safety_error(values)
        if error:
            raise RuntimeError("校准完成卸力：" + error)
        time.sleep(0.10)

    # Re-assert the weak target on every motor so a stale torque target cannot
    # leave one driver stronger than the others during the settling window.
    for index, address in enumerate(MOTOR_IDS):
        accepted, _, reply = torque_command(connection, address, pull_directions()[index], weak)
        if not accepted:
            detail = direct.hex_bytes(reply) if reply else "超时无回包"
            raise RuntimeError(f"校准完成均衡张力：M{index + 1} 未确认 {weak}mA，RX={detail}")
    settle_until = time.monotonic() + 0.60
    while time.monotonic() < settle_until:
        values = read_all_reliable(connection, attempts=2)
        error = safety_error(values)
        if error:
            raise RuntimeError("校准完成均衡张力：" + error)
        time.sleep(0.10)


def calibration_worker(width: float, height: float, ball: float, strong: int, weak: int) -> None:
    samples: list[list[float]] = []
    error: str | None = None
    try:
        set_state(auto="calibration", message="正在启动校准并检查四台电机", corner=0, point=0, points=0, repeat=False)
        with SERIAL_LOCK, direct.open_serial(current_port(), current_baud()) as connection:
            initial_disable = disable_all(connection)
            if not all(item["accepted"] for item in initial_disable):
                raise RuntimeError("校准启动前四台电机未能全部失能，已取消校准")
            firmware_results = [read_firmware_status(connection, address) for address in MOTOR_IDS]
            non_x = [
                f"M{result['id']}={result.get('firmware') or '无可靠回包'}"
                for result in firmware_results
                if result.get("firmware") != "X"
            ]
            if non_x:
                raise RuntimeError("启动检查失败：实际固件不是全 X（" + "、".join(non_x) + "）")
            values = read_all_reliable(connection)
            if not telemetry_ready_for_calibration(values):
                raise RuntimeError("启动检查失败：四台电机遥测数据不完整")
            error = safety_error(values)
            if error:
                raise RuntimeError("启动检查失败：" + error)
            for corner in range(4):
                if STOP_EVENT.is_set():
                    raise RuntimeError("用户停止校准")
                set_state(
                    corner=corner + 1,
                    message=f"校准角点 {corner + 1}/4：保持四台使能，M{corner + 1} 以 {strong}mA 拉向角点，其余保持 {weak}mA",
                )
                if corner == 0:
                    # The initial disable above is the only deliberate
                    # release. Subsequent corners must change torque targets
                    # while all four drivers remain enabled, otherwise the
                    # ball visibly unloads and re-tensions between corners.
                    for index, address in enumerate(MOTOR_IDS):
                        ok, _, _ = enable_address(connection, address)
                        if not ok:
                            raise RuntimeError(f"电机 M{index + 1} 无法使能")
                    command_order = tuple(range(4))
                else:
                    # Put the new corner under the strong pull first, then
                    # release only the previous corner back to the weak
                    # tension. The other two motors are already at weak.
                    command_order = (corner, corner - 1)
                for index in command_order:
                    address = MOTOR_IDS[index]
                    current = strong if index == corner else weak
                    direction = pull_directions()[index]
                    accepted, tx, reply = torque_command(connection, address, direction, current)
                    if not accepted:
                        detail = direct.hex_bytes(reply) if reply else "超时无回包"
                        raise RuntimeError(f"电机 M{index + 1} 张紧命令未确认，RX={detail}")
                started = time.monotonic()
                previous: list[float] | None = None
                stable_started: float | None = None
                stable_positions: list[list[float]] = []
                previous_corner = samples[-1] if samples else None
                while True:
                    if STOP_EVENT.is_set():
                        raise RuntimeError("用户停止校准")
                    values = read_all_reliable(connection)
                    error = safety_error(values)
                    if error:
                        raise RuntimeError(error)
                    positions = [t.position for t in values]
                    rpms = [t.rpm for t in values]
                    currents = [t.phase_current for t in values]
                    if any(v is None for v in positions + rpms + currents):
                        raise RuntimeError("校准中收到不完整的遥测数据")
                    positions = [float(v) for v in positions]
                    rpms = [float(v) for v in rpms]
                    currents = [int(v) for v in currents]
                    if previous_corner is not None:
                        min_travel = max(180.0, max(width, height) * 1.4)
                        movement_ok = any(abs(positions[i] - previous_corner[i]) >= min_travel for i in range(4)) and abs(positions[corner] - previous_corner[corner]) >= min_travel
                    else:
                        movement_ok = time.monotonic() - started > 0.8
                    stable = (
                        currents[corner] >= max(300, strong * 0.5)
                        and abs(rpms[corner]) < 35.0
                        and max(abs(rpm) for rpm in rpms) < 80.0
                        and previous is not None
                        and max(abs(positions[i] - previous[i]) for i in range(4)) <= 3.0
                        and movement_ok
                    )
                    previous = positions
                    if stable:
                        stable_started = stable_started or time.monotonic()
                        stable_positions.append(positions)
                    else:
                        stable_started = None
                        stable_positions = []
                    set_state(message=f"校准角点 {corner + 1}/4：M{corner + 1} 电流 {currents[corner]}mA，转速 {rpms[corner]:.1f}RPM，" + ("正在确认稳定" if stable_started else "持续拉向角点"))
                    if stable_started and time.monotonic() - stable_started >= 0.6 and len(stable_positions) >= 2:
                        captured = [sum(row[i] for row in stable_positions) / len(stable_positions) for i in range(4)]
                        samples.append(captured)
                        append_process_log(
                            "calibration_corner",
                            corner=corner + 1,
                            samples=captured,
                            strong_current=strong,
                            weak_current=weak,
                        )
                        break
                    if time.monotonic() - started > 90.0:
                        raise RuntimeError(f"角点 {corner + 1} 90 秒内未稳定到位，请检查绳路或调整目标电流")
                    time.sleep(0.05)
            result, reason = compute_calibration(samples, width, height, ball)
            if result is None:
                raise RuntimeError(reason)
            result["strong_current"] = strong
            result["weak_current"] = weak
            with STATE_LOCK:
                STATE["calibration"] = result
                STATE["message"] = "四角校准完成，正在平滑均衡释放最后角点张力"
            persist_calibration(result)
            append_process_log(
                "calibration_complete",
                samples=samples,
                slope=result.get("slope"),
                raw_slope=result.get("raw_slope"),
                common_slope_abs=result.get("common_slope_abs"),
                slope_trim_percent=result.get("slope_trim_percent"),
                r2=result.get("r2"),
                strong_current=strong,
                weak_current=weak,
            )
            smooth_calibration_release(connection, 3, strong, weak)
            disable_results = disable_all(connection)
            if not all(item["accepted"] for item in disable_results):
                raise RuntimeError("四角校准已计算完成，但结束失能未全部收到 ACK；请检查后重新准备")
            set_state(auto="idle", message="四角校准完成，已平滑释放张力，四台电机已失能")
    except Exception as exc:
        error = str(exc)
        append_process_log("calibration_error", error=error, corner=STATE.get("corner"))
        set_state(auto="error", message="已停止：" + error, repeat=False)
    finally:
        if error or STOP_EVENT.is_set():
            recovery = disable_all_with_reconnect(current_port(), current_baud(), announce=True)
            if not recovery["ok"]:
                set_state(
                    auto="error",
                    message="校准/通讯异常，自动重连后仍无法确认四台失能；请立即切断电机动力或使用硬件急停（" + recovery["error"] + "）",
                    repeat=False,
                )
        if STOP_EVENT.is_set() and not error:
            set_state(auto="idle", message="已停止，电机已发送急停", repeat=False)
        STOP_EVENT.clear()


def trajectory_worker(kind: str, spec: dict, *, keep_enabled: bool = False) -> bool:
    completed = False
    error: str | None = None
    try:
        workspace_width, workspace_height = validate_trajectory_bounds(kind, spec)
        with SERIAL_LOCK, direct.open_serial(current_port(), current_baud()) as connection:
            for address in MOTOR_IDS:
                ok, _, _ = enable_address(connection, address)
                if not ok:
                    raise RuntimeError(f"轨迹启动：电机 ID {address} 无法使能")
            initial_values = read_all_reliable(connection)
            if not telemetry_ready_for_calibration(initial_values):
                raise RuntimeError("轨迹启动：四台电机实时位置读取不完整")
            error = safety_error(initial_values)
            if error:
                raise RuntimeError("轨迹启动：" + error)
            current_positions = [float(t.position) for t in initial_values]
            if any(t.position is None for t in initial_values) or len(current_positions) != len(MOTOR_IDS):
                raise RuntimeError("轨迹启动：四台电机当前位置不可用")
            start_x, start_y = sample_path(kind, 0.0, spec)
            targets = [motor_degrees(i, start_x, start_y) for i in range(4)]
            append_process_log(
                "trajectory_open",
                shape=kind,
                workspace=[workspace_width, workspace_height],
                start_xy=[round(start_x, 2), round(start_y, 2)],
                current_positions=current_positions,
                target_positions=targets,
                trajectory_current=trajectory_current_limit(),
            )
            move_to_start_smooth(connection, current_positions, targets, kind_label(kind))
            direct.transact(connection, direct.frame(0x00, 0xFF, 0x66, direct.FRAME_END))
            set_state(
                trajectory_target_path=[],
                trajectory_actual_path=[],
                trajectory_actual_feedback_path=[],
                trajectory_actual_error_mm=None,
                message=f"正在移动到{kind_label(kind)}起点（四电机 0xAA 合帧）",
            )
            started = time.monotonic()
            next_start_safety = started
            next_start_command = started + 0.45
            start_settled_since: float | None = None
            start_near_settled_since: float | None = None
            while True:
                if STOP_EVENT.is_set():
                    raise RuntimeError("用户停止轨迹")
                now = time.monotonic()
                if now >= next_start_command:
                    # Keep reasserting the exact endpoint while the driver
                    # settles; this is only a low-rate keep-alive, not a new
                    # jump to the next shape.
                    send_fast(connection, four_motor_position_frame(targets, [TRAJECTORY_START_RPM] * len(MOTOR_IDS)))
                    next_start_command = now + 0.45
                # Match the ESP32 flow: one complete safety/position pass per
                # 200 ms while settling, instead of hammering all 7 telemetry
                # queries for all four motors every 50 ms on the PC bus.
                if now >= next_start_safety:
                    values = read_all_reliable(connection, attempts=2)
                    error = safety_error(values)
                    if error:
                        raise RuntimeError(error)
                    reached = draw_targets_reached(values, targets)
                    append_process_log(
                        "trajectory_start_sample",
                        shape=kind,
                        reached=reached,
                        positions=[t.position for t in values],
                        rpms=[t.rpm for t in values],
                    )
                    if reached:
                        start_settled_since = start_settled_since or now
                        start_near_settled_since = None
                        if now - start_settled_since >= TRAJECTORY_START_SETTLE_SECONDS:
                            break
                        set_state(message=f"已到{kind_label(kind)}起点，等待四绳张力稳定")
                    elif draw_targets_near_and_stopped(values, targets):
                        start_settled_since = None
                        start_near_settled_since = start_near_settled_since or now
                        if now - start_near_settled_since >= TRAJECTORY_NEAR_SETTLE_SECONDS:
                            residual = max_target_error(values, targets)
                            append_process_log(
                                "trajectory_start_soft_settle",
                                shape=kind,
                                max_error_deg=round(residual or 0.0, 2),
                                tolerance_deg=TRAJECTORY_NEAR_POSITION_TOLERANCE,
                            )
                            set_state(message=f"起点已接近（最大误差 {residual:.1f}°），平滑接管轨迹")
                            break
                        set_state(message="起点已接近，等待四绳张力稳定")
                    else:
                        start_settled_since = None
                        start_near_settled_since = None
                        set_state(message=f"正在移动到{kind_label(kind)}起点")
                    next_start_safety = now + 0.2
                if now - started > 30.0:
                    append_process_log("trajectory_start_timeout", shape=kind, targets=targets)
                    raise RuntimeError("移动到轨迹起点超时")
                time.sleep(0.01)
            while True:
                with STATE_LOCK:
                    repeat = bool(STATE.get("repeat")) and bool(spec.get("repeat"))
                cycle = 1
                duration = max(0.028, float(spec["duration_ms"]) / 1000.0)
                # Use an absolute 100 Hz command clock. Each tick contains
                # exactly one 0xAA frame with M1..M4 together; never split a
                # trajectory tick into four address frames.
                tick = time.monotonic()
                frame_index = 0
                sent_frames = 0
                next_safety = tick + TRAJECTORY_SAFETY_INTERVAL
                last_rate_report = tick
                last_process_log = tick
                safety_motor = 0
                safety_field = 0
                set_state(
                    trajectory_hz_target=TRAJECTORY_HZ,
                    trajectory_hz_actual=0.0,
                    trajectory_frame_bytes=61,
                    trajectory_frames=0,
                    trajectory_target_path=[],
                    trajectory_actual_path=[],
                    trajectory_actual_feedback_path=[],
                    trajectory_actual_error_mm=None,
                    message=f"{kind_label(kind)}轨迹准备发送：四电机 0xAA 合帧，控制目标 {TRAJECTORY_HZ:.0f}Hz",
                )
                position_index = 0
                actual_positions: list[float | None] = [None for _ in MOTOR_IDS]
                filtered_actual: tuple[float, float] | None = None
                while frame_index * TRAJECTORY_INTERVAL < duration:
                    if STOP_EVENT.is_set():
                        raise RuntimeError("用户停止轨迹")
                    scheduled_elapsed = frame_index * TRAJECTORY_INTERVAL
                    deadline = tick + scheduled_elapsed
                    remaining = deadline - time.monotonic()
                    if remaining > 0.0:
                        time.sleep(min(remaining, TRAJECTORY_INTERVAL))
                    phase = scheduled_elapsed / duration
                    next_elapsed = min(duration, scheduled_elapsed + TRAJECTORY_INTERVAL)
                    next_phase = next_elapsed / duration
                    x, y = sample_path(kind, phase, spec)
                    next_x, next_y = sample_path(kind, next_phase, spec)
                    send_fast(connection, trajectory_frame(x, y, next_x, next_y, max(0.001, next_elapsed - scheduled_elapsed)))
                    sent_frames += 1
                    frame_index += 1
                    now = time.monotonic()
                    if frame_index % TRAJECTORY_POSITION_SAMPLE_EVERY_FRAMES == 0:
                        ok, position, reason = query_position_field(connection, position_index)
                        if not ok or position is None:
                            raise RuntimeError(reason)
                        actual_positions[position_index] = position
                        position_index = (position_index + 1) % len(MOTOR_IDS)
                        if position_index == 0 and all(value is not None for value in actual_positions):
                            estimate = position_to_xy([float(value) for value in actual_positions])
                            if estimate is not None:
                                actual_x, actual_y, actual_error = estimate
                                append_trajectory_trace("trajectory_actual_feedback_path", (actual_x, actual_y))
                                if filtered_actual is None:
                                    filtered_actual = (actual_x, actual_y)
                                else:
                                    filtered_actual = (
                                        filtered_actual[0] * (1.0 - TRAJECTORY_FEEDBACK_FILTER_ALPHA)
                                        + actual_x * TRAJECTORY_FEEDBACK_FILTER_ALPHA,
                                        filtered_actual[1] * (1.0 - TRAJECTORY_FEEDBACK_FILTER_ALPHA)
                                        + actual_y * TRAJECTORY_FEEDBACK_FILTER_ALPHA,
                                    )
                                append_trajectory_trace("trajectory_actual_path", filtered_actual)
                                set_state(trajectory_actual_error_mm=round(actual_error, 2))
                    # Keep the commanded path dense enough for a faithful
                    # green trace; this is separate from the slower feedback
                    # polling so visual quality does not reduce control rate.
                    if frame_index % TRAJECTORY_TARGET_TRACE_SAMPLE_EVERY_FRAMES == 0:
                        append_trajectory_trace("trajectory_target_path", (x, y))
                    # Poll one safety field after sending the control frame;
                    # this avoids putting a query in front of a scheduled
                    # 0xAA tick and records any adapter-induced delay.
                    if now >= next_safety:
                        ok, reason = query_safety_field(connection, safety_motor, safety_field)
                        if not ok:
                            raise RuntimeError(reason)
                        safety_field = (safety_field + 1) % 3
                        if safety_field == 0:
                            safety_motor = (safety_motor + 1) % 4
                        next_safety += TRAJECTORY_SAFETY_INTERVAL
                        while next_safety <= now:
                            next_safety += TRAJECTORY_SAFETY_INTERVAL
                    runtime = max(0.001, now - tick)
                    actual_hz = sent_frames / runtime
                    if now - last_process_log >= TRAJECTORY_LOG_INTERVAL:
                        with STATE_LOCK:
                            live_telemetry = copy.deepcopy(STATE.get("telemetry", {}))
                            live_error = STATE.get("trajectory_actual_error_mm")
                        append_process_log(
                            "trajectory_sample",
                            shape=kind,
                            frame=sent_frames,
                            target_xy=[round(x, 2), round(y, 2)],
                            actual_error_mm=live_error,
                            actual_positions=actual_positions,
                            phase_current=[live_telemetry.get(str(address), {}).get("phase_current") for address in MOTOR_IDS],
                            rpm=[live_telemetry.get(str(address), {}).get("rpm") for address in MOTOR_IDS],
                            hz=round(actual_hz, 2),
                        )
                        last_process_log = now
                    if now - last_rate_report >= 0.2 or frame_index == 1:
                        set_state(
                            point=int(round(phase * (spec["point_count"] - 1))),
                            points=spec["point_count"],
                            trajectory_hz_actual=round(actual_hz, 1),
                            trajectory_frames=sent_frames,
                    message=f"{kind_label(kind)}轨迹第 {cycle} 轮 · 四电机合帧目标 {TRAJECTORY_HZ:.0f}Hz（实测发送 {actual_hz:.1f}Hz）· ({x:.1f}, {y:.1f})mm",
                        )
                        last_rate_report = now
                end_x, end_y = sample_path(kind, 1.0, spec)
                previous_x, previous_y = sample_path(kind, max(0.0, 1.0 - TRAJECTORY_INTERVAL / duration), spec)
                send_fast(connection, trajectory_frame(previous_x, previous_y, end_x, end_y, TRAJECTORY_INTERVAL))
                sent_frames += 1
                runtime = max(0.001, time.monotonic() - tick)
                set_state(trajectory_hz_actual=round(sent_frames / runtime, 1), trajectory_frames=sent_frames)
                append_trajectory_trace("trajectory_target_path", (end_x, end_y))
                final_started = time.monotonic()
                final_targets = [motor_degrees(i, end_x, end_y) for i in range(4)]
                final_settled = False
                final_near_settled_since: float | None = None
                next_final_command = final_started
                last_final_error: float | None = None
                while time.monotonic() - final_started < 15.0:
                    if STOP_EVENT.is_set():
                        raise RuntimeError("用户停止轨迹")
                    now = time.monotonic()
                    if now >= next_final_command:
                        # Reassert the exact endpoint during the bounded
                        # finish window so a single lost AA frame cannot
                        # leave one motor just short forever.
                        send_fast(
                            connection,
                            four_motor_position_frame(
                                final_targets,
                                [max(20.0, TRAJECTORY_START_RPM * 0.60)] * len(MOTOR_IDS),
                            ),
                        )
                        next_final_command = now + TRAJECTORY_FINAL_REASSERT_INTERVAL
                    values = read_all_reliable(connection, attempts=2)
                    error = safety_error(values)
                    if error:
                        raise RuntimeError(error)
                    live_positions = [t.position for t in values]
                    if all(value is not None for value in live_positions):
                        estimate = position_to_xy([float(value) for value in live_positions])
                        if estimate is not None:
                            actual_x, actual_y, actual_error = estimate
                            append_trajectory_trace("trajectory_actual_feedback_path", (actual_x, actual_y))
                            if filtered_actual is None:
                                filtered_actual = (actual_x, actual_y)
                            else:
                                filtered_actual = (
                                    filtered_actual[0] * (1.0 - TRAJECTORY_FEEDBACK_FILTER_ALPHA)
                                    + actual_x * TRAJECTORY_FEEDBACK_FILTER_ALPHA,
                                    filtered_actual[1] * (1.0 - TRAJECTORY_FEEDBACK_FILTER_ALPHA)
                                    + actual_y * TRAJECTORY_FEEDBACK_FILTER_ALPHA,
                                )
                            append_trajectory_trace("trajectory_actual_path", filtered_actual)
                            set_state(trajectory_actual_error_mm=round(estimate[2], 2))
                    if draw_targets_reached(values, final_targets):
                        final_settled = True
                        break
                    if draw_targets_near_and_stopped(values, final_targets):
                        final_near_settled_since = final_near_settled_since or now
                        if now - final_near_settled_since >= TRAJECTORY_NEAR_SETTLE_SECONDS:
                            last_final_error = max_target_error(values, final_targets)
                            append_process_log(
                                "trajectory_finish_soft_settle",
                                shape=kind,
                                max_error_deg=round(last_final_error or 0.0, 2),
                                tolerance_deg=TRAJECTORY_NEAR_POSITION_TOLERANCE,
                            )
                            set_state(message=f"终点已接近（最大误差 {last_final_error:.1f}°），平滑结束")
                            final_settled = True
                            break
                    else:
                        final_near_settled_since = None
                    last_final_error = max_target_error(values, final_targets)
                    time.sleep(0.2)
                if not final_settled:
                    raise RuntimeError(
                        f"轨迹终点 15 秒内未稳定到位（最大误差 {last_final_error:.1f}°），已停止并失能"
                        if last_final_error is not None
                        else "轨迹终点 15 秒内未收到完整位置反馈，已停止并失能"
                    )
                with STATE_LOCK:
                    repeat = bool(STATE.get("repeat")) and bool(spec.get("repeat"))
                if not repeat:
                    append_process_log(
                        "trajectory_complete",
                        shape=kind,
                        frames=sent_frames,
                        hz=round(sent_frames / max(0.001, time.monotonic() - tick), 2),
                        endpoint=[round(end_x, 2), round(end_y, 2)],
                    )
                    set_state(auto="idle", point=spec["point_count"] - 1, message=f"{kind_label(kind)}轨迹完成，四台电机已自动失能", repeat=False)
                    completed = True
                    return True
                set_state(message="当前轨迹轮完成，继续下一轮")
    except Exception as exc:
        error = str(exc)
        append_process_log("trajectory_error", shape=kind, error=error)
        set_state(auto="error", message="已紧急停止：" + error + "；正在自动重连并失能四台电机", repeat=False)
    finally:
        # A four-shape test uses one continuous enabled session: disabling
        # here after every shape unloads the cables and makes the next shape's
        # start move prone to stalling. Any error or user stop still forces
        # the safety shutdown even when the caller requests keep_enabled.
        must_disable = bool(error or STOP_EVENT.is_set() or not keep_enabled)
        recovery = (
            disable_all_with_reconnect(current_port(), current_baud(), announce=bool(error or STOP_EVENT.is_set()))
            if must_disable
            else {"ok": True, "error": ""}
        )
        if not recovery["ok"]:
            set_state(
                auto="error",
                message="轨迹已停止，但 RS485 自动重连后仍无法确认四台失能；请立即切断电机动力或使用硬件急停（" + recovery["error"] + "）",
                repeat=False,
            )
        elif STOP_EVENT.is_set():
            set_state(auto="idle", message="已停止，已自动重连并确认四台电机急停/失能", repeat=False)
        elif error:
            set_state(auto="error", message="已紧急停止：" + error + "；已自动重连并确认四台电机失能", repeat=False)
        elif keep_enabled:
            set_state(message=f"{kind_label(kind)}完成，四台保持使能，准备下一个测试图形")
        STOP_EVENT.clear()
    return completed


def test_shape_spec(kind: str, width: float, height: float) -> dict:
    durations = {"circle": 1200, "triangle": 900, "rectangle": 1400, "cross": 1200}
    if kind not in durations:
        raise ValueError("未知测试图形")
    duration = durations[kind]
    spec = {
        "width": width,
        "height": height,
        "duration_ms": duration,
        "point_count": int(duration / (TRAJECTORY_INTERVAL * 1000.0)) + 1,
        "repeat": False,
    }
    if kind == "triangle":
        vertices = [(width / 2.0, height * 0.10), (width * 0.90, height * 0.85), (width * 0.10, height * 0.85)]
        path = chaikin_path(vertices, closed=True, passes=2)
        spec["path"], spec["cumulative"] = path_with_cumulative(path)
    elif kind == "rectangle":
        vertices = [(width * 0.18, height * 0.18), (width * 0.82, height * 0.18), (width * 0.82, height * 0.82), (width * 0.18, height * 0.82)]
        path = chaikin_path(vertices, closed=True, passes=2)
        spec["path"], spec["cumulative"] = path_with_cumulative(path)
    elif kind == "cross":
        vertices = [(width * 0.18, height * 0.18), (width * 0.82, height * 0.82), (width / 2.0, height / 2.0), (width * 0.82, height * 0.18), (width * 0.18, height * 0.82)]
        path = chaikin_path(vertices, closed=False, passes=2)
        spec["path"], spec["cumulative"] = path_with_cumulative(path)
    return spec


def kind_label(kind: str) -> str:
    return {"circle": "圆形", "triangle": "三角形", "rectangle": "矩形", "cross": "十字", "custom": "手绘"}.get(kind, kind)


def current_port() -> str:
    with STATE_LOCK:
        port = STATE.get("port")
    return str(port or direct.choose_port(None))


def current_baud() -> int:
    with STATE_LOCK:
        return int(STATE.get("baud") or direct.DEFAULT_BAUD)


def start_thread(target, *args) -> None:
    global AUTO_THREAD
    if active_auto():
        raise RuntimeError("自动流程正在运行，请先按急停")
    STOP_EVENT.clear()
    AUTO_THREAD = threading.Thread(target=target, args=args, daemon=True)
    AUTO_THREAD.start()


def auto_prepare_on_boot() -> None:
    """Run the non-motion safety preparation once when the web server starts.

    The user has already confirmed this machine's mechanism.  Preparation only
    scans, verifies/switches firmware, locks local keys, and sends STOP/DISABLE;
    it does not enable or move a motor.  Keeping it here prevents a browser
    reload or a code restart from silently leaving the UI in a stale locked
    state and asking for the same confirmation again.
    """

    try:
        port = direct.choose_port(None)
        baud = direct.DEFAULT_BAUD
        set_state(port=port, baud=baud, control_unlocked=False, message="自动准备：服务启动后正在扫描 ID 1..4")
        result = prepare_system(port, baud)
        if not result.get("ready"):
            set_state(control_unlocked=False, auto="error", message=result.get("message", "自动准备未完成"))
    except Exception as exc:
        set_state(control_unlocked=False, auto="error", message="服务启动自动准备失败：" + str(exc))


def require_confirmed(data: dict) -> None:
    if data.get("confirmed") is not True:
        raise ValueError("此操作需要网页安全确认")


def require_control_unlocked() -> None:
    with STATE_LOCK:
        if not STATE["control_unlocked"]:
            raise RuntimeError("控制按钮当前已锁定，请先确认 X 固件绿灯闪烁模式和机构安全后解锁")


def set_control_lock(data: dict) -> dict:
    global STATE
    armed = bool(data.get("armed", False))
    if armed:
        require_confirmed(data)
    with STATE_LOCK:
        STATE["control_unlocked"] = armed
    return {"ok": True, "control_unlocked": armed, "message": "控制按钮已解锁" if armed else "控制按钮已锁定"}


def _prepare_system_impl(port: str, baud: int) -> dict:
    if active_auto():
        raise RuntimeError("自动流程运行中，请先按急停")
    set_state(control_unlocked=False, message="自动准备：正在扫描 ID 1..4")
    with SERIAL_LOCK, direct.open_serial(port, baud) as connection:
        found = direct.scan(connection, 1, 4, verbose=False)
        missing = [address for address in MOTOR_IDS if address not in found]
        if missing:
            message = "缺少电机 ID " + ", ".join(map(str, missing)) + "；请在 ID 修改区处理后重新执行一键准备"
            set_state(message=message, auto="idle", control_unlocked=False)
            return {"ok": True, "action": "prepare", "ready": False, "stage": "ids", "found": found, "missing": missing, "message": message}

        set_state(message="自动准备：正在读取四台固件状态")
        firmware = [read_firmware_status(connection, address) for address in MOTOR_IDS]
        firmware_ok = all(item.get("firmware") == "X" for item in firmware)
        switch_result = None
        if not firmware_ok:
            set_state(message="自动准备：发现非 X 固件，正在停止并切换四台为 X")
            switch_result = switch_all_to_x(connection)
            if not switch_result["ok"]:
                disable_all(connection)
                message = "X 固件切换失败；请检查回包和接线后重试"
                set_state(message=message, auto="error", control_unlocked=False)
                return {"ok": True, "action": "prepare", "ready": False, "stage": "firmware", "firmware": switch_result.get("status", firmware), "switch": switch_result, "message": message}
            firmware = switch_result["status"]

        set_state(message="自动准备：正在锁定四台电机本体按键")
        keys_ok, key_results = lock_keys_all(connection)
        set_state(message="自动准备：正在让四台电机全部失能")
        disable_results = disable_all(connection)
        disable_ok = all(item["accepted"] for item in disable_results)
        if not keys_ok:
            message = "电机本体按键未全部锁定；四台已发送失能，请检查回包后重试"
            set_state(message=message, auto="error", control_unlocked=False)
            return {"ok": True, "action": "prepare", "ready": False, "stage": "keys", "firmware": firmware, "keys": key_results, "disable": disable_results, "message": message}
        if not disable_ok:
            message = "四台电机失能未全部收到 ACK；请检查 RS485 后重试"
            set_state(message=message, auto="error", control_unlocked=False)
            return {"ok": True, "action": "prepare", "ready": False, "stage": "disable", "firmware": firmware, "keys": key_results, "disable": disable_results, "message": message}

    message = "自动准备完成：ID 1..4 在线、X 固件已确认、物理按键已锁定、四台电机已失能"
    set_state(message=message, auto="idle", control_unlocked=True)
    return {"ok": True, "action": "prepare", "ready": True, "found": list(MOTOR_IDS), "firmware": firmware, "keys": key_results, "disable": disable_results, "message": message}


def prepare_system(port: str, baud: int) -> dict:
    if not PREPARE_GUARD.acquire(blocking=False):
        raise RuntimeError("自动准备正在进行，请等待连接检查完成")
    try:
        return _prepare_system_impl(port, baud)
    finally:
        PREPARE_GUARD.release()


def schedule_prepare_if_needed(port: str, baud: int) -> None:
    """Re-run the non-motion preparation after a previously lost link returns."""

    with STATE_LOCK:
        needed = not bool(STATE.get("control_unlocked")) and STATE.get("auto") not in {"calibration", "direction-test"}
    if not needed or PREPARE_GUARD.locked():
        return

    def worker() -> None:
        try:
            prepare_system(port, baud)
        except Exception as exc:
            set_state(control_unlocked=False, auto="error", message="通讯恢复后的自动准备失败：" + str(exc))

    threading.Thread(target=worker, name="y42-reprepare", daemon=True).start()


def execute(data: dict) -> dict:
    action = str(data.get("action", ""))
    if action == "ports":
        return {"ok": True, "ports": [serial_port_label(p) for p in direct.list_serial_ports()]}
    if action == "stop":
        STOP_EVENT.set()
        if active_auto():
            set_state(message="已发出急停请求，正在停止并失能四台电机")
            return {"ok": True, "message": "已发出急停请求；自动线程会立即停止并失能四台电机"}
    port = direct.choose_port(str(data.get("port") or "") or None)
    baud = int(data.get("baud", direct.DEFAULT_BAUD))
    if baud < 1200 or baud > 1000000:
        raise ValueError("波特率范围无效")
    with STATE_LOCK:
        STATE["port"] = port
        STATE["baud"] = baud
    if action == "prepare":
        require_confirmed(data)
        return prepare_system(port, baud)
    if action == "stop":
        STOP_EVENT.clear()
        recovery = disable_all_with_reconnect(port, baud, announce=True)
        acknowledged = bool(recovery["ok"])
        message = (
            "四台电机已急停并失能，已自动重连并收到完整回包"
            if acknowledged
            else "急停帧已重试，但仍无法确认四台失能；请立即切断电机动力或使用硬件急停"
        )
        return {"ok": True, "action": action, "acknowledged": acknowledged, "message": message, "results": recovery.get("results", []), "recovery": recovery}
    if action == "stability":
        if active_auto():
            raise RuntimeError("自动流程运行中，请先按急停再做通讯稳定性测试")
        rounds = int(data.get("rounds", 20))
        return stability_test(port, baud, rounds)
    if action == "baud-sweep":
        if active_auto():
            raise RuntimeError("自动流程运行中，请先按急停再做波特率诊断")
        requested = data.get("bauds", [115200, 57600, 38400, 19200, 9600, 460800])
        bauds = [int(value) for value in requested]
        if not bauds or len(bauds) > 10 or any(value < 1200 or value > 1000000 for value in bauds):
            raise ValueError("波特率诊断参数无效")
        return baud_sweep(port, bauds)
    if action == "status":
        if active_auto():
            snapshot = state_snapshot()
            return {"ok": True, "action": action, "motors": list(snapshot["telemetry"].values()), "auto": snapshot["auto"]}
        return status_for_web(port, baud)
    if action == "firmware-status":
        if active_auto():
            raise RuntimeError("自动流程运行中，请先按急停再读取固件类型")
        results = []
        with SERIAL_LOCK, direct.open_serial(port, baud) as connection:
            for address in MOTOR_IDS:
                results.append(read_firmware_status(connection, address))
        return {"ok": True, "action": action, "all_responded": all(item["ok"] for item in results), "results": results, "message": "已读取四台电机实际固件类型；无回包的电机可能不支持固件类型读取指令"}
    if action == "scan":
        start = int(data.get("start", direct.MIN_ID))
        end = int(data.get("end", direct.MAX_ID))
        if not 0 <= start <= end <= direct.MAX_ID:
            raise ValueError("扫描范围必须是 0 <= start <= end <= 255")
        if active_auto():
            raise RuntimeError("自动流程运行中，请先按急停再扫描 ID")
        # ID 0 is the broadcast address. Sending a status query to it can make
        # every motor answer at once, causing a collision, so 0 is accepted as
        # the UI range marker but is never transmitted on the bus.
        scan_start = max(direct.MIN_ID, start)
        with SERIAL_LOCK, direct.open_serial(port, baud) as connection:
            found = direct.scan(connection, scan_start, end, verbose=False) if scan_start <= end else []
        note = "（0 为广播地址，未向 ID 0 发送帧）" if start == 0 else ""
        return {"ok": True, "action": action, "start": start, "end": end, "scanned_start": scan_start, "found": found, "message": f"扫描完成，实际扫描 ID {scan_start}..{end}，发现 {len(found)} 个地址{note}"}
    if action == "direction-test":
        require_control_unlocked()
        require_confirmed(data)
        if active_auto():
            raise RuntimeError("已有自动流程运行中，请先按急停")
        address = direct.check_id(int(data.get("id", 0)))
        with STATE_LOCK:
            ready = copy.deepcopy(STATE.get("direction_ready", {}))
            ready.pop(str(address), None)
            STATE["direction_ready"] = ready
        start_thread(direction_test_worker, address)
        return {"ok": True, "action": action, "id": address, "message": f"M{MOTOR_IDS.index(address) + 1} 正方向测试已启动：仅转 45°，不复位"}
    if action == "direction-save":
        require_control_unlocked()
        require_confirmed(data)
        if active_auto():
            raise RuntimeError("正方向测试仍在运行，请等待测试完成")
        address = direct.check_id(int(data.get("id", 0)))
        return save_direction_choice(address, data.get("reverse") is True)
    if action in {"all-enable", "all-disable", "release-all"}:
        require_control_unlocked()
        require_confirmed(data)
        if active_auto():
            raise RuntimeError("自动流程运行中，仅允许急停")
        with SERIAL_LOCK, direct.open_serial(port, baud) as connection:
            if action == "all-enable":
                results = []
                for address in MOTOR_IDS:
                    ok, tx, rx = enable_address(connection, address)
                    results.append({"id": address, "ok": ok, "tx": direct.hex_bytes(tx), "rx": direct.hex_bytes(rx)})
                return {"ok": all(item["ok"] for item in results), "action": action, "results": results}
            if action == "all-disable":
                return {"ok": True, "action": action, "results": disable_all(connection)}
            results = []
            for address in MOTOR_IDS:
                tx = command_frame(address, "release", {})
                results.append(command_result(address, 0x0E, tx, direct.transact(connection, tx)))
            return {"ok": True, "action": action, "results": results}
    if action in {"firmware-x", "firmware-emm", "firmware-turbo"}:
        require_confirmed(data)
        if active_auto():
            raise RuntimeError("自动流程运行中，仅允许急停")
        firmware_code = {"firmware-x": 0, "firmware-emm": 1, "firmware-turbo": 2}[action]
        firmware_name = {0: "X", 1: "Emm", 2: "Emm 狂暴模式"}[firmware_code]
        with SERIAL_LOCK, direct.open_serial(port, baud) as connection:
            stop_results = stop_all(connection)
            if not all(item["ok"] for item in stop_results):
                return {"ok": False, "action": action, "target": firmware_name, "stop_results": stop_results, "message": "切换前急停未全部收到回包，已取消固件切换"}
            results = []
            for address in MOTOR_IDS:
                tx = firmware_switch_frame(address, firmware_code, store=True)
                rx = direct.transact(connection, tx, first_byte_timeout=0.3)
                results.append(command_result(address, 0xD5, tx, rx))
            restart_results = []
            for address in MOTOR_IDS:
                tx = firmware_restart_frame(address)
                rx = direct.transact(connection, tx, first_byte_timeout=0.5)
                restart_results.append(command_result(address, 0x08, tx, rx))
                time.sleep(0.25)
            time.sleep(0.5)
            status = []
            for address in MOTOR_IDS:
                status.append(read_firmware_status(connection, address))
        accepted = all(item["accepted"] for item in results)
        verified = all(item.get("ok") and item.get("firmware_code") == firmware_code for item in status)
        restarted = all(item["accepted"] for item in restart_results)
        message = f"已逐台写入 {firmware_name} 固件类型并重启复核"
        if verified:
            message += "，读取复核一致"
        else:
            message += "，仍有电机不是目标固件或未提供可靠回读"
        if not restarted:
            message += "；部分重启命令未收到 ACK"
        return {"ok": accepted and verified, "action": action, "target": firmware_name, "stored": True, "accepted": accepted, "restarted": restarted, "verified": verified, "results": results, "restart_results": restart_results, "status": status, "message": message}
    if action == "lock-keys":
        require_confirmed(data)
        if active_auto():
            raise RuntimeError("自动流程运行中，请先按急停再锁定电机按键")
        with SERIAL_LOCK, direct.open_serial(port, baud) as connection:
            ok, results = lock_keys_all(connection)
        return {"ok": ok, "action": action, "locked": ok, "results": results, "message": "四台电机物理按键已锁定并读取复核" if ok else "物理按键锁定未全部复核成功"}
    if action == "set-id":
        require_confirmed(data)
        if active_auto():
            raise RuntimeError("自动流程运行中，请先按急停再修改 ID")
        old_id = direct.check_id(int(data["old"]), "旧 ID")
        new_id = direct.check_id(int(data["new"]), "新 ID")
        if old_id == new_id:
            raise ValueError("旧 ID 和新 ID 不能相同")
        with SERIAL_LOCK, direct.open_serial(port, baud) as connection:
            tx = direct.frame(old_id, 0xAE, 0x4B, 0x01, new_id, direct.FRAME_END)
            rx = direct.set_motor_id(connection, old_id, new_id)
        accepted = direct.command_accepted(rx, old_id, 0xAE)
        if accepted:
            set_state(control_unlocked=False, message=f"ID {old_id} 已改为 {new_id}；请断电重启后重新执行一键准备")
        return {"ok": accepted, "action": action, "accepted": accepted, "tx": direct.hex_bytes(tx), "rx": direct.hex_bytes(rx), "message": "已收到设置 ID ACK，请断电重启后重新扫描新 ID" if accepted else "设置 ID 未收到标准 ACK，请勿继续改名；确认总线上只有目标电机"}
    if action == "raw":
        require_control_unlocked()
        require_confirmed(data)
        tx = direct.parse_hex_frame(str(data.get("hex_frame", "")))
        with SERIAL_LOCK, direct.open_serial(port, baud) as connection:
            rx = direct.transact(connection, tx, first_byte_timeout=0.08)
        return {"ok": True, "action": action, "tx": direct.hex_bytes(tx), "rx": direct.hex_bytes(rx)}
    address = direct.check_id(int(data.get("id", 0)))
    if action in {"enable", "position", "torque", "speed", "home", "disable", "clear", "release", "abort-home", "set-home"}:
        require_control_unlocked()
        if action not in {"disable", "clear", "release", "abort-home", "set-home"}:
            require_confirmed(data)
        if active_auto():
            raise RuntimeError("自动流程运行中，仅允许急停")
        with SERIAL_LOCK, direct.open_serial(port, baud) as connection:
            if action in {"position", "torque", "speed", "home"}:
                enabled, _, _ = enable_address(connection, address)
                if not enabled:
                    raise RuntimeError(f"电机 ID {address} 自动使能失败")
            if action == "disable":
                stop_tx = command_frame(address, "stop", {})
                stop_rx = direct.transact(connection, stop_tx)
                tx = command_frame(address, action, data)
                rx = direct.transact(connection, tx)
                return {"ok": True, "action": action, "stop_rx": direct.hex_bytes(stop_rx), "tx": direct.hex_bytes(tx), "rx": direct.hex_bytes(rx)}
            tx = command_frame(address, action, data)
            rx = direct.transact(connection, tx)
        return {"ok": True, "action": action, **command_result(address, tx[1], tx, rx)}
    raise ValueError(f"未知操作: {action}")


def start_auto(data: dict) -> dict:
    global TRAJECTORY_SPEC
    action = str(data.get("action", ""))
    if action == "abort":
        STOP_EVENT.set()
        return {"ok": True, "message": "已请求停止自动流程"}
    if action == "stop_loop":
        require_control_unlocked()
        require_confirmed(data)
        with STATE_LOCK:
            STATE["repeat"] = False
        return {"ok": True, "message": "已停止后续循环；当前轮结束后保持终点"}
    require_control_unlocked()
    require_confirmed(data)
    if active_auto():
        raise RuntimeError("自动流程正在运行，请先按急停")
    calibration = calibration_data()
    if action == "calibration-settings":
        return save_calibration_settings(data)
    if action == "calibrate":
        height = float(data.get("side", calibration.get("side", 150)))
        ball = float(data.get("ball", calibration.get("ball", 40)))
        width = height - ball
        strong = int(data.get("strong", calibration.get("strong_current", CALIBRATION_DEFAULT_STRONG_CURRENT)))
        weak = int(data.get("weak", calibration.get("weak_current", CALIBRATION_DEFAULT_WEAK_CURRENT)))
        trajectory_current = int(data.get("trajectory_current", calibration.get("trajectory_current", TRAJECTORY_DEFAULT_CURRENT)))
        if not 50 <= height <= 1000 or not 1 <= ball < height or width < 50 or not 50 <= strong <= HARD_CURRENT or not 20 <= weak < strong or not 300 <= trajectory_current <= HARD_CURRENT:
            raise ValueError("校准参数范围无效：上下距离 50..1000、球径 1..上下距离、实际左右宽度至少 50、强电流 50..3000、张力 20..强电流、轨迹电流 300..3000")
        previous = calibration
        calibration = blank_calibration()
        calibration["direction_reverse"] = list(direction_reversal_flags())
        calibration["slope_trim_percent"] = normalized_slope_trims(previous)
        calibration["trajectory_current"] = trajectory_current
        calibration.update(
            {
                "side": height,
                "ball": ball,
                "effective_side": height,
                "workspace_width": width,
                "workspace_height": height,
                "geometry_version": 2,
                "strong_current": strong,
                "weak_current": weak,
            }
        )
        with STATE_LOCK:
            STATE["calibration"] = calibration
            STATE["message"] = "已开始四角校准；请确认机构无人接触并准备硬件急停"
        start_thread(calibration_worker, width, height, ball, strong, weak)
        append_process_log(
            "calibration_start",
            side=height,
            ball=ball,
            workspace=[width, height],
            strong_current=strong,
            weak_current=weak,
            trajectory_current=trajectory_current,
            slope_trim_percent=calibration.get("slope_trim_percent"),
        )
        return {"ok": True, "message": f"四角自动校准已启动：实际工作区 {width:.1f}×{height:.1f}mm，张力 {weak}mA"}
    if not calibration.get("valid"):
        raise RuntimeError("请先完成四角自动校准")
    width, height = workspace_dimensions(calibration)
    if action in {"circle", "triangle", "rectangle", "cross"}:
        TRAJECTORY_SPEC = test_shape_spec(action, width, height)
        with STATE_LOCK:
            STATE["repeat"] = False
        start_thread(trajectory_worker, action, TRAJECTORY_SPEC)
        return {"ok": True, "message": f"{kind_label(action)}轨迹已启动"}
    if action == "loop_path":
        if not TRAJECTORY_SPEC or TRAJECTORY_SPEC.get("kind") != "custom":
            raise RuntimeError("还没有可循环的手绘轨迹，请先运行一次手绘轨迹")
        spec = copy.deepcopy(TRAJECTORY_SPEC)
        spec["repeat"] = True
        with STATE_LOCK:
            STATE["repeat"] = True
        start_thread(trajectory_worker, "custom", spec)
        return {"ok": True, "message": "已开始循环运行上次手绘轨迹；按急停结束"}
    raise ValueError(f"未知自动操作: {action}")


def handle_path(data: dict) -> dict:
    global TRAJECTORY_SPEC
    require_control_unlocked()
    require_confirmed(data)
    if active_auto():
        raise RuntimeError("自动流程运行中，请先按急停")
    calibration = calibration_data()
    if not calibration.get("valid"):
        raise RuntimeError("请先完成四角自动校准")
    raw = data.get("points")
    if not isinstance(raw, list) or not 2 <= len(raw) <= 2000:
        raise ValueError("手绘轨迹需要 2..2000 个点")
    width, height = workspace_dimensions(calibration)
    points = []
    for item in raw:
        nx = float(item["x"])
        ny = float(item["y"])
        if not 0 <= nx <= 1 or not 0 <= ny <= 1:
            raise ValueError("手绘坐标超出画布")
        points.append((width * nx, height * (1.0 - ny)))
    filtered = [points[0]]
    for point in points[1:-1]:
        if math.hypot(point[0] - filtered[-1][0], point[1] - filtered[-1][1]) >= 1.0:
            filtered.append(point)
    if math.hypot(points[-1][0] - filtered[-1][0], points[-1][1] - filtered[-1][1]) > 0.05:
        filtered.append(points[-1])
    if len(filtered) >= 3:
        # Two corner-cutting passes remove hand jitter and prevent a sudden
        # direction/velocity jump at every raw pointer sample. Endpoints stay
        # fixed so the start/end position remains deterministic.
        points = chaikin_path(filtered, closed=False, passes=2)
    else:
        points = filtered
    total = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))
    if total < 3:
        raise ValueError("手绘轨迹太短")
    requested = max(30.0, min(5000.0, float(data.get("speed", 200))))
    max_slope = max(abs(value) for value in effective_slopes(calibration))
    motor_limit = min(5000.0, TRAJECTORY_MAX_RPM * 6.0 / (max_slope * TRAJECTORY_FEEDFORWARD)) if max_slope > 0.001 else 300.0
    interval_ms = TRAJECTORY_INTERVAL * 1000.0
    controller_limit = TRAJECTORY_MAX_STEP_MM * 1000.0 / interval_ms
    update_limit = total * 1000.0 / (interval_ms * 4.0)
    speed = min(requested, motor_limit, controller_limit, update_limit)
    elapsed = 0.0
    cumulative = [0.0]
    for a, b in zip(points, points[1:]):
        elapsed += math.hypot(b[0] - a[0], b[1] - a[1]) / speed
        cumulative.append(elapsed)
    duration = round(elapsed * 1000.0)
    if duration < 28:
        raise ValueError("轨迹过短或速度过高，请降低速度")
    if duration > 600000:
        raise ValueError("轨迹运行时间超过 10 分钟，请缩短轨迹")
    TRAJECTORY_SPEC = {"kind": "custom", "width": width, "height": height, "path": points, "cumulative": cumulative, "duration_ms": duration, "point_count": int(duration / interval_ms) + 1, "repeat": bool(data.get("repeat", False))}
    with STATE_LOCK:
        STATE["repeat"] = bool(TRAJECTORY_SPEC["repeat"])
    start_thread(trajectory_worker, "custom", TRAJECTORY_SPEC)
    return {"ok": True, "message": f"已接收 {len(raw)} 个点并平滑为 {len(points)} 点，采用 {speed:.1f}mm/s，预计 {duration / 1000:.2f} 秒"}


PAGE = r'''<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Y42 · 校准与绘画</title>
<style>
*{box-sizing:border-box}:root{--ink:#1d1d1f;--muted:#6e6e73;--line:#d2d2d7;--surface:#fff;--soft:#f5f5f7;--blue:#0071e3;--green:#16834b;--amber:#b7791f;--red:#b42318}body{margin:0;background:var(--soft);color:var(--ink);font:14px/1.5 system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI","Microsoft YaHei",sans-serif;letter-spacing:.01em}main{max-width:1080px;margin:auto;padding:22px 20px 60px;display:grid;grid-template-columns:1fr;gap:16px}.full{grid-column:1/-1}.card{background:var(--surface);border:1px solid #e5e5ea;border-radius:18px;padding:22px;box-shadow:0 8px 28px #0000000a;margin:0}h1{margin:0 0 8px;font-size:32px;line-height:1.15;letter-spacing:-.03em}h2{margin:0 0 10px;font-size:21px;line-height:1.2;letter-spacing:-.02em}h3{margin:12px 0 7px;font-size:14px}.muted{color:var(--muted);font-size:13px;line-height:1.6}.warning{background:#fffaf1;border-color:#f0d59b}.danger{background:#fff5f3;border-color:#efb5b0}.goodbox{background:#f3fbf6;border-color:#c5e8d2}label{display:block;color:var(--muted);font-size:12px;margin:9px 0 4px}input,select,button{width:100%;height:38px;border:1px solid var(--line);border-radius:10px;padding:0 11px;font:inherit;background:#fff}input:focus,select:focus{outline:3px solid #0071e333;border-color:var(--blue)}button{border:0;background:var(--blue);color:#fff;font-weight:700;cursor:pointer;margin-top:8px;transition:transform .15s,filter .15s}button:hover:not(:disabled){filter:brightness(1.05);transform:translateY(-1px)}button.secondary{background:#6e6e73}button.good{background:var(--green)}button.warn{background:var(--amber)}button.stop{background:var(--red)}button.ghost{background:#eef0f3;color:#253249}button:disabled{opacity:.45;cursor:not-allowed;transform:none}.floating-status{position:sticky;top:0;z-index:20;max-width:1080px;margin:0 auto;padding:8px 20px;display:grid;grid-template-columns:repeat(4,1fr);gap:6px;background:#f5f5f7e8;backdrop-filter:blur(16px)}.floating-motor{min-width:0;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;padding:7px 9px;border:1px solid var(--line);border-radius:10px;background:#ffffffd9;font-size:12px;font-weight:700}.floating-motor.online{color:#087443;border-color:#b9e4ca}.floating-motor.offline{color:#a5261a;border-color:#efb5b0}.attention{animation:attention-pulse 1s ease-in-out 3}@keyframes attention-pulse{50%{box-shadow:0 0 0 5px #f59e0b88}}.row{display:grid;grid-template-columns:1fr 1fr;gap:9px}.triple{display:grid;grid-template-columns:repeat(3,1fr);gap:9px}.quad{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}.status{margin-top:10px;padding:9px 11px;border-radius:10px;background:#f5f6f8;color:#40506a;line-height:1.55;min-height:38px}.pill{display:inline-block;padding:2px 7px;border-radius:999px;font-size:12px;font-weight:800}.pill.ok{background:#dff6e8;color:#087443}.pill.bad{background:#ffe1de;color:#a5261a}.motor-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:10px}.motor{border:1px solid #e5e5ea;border-radius:14px;padding:12px;background:#fff}.motor h3{font-size:13px}.metric{display:flex;justify-content:space-between;border-top:1px solid #f0f0f2;padding:5px 0;font-size:12px;color:var(--muted)}.metric b{color:var(--ink)}.online{color:#087443}.offline{color:#a5261a}.small{font-size:11px;color:var(--muted);min-height:18px}.check{display:flex;align-items:center;gap:7px}.check input{width:auto;height:auto}canvas{width:100%;max-width:520px;border:1px solid var(--line);border-radius:14px;background:#fff}.setup-stack{display:grid;gap:14px}.fold{overflow:hidden}.fold>summary{list-style:none;cursor:pointer;font-size:16px;font-weight:800;display:flex;align-items:center;justify-content:space-between}.fold>summary::-webkit-details-marker{display:none}.fold>summary:after{content:'＋';font-size:20px;font-weight:400;color:var(--muted)}.fold[open]>summary:after{content:'−'}.fold[open]>summary{margin-bottom:14px}.fold.card{padding:18px 20px}.setup-stack>.card{box-shadow:0 4px 16px #00000008}.hero{padding:30px 28px}.hero .muted{max-width:720px}.prepare-card{padding:24px 28px}.step-kicker{color:var(--blue);font-size:12px;font-weight:800;letter-spacing:.08em;text-transform:uppercase;margin-bottom:6px}.step-card{padding:24px 28px}.compact-card{padding:18px 20px}.direction-body{padding-top:2px}.unavailable{color:var(--muted)}.advanced{display:none}@media(max-width:720px){main{padding:12px 12px 40px}.floating-status{padding:6px 12px;grid-template-columns:repeat(2,1fr)}.floating-motor{font-size:11px}.card{padding:18px;border-radius:15px}h1{font-size:26px}h2{font-size:19px}.hero,.prepare-card,.step-card{padding:20px}.motor-grid{grid-template-columns:repeat(2,1fr)}.triple,.row{grid-template-columns:1fr}.quad{grid-template-columns:repeat(2,1fr)}}.x-banner{margin-top:18px;padding:10px 12px;border:1px solid #f0d59b;border-radius:12px;background:#fffaf1;color:#8a5a13;font-size:13px;font-weight:700;line-height:1.5;text-align:left}.x-banner .blink{color:#087443}
</style>
<style>
:root{--ink:#172033;--muted:#718096;--line:#e5eaf2;--surface:#ffffffef;--soft:#f5f7fb;--blue:#1769e8;--green:#168451;--amber:#b87516;--red:#c5362f;--shadow:0 18px 55px rgba(38,54,82,.09)}
html{scroll-behavior:smooth}body{min-height:100vh;overflow-x:hidden;background:radial-gradient(circle at 8% -5%,#e5efff 0,transparent 35%),radial-gradient(circle at 100% 12%,#fff0dc 0,transparent 30%),var(--soft);color:var(--ink);font-size:15px;line-height:1.6}main{max-width:1180px;padding:28px 24px 76px;gap:20px}.card{border:1px solid rgba(218,225,236,.9);border-radius:24px;background:var(--surface);box-shadow:var(--shadow);padding:26px;backdrop-filter:blur(18px)}.card:hover{box-shadow:0 22px 64px rgba(38,54,82,.12)}h1{font-size:clamp(30px,4vw,46px);font-weight:750;letter-spacing:-.055em}h2{font-size:24px;font-weight:740;letter-spacing:-.035em}h3{font-weight:720;letter-spacing:-.015em}.muted{color:var(--muted);font-size:14px;line-height:1.7}.hero{position:relative;isolation:isolate;overflow:hidden;padding:38px 40px;background:linear-gradient(135deg,rgba(255,255,255,.97),rgba(241,247,255,.91));border-color:#dbe7fb}.hero:after{content:'';position:absolute;z-index:-1;width:260px;height:260px;right:-80px;top:-100px;border-radius:50%;background:radial-gradient(circle,#dceaff 0,rgba(220,234,255,0) 70%)}.hero .muted{max-width:780px}.step-kicker{font-size:11px;letter-spacing:.14em;color:#4775b9}.mapping{color:#315c9d;font-weight:700}.x-banner{margin-top:18px;padding:12px 15px;border:1px solid #f2d9a7;border-radius:14px;background:linear-gradient(90deg,#fff8e9,#fffdf7);color:#7c5b20}.prepare-card{display:grid;grid-template-columns:1fr 1fr;gap:10px 12px;padding:26px 30px;background:linear-gradient(135deg,#fffaf0,#fff)}.prepare-card>.step-kicker,.prepare-card>h2,.prepare-card>.muted,.prepare-card>#prepareState{grid-column:1/-1}.prepare-card>#prepareButton{grid-column:1/-1;height:50px;font-size:16px}.prepare-card>#lockButton,.prepare-card>button.stop{height:44px}.setup-stack{gap:16px}.fold.card{border-radius:20px;padding:18px 22px;background:rgba(255,255,255,.8);box-shadow:0 8px 28px rgba(38,54,82,.055)}.fold>summary{font-size:15px;letter-spacing:-.01em}.goodbox,.warning,.danger{background:var(--surface);border-color:var(--line)}.step-card{position:relative;overflow:hidden;padding:30px 32px}.step-card:before{content:'';position:absolute;left:0;top:28px;bottom:28px;width:4px;border-radius:0 4px 4px 0;background:linear-gradient(180deg,#6ba1ff,#b9d2ff)}#directionSection:before{background:linear-gradient(180deg,#f1ae52,#ffd99d)}#calibrationSection:before{background:linear-gradient(180deg,#57b887,#b9e6cd)}.status{border:1px solid #edf0f5;background:#f7f9fc;color:#445673;border-radius:13px;padding:11px 13px;min-height:42px}.row,.triple,.quad{gap:12px}input,select,button{height:42px;border-radius:12px;border-color:#dbe2ec}input,select{box-shadow:0 2px 7px rgba(40,57,84,.035)}input:focus,select:focus{outline:4px solid #1769e821;box-shadow:0 4px 16px rgba(23,105,232,.12)}button{box-shadow:0 5px 13px rgba(30,52,86,.1);font-size:14px}button:hover:not(:disabled){transform:translateY(-2px);box-shadow:0 9px 20px rgba(30,52,86,.14)}button.good{background:linear-gradient(135deg,#168451,#229d62)}button.warn{background:linear-gradient(135deg,#b87516,#d39428)}button.stop{background:linear-gradient(135deg,#c5362f,#e05148)}button.secondary{background:linear-gradient(135deg,#667085,#858fa0)}button.ghost{background:#f0f4f8;color:#30445f;box-shadow:none}.floating-status{max-width:1180px;padding:10px 24px;border-bottom:1px solid rgba(219,226,236,.9);background:rgba(247,249,252,.78);box-shadow:0 5px 22px rgba(38,54,82,.06)}.floating-motor{padding:8px 12px;border-radius:12px;background:rgba(255,255,255,.9);font-size:12px;letter-spacing:.01em}.motor-grid{gap:12px}.motor{border:1px solid #e6ebf2;border-radius:17px;padding:15px;background:linear-gradient(180deg,#fff,#fbfcfe);box-shadow:0 5px 16px rgba(38,54,82,.045)}.motor h3{margin-top:0}.metric{padding:6px 0;border-top-color:#edf1f6}.pill{padding:3px 8px}.canvas,canvas{display:block;border:1px solid #dce4ef;border-radius:22px;box-shadow:inset 0 0 0 8px rgba(247,249,252,.72),0 10px 25px rgba(38,54,82,.07);background-color:#fff;background-image:linear-gradient(rgba(61,104,166,.055) 1px,transparent 1px),linear-gradient(90deg,rgba(61,104,166,.055) 1px,transparent 1px);background-size:10% 10%}.advanced{display:none}details#logDetails #log{max-height:300px;overflow:auto;padding:14px;border-radius:14px;background:#111827;color:#dbeafe;font:12px/1.55 ui-monospace,SFMono-Regular,Consolas,monospace;white-space:pre-wrap}section#calibrationSection h3{margin-top:22px}#trajectoryState{font-weight:650}#prepareState{font-weight:650}
@media(max-width:820px){main{padding:18px 14px 54px;gap:14px}.card{padding:20px;border-radius:20px}.hero,.prepare-card,.step-card{padding:24px 20px}.prepare-card{grid-template-columns:1fr}.prepare-card>#lockButton,.prepare-card>button.stop{grid-column:1/-1}.row,.triple{grid-template-columns:1fr 1fr}.quad{grid-template-columns:repeat(2,1fr)}.motor-grid{grid-template-columns:repeat(2,1fr)}.floating-status{padding:8px 14px}}
@media(max-width:520px){h1{font-size:31px}h2{font-size:21px}.row,.triple,.quad{grid-template-columns:1fr}.motor-grid{grid-template-columns:1fr 1fr}.floating-motor{padding:7px 8px;font-size:11px}.x-banner{font-size:13px}}
</style>
<style>
.quick-connect{grid-column:1/-1;display:grid;grid-template-columns:minmax(190px,1.4fr) minmax(180px,1fr) auto;gap:10px;align-items:end;padding:13px 15px;border:1px solid #dce8f7;border-radius:16px;background:linear-gradient(100deg,#f4f8ff,#fff)}.quick-connect strong{display:block;font-size:14px}.quick-connect select,.quick-connect button{margin:0;height:42px}.quick-connect button{width:auto;padding:0 18px;white-space:nowrap}
@media(max-width:820px){.quick-connect{grid-template-columns:1fr 1fr}.quick-connect>div{grid-column:1/-1}.quick-connect button{width:100%}}
@media(max-width:520px){.quick-connect{grid-template-columns:1fr}.quick-connect>div{grid-column:auto}}
</style>
</head>
<body><div id="floatingStatus" class="floating-status"><div class="floating-motor">M1 ● 待回包</div><div class="floating-motor">M2 ● 待回包</div><div class="floating-motor">M3 ● 待回包</div><div class="floating-motor">M4 ● 待回包</div></div><main>
<section class="card full hero"><div class="step-kicker">Y42 · DESKTOP CONTROL</div><h1>校准与绘画</h1><div class="muted">电脑 → USB-RS485 → 四台 Y42。固定映射：<span class="mapping">M1 左上 / M2 右上 / M3 右下 / M4 左下</span>。当前只保留校准、手绘、连接和急停。</div><div class="x-banner">⚠️ 校准前必须确认四台实际都是 <span class="blink">X 固件</span>（绿灯一闪一闪）　·　网页运动按键默认禁用　·　空格键 = 四台急停并失能</div></section>
<section id="prepareSection" class="card full warning prepare-card"><div class="step-kicker">STEP 1 · READY</div><h2>一键自动准备</h2><div class="muted">服务启动后会自动扫描 ID 1–4、检查并切换 X 固件、锁定电机本体按键，最后让四台电机全部失能；这一步不使能、不移动电机。通讯恢复或网页重载也不会重复询问已确认的机构。</div><div class="quick-connect"><div><div class="step-kicker">CONNECTION</div><strong>USB-RS485 串口</strong><div class="muted">可自动选择，也可以手动指定</div></div><select id="quickPort" onchange="syncPortSelects(this.value)"><option value="">自动选择可用串口</option></select><button class="secondary" onclick="refreshPorts()">刷新串口</button></div><button id="prepareButton" class="warn" onclick="prepareSystem()">重新连接并自动准备</button><button id="lockButton" class="secondary" onclick="toggleControlLock()">运动按钮当前已锁定</button><button class="stop" onclick="stopAll()">急停（始终可用）</button><div id="prepareState" class="status">正在执行无运动自动准备……</div></section>
<section class="setup-stack">
  <details id="deviceTools" class="card fold"><summary>设备设置与故障排查</summary>
  <div class="card compact-card"><h2>自动连接与急停</h2><label>串口（可选）</label><select id="port" onchange="syncPortSelects(this.value)"><option value="">自动选择可用串口</option></select><label>波特率</label><input id="baud" type="number" value="115200"><div class="row"><button class="secondary" onclick="refreshPorts()">自动连接 / 刷新串口</button><button class="secondary" onclick="refreshStatus(true)">读取四电机状态</button></div><div id="portState" class="status">启动时自动选择唯一可用串口。</div><button class="stop" onclick="stopAll()">四台急停</button></div>
  <div id="firmwareSection" class="card goodbox"><h2>固件状态（0x42 实际检测）</h2><div class="muted">网页优先读取 0x42 配置块长度判断当前实际固件：0x25=X，0x21=Emm。确认四台都是 X、绿灯一闪一闪后再校准。</div><div class="row"><button class="secondary" onclick="firmwareStatus()">重新读取固件状态（只读）</button><button class="warn" onclick="switchFirmware('x')">一键切换全部为 X 并重启</button></div><div id="firmwareState" class="status">正在读取实际固件状态……</div></div>
  <div id="idSection" class="card warning"><h2>ID 扫描与修改</h2><div class="muted"><b>扫描只读：</b>默认扫描 0–20；ID 0 是广播地址，网页不会向 0 发送帧，实际安全扫描 ID 1–20。<br><b>改 ID：</b>总线上必须只接目标电机，避免同 ID 多台一起改名；改完请断电重启，再扫描确认。</div><div class="row"><div><label>扫描起始 ID</label><input id="scanStart" type="number" min="0" max="255" value="0"></div><div><label>扫描结束 ID</label><input id="scanEnd" type="number" min="1" max="255" value="20"></div></div><button id="scanButton" class="secondary" onclick="scanIds()">自动扫描 0–20 ID（只读）</button><div id="idState" class="status">等待扫描；扫描不会使电机运动。</div><div class="row"><div><label>旧 ID</label><input id="oldId" type="number" min="1" max="255" value="1"></div><div><label>新 ID</label><input id="newId" type="number" min="1" max="255" value="5"></div></div><button class="warn" onclick="setId()">确认并设置新 ID（只改参数，不运动）</button></div>
  </details>
  <div class="card goodbox compact-card"><h2>运行保护</h2><div class="muted">自动准备完成后运动按钮才会开放；电机默认失能。急停始终可用，方向测试和四角校准结束后都会再次失能。</div><div id="controlState" class="status">运动按钮已锁定</div></div>
  <div id="directionSection" class="card warning step-card"><div class="step-kicker">STEP 2 · DIRECTION</div><h2>逐台确认正方向</h2><div class="muted">四个 ID 同时显示。点击哪台，就只让哪台按当前候选方向转固定 45°；到位后立即停止并失能，不复位、不回原点。观察机构方向后，在对应卡片上直接保存“方向正确”或“需要反向”。</div><div id="directionMotorGrid" class="motor-grid"><div class="motor"><h3>M1 左上 <span class="muted">/ ID 1</span></h3><div id="directionCandidate1" class="status">驱动器方向 0（0=正，1=反）</div><button id="directionTestButton1" class="warn" data-motion data-direction-test="1" disabled onclick="startDirectionTest(1)">M1 · 正转 45°</button><div class="row"><button id="directionCorrectButton1" class="good" data-motion data-direction-save="1" data-reverse="0" disabled onclick="saveDirection(1,false)">方向正确</button><button id="directionReverseButton1" class="secondary" data-motion data-direction-save="1" data-reverse="1" disabled onclick="saveDirection(1,true)">需要反向</button></div><div id="directionState1" class="status">待测试</div></div><div class="motor"><h3>M2 右上 <span class="muted">/ ID 2</span></h3><div id="directionCandidate2" class="status">驱动器方向 1（0=正，1=反）</div><button id="directionTestButton2" class="warn" data-motion data-direction-test="2" disabled onclick="startDirectionTest(2)">M2 · 正转 45°</button><div class="row"><button id="directionCorrectButton2" class="good" data-motion data-direction-save="2" data-reverse="0" disabled onclick="saveDirection(2,false)">方向正确</button><button id="directionReverseButton2" class="secondary" data-motion data-direction-save="2" data-reverse="1" disabled onclick="saveDirection(2,true)">需要反向</button></div><div id="directionState2" class="status">待测试</div></div><div class="motor"><h3>M3 右下 <span class="muted">/ ID 3</span></h3><div id="directionCandidate3" class="status">驱动器方向 0（0=正，1=反）</div><button id="directionTestButton3" class="warn" data-motion data-direction-test="3" disabled onclick="startDirectionTest(3)">M3 · 正转 45°</button><div class="row"><button id="directionCorrectButton3" class="good" data-motion data-direction-save="3" data-reverse="0" disabled onclick="saveDirection(3,false)">方向正确</button><button id="directionReverseButton3" class="secondary" data-motion data-direction-save="3" data-reverse="1" disabled onclick="saveDirection(3,true)">需要反向</button></div><div id="directionState3" class="status">待测试</div></div><div class="motor"><h3>M4 左下 <span class="muted">/ ID 4</span></h3><div id="directionCandidate4" class="status">驱动器方向 1（0=正，1=反）</div><button id="directionTestButton4" class="warn" data-motion data-direction-test="4" disabled onclick="startDirectionTest(4)">M4 · 正转 45°</button><div class="row"><button id="directionCorrectButton4" class="good" data-motion data-direction-save="4" data-reverse="0" disabled onclick="saveDirection(4,false)">方向正确</button><button id="directionReverseButton4" class="secondary" data-motion data-direction-save="4" data-reverse="1" disabled onclick="saveDirection(4,true)">需要反向</button></div><div id="directionState4" class="status">待测试</div></div></div><div id="directionSaved" class="status">已保存方向：M1=0，M2=1，M3=0，M4=1</div></div>
  <div class="card advanced unavailable"><h2>气泵 / 阀门</h2><div>原 ESP32 页面中的充气、放气、GPIO47/48/45 独立输出依赖 ESP32 GPIO。当前链路只有电脑 USB-RS485，没有 ESP32 或 GPIO 控制器，因此这些按钮不能安全迁移，已禁用。</div></div>
</section>
<section>
  <details id="liveDetails" class="card fold"><summary>实时状态 <span class="muted">展开查看完整遥测</span></summary>
  <div class="compact-card"><div class="step-kicker">LIVE · RS485</div><h2>实时状态</h2><div id="motorGrid" class="motor-grid"></div><div id="autoState" class="status">自动流程：未启动</div></div></details>
  <div class="card advanced"><h2>单电机调试</h2><label>目标电机</label><select id="motorId"><option value="1">M1 左上 / ID 1</option><option value="2">M2 右上 / ID 2</option><option value="3">M3 右下 / ID 3</option><option value="4">M4 左下 / ID 4</option></select><div class="quad"><button class="good" data-motion disabled onclick="motorSimple('enable')">使能</button><button class="secondary" data-motion disabled onclick="motorSimple('disable')">失能</button><button class="ghost" data-motion disabled onclick="motorSimple('clear')">清保护</button><button class="ghost" data-motion disabled onclick="motorSimple('release')">释放</button></div><div class="row"><button class="ghost" data-motion disabled onclick="motorSimple('abort-home')">停止回零</button><button class="ghost" data-motion disabled onclick="motorSimple('set-home')">当前位置设零</button></div><label>回零模式 0..5</label><div class="row"><select id="homeMode"><option value="0">0</option><option value="1">1</option><option value="2">2</option><option value="3">3</option><option value="4">4</option><option value="5">5</option></select><button class="warn" data-motion disabled onclick="motorCommand('home')">执行回零</button></div><h3>速度模式</h3><div class="triple"><div><label>方向</label><select id="speedDirection"><option value="0">0</option><option value="1">1</option></select></div><div><label>RPM</label><input id="speedRpm" type="number" min="0" max="3000" step="0.1" value="5"></div><div><label>加速度</label><input id="speedAcc" type="number" min="0" max="65535" value="100"></div></div><button class="warn" data-motion disabled onclick="motorCommand('speed')">发送速度</button><h3>力矩模式</h3><div class="quad"><div><label>方向</label><select id="torqueDirection"><option value="0">0</option><option value="1">1</option></select></div><div><label>电流 mA</label><input id="torqueCurrent" type="number" min="0" max="5000" value="100"></div><div><label>斜率</label><input id="torqueSlope" type="number" min="0" max="65535" value="2000"></div><div><label>最大 RPM</label><input id="torqueMaxRpm" type="number" min="0.1" max="3000" step="0.1" value="10"></div></div><button class="warn" data-motion disabled onclick="motorCommand('torque')">发送力矩</button><h3>位置模式（Y42 X）</h3><div class="quad"><div><label>方向</label><select id="positionDirection"><option value="0">0</option><option value="1">1</option></select></div><div><label>角度 °</label><input id="positionDegrees" type="number" min="0" max="429496729.5" step="0.1" value="5"></div><div><label>RPM</label><input id="positionRpm" type="number" min="0.1" max="3000" step="0.1" value="5"></div><div><label>加速度</label><input id="positionAcc" type="number" min="0" max="65535" value="100"></div></div><label>位置模式 0..2</label><div class="row"><select id="positionMode"><option value="2">2（相对当前实时位置）</option><option value="0">0</option><option value="1">1</option></select><button class="warn" data-motion disabled onclick="motorCommand('position')">发送位置</button></div></div>
  <div class="card advanced"><h2>原始 RS485 帧</h2><div class="muted">高级用户使用；例如 <code>01 3A 6B</code>。发送前请确认帧内容和机构安全。</div><input id="rawFrame" value="01 3A 6B"><button class="warn" data-motion disabled onclick="rawCommand()">发送原始帧</button></div>
</section>
<section id="calibrationSection" class="card full step-card"><div class="step-kicker">STEP 3 · GEOMETRY</div><h2>四角几何校准</h2><div class="muted">参数会保存在电脑本地 JSON。四角校准只采集几何，不会再次做单台编码器校准；切换角点时四台保持使能，非主拉电机默认保持 300mA。</div><div class="triple"><div><label>上下中心距 mm</label><input id="calSide" type="number" min="50" max="1000" value="150"></div><div><label>小球直径 mm</label><input id="calBall" type="number" min="1" value="40"></div><div><label>目标强电流 mA</label><input id="calStrong" type="number" min="50" max="3000" value="1500"></div></div><div class="row"><div><label>其余电机张力 mA</label><input id="calWeak" type="number" min="20" value="300"></div><div><label>轨迹位置电流上限 mA</label><input id="calTrajectoryCurrent" type="number" min="300" max="3000" value="2000"></div></div><h3>四台斜率独立微调（百分比，保存后立即用于轨迹反算）</h3><div class="quad"><div><label>M1 微调 %</label><input id="slopeTrim1" type="number" min="-20" max="20" step="0.1" value="0"></div><div><label>M2 微调 %</label><input id="slopeTrim2" type="number" min="-20" max="20" step="0.1" value="0"></div><div><label>M3 微调 %</label><input id="slopeTrim3" type="number" min="-20" max="20" step="0.1" value="0"></div><div><label>M4 微调 %</label><input id="slopeTrim4" type="number" min="-20" max="20" step="0.1" value="0"></div></div><div class="row"><button class="secondary" data-motion disabled onclick="saveCalibrationSettings()">保存校准参数 / 斜率微调</button><button class="warn" data-motion disabled onclick="calibrate()">开始四角自动校准</button><button class="stop" onclick="autoAction('abort')">停止校准 / 自动流程</button></div><div><label>当前校准系数</label><div id="calibrationInfo" class="status">未校准</div></div></section>
<section class="card full step-card"><div class="step-kicker">STEP 4 · DRAW</div><h2>绘画轨迹</h2><div class="muted">先完成四角校准，再在画布上绘制轨迹。绿色线是当前发送的目标轨迹，蓝色线是你的手绘输入；控制会按平滑后的目标轨迹运行。</div><div class="row"><div><canvas id="canvas" width="520" height="520"></canvas><div class="muted">图例：<span style="color:#2563eb">蓝色=手绘输入</span>　<span style="color:#16a34a">绿色=目标轨迹</span></div><div class="row"><button class="ghost" onclick="clearPath()">清空手绘</button><button class="warn" data-motion disabled onclick="runCustom()">运行手绘轨迹</button></div></div><div><label>手绘速度 mm/s</label><input id="pathSpeed" type="number" min="30" max="5000" value="200"><label class="check"><input id="repeat" type="checkbox">手绘轨迹循环</label><div class="quad"><button class="advanced warn" data-motion disabled onclick="autoAction('circle')">测试圆形</button><button class="advanced warn" data-motion disabled onclick="autoAction('triangle')">测试三角形</button><button class="advanced secondary" data-motion disabled onclick="autoAction('rectangle')">测试矩形</button><button class="advanced secondary" data-motion disabled onclick="autoAction('cross')">测试十字</button><button class="advanced secondary" data-motion disabled onclick="autoAction('loop_path')">循环上次手绘轨迹</button><button class="advanced secondary" data-motion disabled onclick="autoAction('stop_loop')">停止后续循环</button><button class="stop" onclick="autoAction('abort')">轨迹急停</button></div><div id="trajectoryState" class="status">未运行。</div></div></div></section>
<details id="logDetails" class="card full fold"><summary>过程与通讯日志 <span class="muted">轨迹目标、实际位置、电流、频率和错误会自动保存</span></summary><div class="row"><button class="ghost" onclick="loadProcessLog()">读取最近过程数据</button><button class="ghost" onclick="window.open('/api/process-log','_blank')">打开 JSONL 过程文件</button></div><div id="log">新版电脑直连完整调试台已加载。</div></details>
</main>
<script>
const $=id=>document.getElementById(id),logEl=$('log');let points=[],trajectoryTarget=[],drawing=false,controlsLocked=true,sessionSafetyConfirmed=false,scanBusy=false,statusPollInFlight=false,directionBusy=false,spaceHeld=false,latestAuto={},motorDisplayCache={};
function log(v){logEl.textContent=typeof v==='string'?v:JSON.stringify(v,null,2)}
	function port(){return $('quickPort')?$('quickPort').value:$('port').value}function syncPortSelects(value){['port','quickPort'].forEach(id=>{const el=$(id);if(el)el.value=value})}function baud(){return +$('baud').value||115200}
async function getJson(url,timeoutMs=5000){const c=new AbortController(),timer=setTimeout(()=>c.abort(),timeoutMs);try{const r=await fetch(url,{signal:c.signal});const j=await r.json();if(j.ok===false)throw Error(j.error||'读取失败');return j}catch(e){if(e.name==='AbortError')throw Error('状态读取超时；后台不会叠加请求，正在自动重连')}finally{clearTimeout(timer)}}
async function post(body,url='/api/command',timeoutMs=15000){const c=new AbortController(),timer=setTimeout(()=>c.abort(),timeoutMs);try{const r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({...body,port:port(),baud:baud(),armed:body.armed===undefined?!controlsLocked:body.armed}),signal:c.signal});const j=await r.json();if(!j.ok)throw Error(j.error||'操作失败');return j}catch(e){if(e.name==='AbortError')throw Error('操作超时；请查看状态和通讯日志，网页不会重复发送')}finally{clearTimeout(timer)}}
function showPrepareFailure(j){const targetId=j.stage==='ids'?'idSection':j.stage==='firmware'?'firmwareSection':j.stage==='keys'||j.stage==='disable'?'prepareSection':'prepareSection';const target=$(targetId);if(target){const fold=target.closest('details');if(fold)fold.open=true;target.classList.remove('attention');void target.offsetWidth;target.classList.add('attention');target.scrollIntoView({behavior:'smooth',block:'center'});setTimeout(()=>target.classList.remove('attention'),3500)}}
async function prepareSystem(){const btn=$('prepareButton');if(btn.disabled)return;sessionSafetyConfirmed=true;btn.disabled=true;$('prepareState').textContent='正在自动准备：扫描 ID 1–4、检查 X 固件并失能，请等待……';try{const j=await post({action:'prepare',confirmed:true});log(j);if(j.ready){controlsLocked=false;updateControlLock();$('prepareState').textContent=j.message}else{controlsLocked=true;updateControlLock();$('prepareState').textContent=j.message+(j.stage==='ids'?'；请处理缺失 ID 后重新点击一键准备。':j.stage==='firmware'?'；请检查固件回包/接线后重试。':'；请检查 RS485 回包后重试。');showPrepareFailure(j)}}catch(e){controlsLocked=true;updateControlLock();$('prepareState').textContent='自动准备失败：'+e.message+'；请检查串口、供电、A/B 接线后重试。';showPrepareFailure({stage:'prepare'});log('自动准备失败：'+e.message)}finally{btn.disabled=false}}
function safe(){if(sessionSafetyConfirmed)return true;alert('请先点击“确认机构安全，开始自动准备”，完成 ID 1–4 扫描、X 固件检查、按键锁定和四台失能。');return false}
 function updateControlLock(){document.querySelectorAll('[data-motion]').forEach(b=>b.disabled=controlsLocked);const busy=directionBusy||latestAuto.auto==='direction-test';document.querySelectorAll('[data-direction-test]').forEach(b=>b.disabled=controlsLocked||busy);document.querySelectorAll('[data-direction-save]').forEach(b=>{const id=b.dataset.directionSave;const ready=!!((latestAuto.direction_ready||{})[id]&&latestAuto.direction_ready[id].completed);b.disabled=controlsLocked||busy||!ready});$('controlState').textContent=controlsLocked?'控制按钮已锁定（急停始终可用）':'控制按钮已解锁（X 模式已确认）';$('lockButton').textContent=controlsLocked?'解锁控制按钮':'立即锁定控制按钮';$('lockButton').className=controlsLocked?'good':'secondary'}
async function toggleControlLock(){if(!controlsLocked){try{await post({armed:false},'/api/arm')}catch(e){}controlsLocked=true;sessionSafetyConfirmed=false;updateControlLock();return}if(!safe())return;try{await post({armed:true,confirmed:true},'/api/arm');controlsLocked=false;updateControlLock();log('控制按钮已解锁；X 固件协议保持锁定。')}catch(e){sessionSafetyConfirmed=false;log('解锁失败：'+e.message)}}
async function lockKeys(){if(!confirm('确认给 ID 1..4 写入并保存“锁定电机本体按键”？此操作不发送运动指令。'))return;try{const j=await post({action:'lock-keys',confirmed:true});$('keyLockState').textContent=j.message;log(j)}catch(e){$('keyLockState').textContent='物理按键锁定失败：'+e.message;log('物理按键锁定失败：'+e.message)}}
	async function refreshPorts(){try{const j=await getJson('/api/ports'),ports=j.ports||[],old=port(),selects=[$('port'),$('quickPort')].filter(Boolean);selects.forEach(select=>{select.innerHTML='<option value="">自动选择可用串口</option>';for(const p of ports){const o=document.createElement('option');o.value=p.device;o.textContent=p.label;select.appendChild(o)}});const selected=ports.some(p=>p.device===old)?old:ports.length===1?ports[0].device:'';syncPortSelects(selected);if(selected){const selectedLabel=$('quickPort').selectedOptions[0].textContent;$('portState').textContent='已选择 '+selectedLabel+'，正在测试连接……';const s=await post({action:'status'});const motors=s.motors||[];const online=motors.filter(x=>x.online).length;if(motors.length)renderMotors(motors);$('portState').textContent=online===4?'已连接 '+selected+'，四台电机全部在线。':'已打开 '+selected+'，在线 '+online+'/4；请检查 RS485 接线或固件模式。';log({ports,selected,online})}else{$('portState').textContent=ports.length?'发现 '+ports.length+' 个串口，请手动选择；未选择时后台不会盲目打开多个串口。':'没有发现 USB-RS485 串口。';log(j)}}catch(e){$('portState').textContent='自动连接失败：'+e.message;log('自动连接失败：'+e.message)}}
function fmt(v,s=''){return v===null||v===undefined?'待回包':Number.isFinite(+v)?(+v).toFixed(Math.abs(v)<100?1:0)+s:v+s}
function stableMotors(ms){const fields=['voltage','bus_current','phase_current','rpm','position','temperature'];return (ms||[]).map(t=>{const id=String(t.id),old=motorDisplayCache[id]||{},out={...old,...t};fields.forEach(k=>{if((t[k]===null||t[k]===undefined)&&old[k]!==null&&old[k]!==undefined)out[k]=old[k]});if(t.online===false){out.online=false;out.flags=null;out.flags_text='未响应'}motorDisplayCache[id]=out;return out})}
function renderFloating(ms){const stable=stableMotors(ms),map=[{id:1,n:'M1'},{id:2,n:'M2'},{id:3,n:'M3'},{id:4,n:'M4'}];$('floatingStatus').innerHTML=map.map(m=>{const t=stable.find(x=>+x.id===m.id)||{id:m.id,online:false};return '<div class="floating-motor '+(t.online?'online':'offline')+'">'+m.n+' '+(t.online?'●在线':'●离线')+' · I '+fmt(t.phase_current,'mA')+' · P '+fmt(t.position,'°')+'</div>'}).join('')}
function renderMotors(ms){const stable=stableMotors(ms),map=[{id:1,n:'M1 左上'},{id:2,n:'M2 右上'},{id:3,n:'M3 右下'},{id:4,n:'M4 左下'}];$('motorGrid').innerHTML=map.map(m=>{const t=stable.find(x=>+x.id===m.id)||{id:m.id,online:false};return '<div class="motor"><h3>'+m.n+' / ID '+m.id+' <span class="'+(t.online?'online':'offline')+'">● '+(t.online?'在线':'无响应')+'</span></h3><div class="metric"><span>母线电压</span><b>'+fmt(t.voltage,' V')+'</b></div><div class="metric"><span>母线电流</span><b>'+fmt(t.bus_current,' mA')+'</b></div><div class="metric"><span>相电流</span><b>'+fmt(t.phase_current,' mA')+'</b></div><div class="metric"><span>转速</span><b>'+fmt(t.rpm,' RPM')+'</b></div><div class="metric"><span>位置</span><b>'+fmt(t.position,' °')+'</b></div><div class="metric"><span>温度</span><b>'+fmt(t.temperature,' °C')+'</b></div><div class="metric"><span>状态</span><b>'+(t.flags===null||t.flags===undefined?'等待回包':('0x'+Number(t.flags).toString(16).padStart(2,'0').toUpperCase()))+'</b></div><div class="small">'+(t.flags_text||'等待回包')+'</div></div>'}).join('');renderFloating(stable)}
async function refreshStatus(force=false){if(scanBusy||statusPollInFlight)return;statusPollInFlight=true;try{const j=await getJson('/api/status?x='+Date.now(),2600);if(j.motors)renderMotors(j.motors);if(j.status_deferred){$('portState').textContent='状态读取暂缓：其他串口操作完成后自动恢复'}else if(j.reconnected===false){$('portState').textContent='已打开 '+port()+'，但电机回包异常：'+(j.message||'无有效回包')}else if(j.reconnected===true){$('portState').textContent='已连接 '+port()+'，四台状态正常（已自动重连）'}if(force||j.reconnected===false&&!j.status_deferred&&j.message)log(j)}catch(e){$('portState').textContent='串口状态读取超时：后台已停止叠加请求，正在自动重连';if(force)log('读取状态失败：'+e.message)}finally{statusPollInFlight=false}}
async function firmwareStatus(showLog=true){try{const j=await post({action:'firmware-status'});$('firmwareState').innerHTML=j.results.map(x=>'M'+x.id+' '+x.label+'：'+(x.firmware?'<span class="pill ok">'+x.firmware+'</span>':'<span class="pill bad">无响应</span>')).join('　');if(showLog)log(j);return j}catch(e){$('firmwareState').textContent='读取失败：'+e.message;if(showLog)log('读取固件类型失败：'+e.message)}}
function firmwareSafe(){return safe()}
async function switchFirmware(target){if(target!=='x'||!firmwareSafe())return;try{log('正在急停并逐台切换为 X 固件类型，请等待……');const j=await post({action:'firmware-x',confirmed:true});log(j);await firmwareStatus(false);refreshStatus(true)}catch(e){log('切换 X 固件失败：'+e.message)}}
async function stabilityTest(){try{log('正在连续测试四个 ID 的 0x3A 状态查询，请等待……');log(await post({action:'stability',rounds:20}))}catch(e){log('稳定性测试失败：'+e.message)}}
async function baudSweep(){try{log('正在用更长等待时间诊断常用波特率，请等待……');log(await post({action:'baud-sweep',bauds:[115200,57600,38400,19200,9600,460800]}))}catch(e){log('波特率诊断失败：'+e.message)}}
 function renderDirectionInfo(j){const c=j.calibration||{},reverse=Array.isArray(c.direction_reverse)&&c.direction_reverse.length===4?c.direction_reverse:[false,false,false,false],base=[0,1,0,1],saved=base.map((v,i)=>v^(reverse[i]?1:0)),readyMap=j.direction_ready||{},runningId=+j.direction_motor||0,directionRunning=j.auto==='direction-test';directionBusy=directionRunning;for(let i=1;i<=4;i++){const ready=readyMap[String(i)]&&readyMap[String(i)].completed,delta=readyMap[String(i)]?.delta;$('directionCandidate'+i).textContent='驱动器方向 '+saved[i-1]+'（0=正，1=反）';$('directionState'+i).textContent=ready?'已转 45°'+(Number.isFinite(+delta)?'（实际 '+(+delta>=0?'+':'')+Number(delta).toFixed(1)+'°）':'')+'，当前位置已停止；请选择方向保存':directionRunning&&runningId===i?(j.message||'正在测试……'):j.auto==='error'&&runningId===i?(j.message||'测试失败，请检查后重试'):'待测试'}$('directionSaved').textContent='已保存方向：'+saved.map((v,i)=>'M'+(i+1)+'='+v).join('，');updateControlLock()}
 async function startDirectionTest(id){if(!safe()||directionBusy)return;directionBusy=true;updateControlLock();$('directionState'+id).textContent='正在测试：8 RPM，仅转 45°，到位后停止并失能……';try{log(await post({action:'direction-test',id,confirmed:true}));await refreshAuto()}catch(e){directionBusy=false;updateControlLock();$('directionState'+id).textContent='正方向测试失败：'+e.message;log('正方向测试失败：'+e.message)}}
 async function saveDirection(id,reverse){if(!safe())return;const ready=(latestAuto.direction_ready||{})[String(id)];if(!ready||!ready.completed){alert('请先完成 M'+id+' 的 45°方向测试。');return}try{const j=await post({action:'direction-save',id,reverse,confirmed:true});log(j);await refreshAuto()}catch(e){$('directionState'+id).textContent='保存方向失败：'+e.message;log('保存方向失败：'+e.message)}}
 function renderAuto(j){latestAuto=j;controlsLocked=!j.control_unlocked;sessionSafetyConfirmed=!!j.control_unlocked;updateControlLock();const c=j.calibration||{};const setIfBlurred=(id,value)=>{const el=$(id);if(el&&document.activeElement!==el&&value!==undefined&&value!==null)el.value=value};setIfBlurred('calSide',c.side);setIfBlurred('calBall',c.ball);setIfBlurred('calStrong',c.strong_current??1500);setIfBlurred('calWeak',c.weak_current??300);setIfBlurred('calTrajectoryCurrent',c.trajectory_current??2000);(c.slope_trim_percent||[0,0,0,0]).forEach((v,i)=>setIfBlurred('slopeTrim'+(i+1),v));$('autoState').textContent='自动流程：'+j.auto+'｜'+j.message;$('trajectoryState').textContent=(j.message||'未运行')+(Number.isFinite(+j.trajectory_actual_error_mm)?' · 实际轨迹反算误差 '+Number(j.trajectory_actual_error_mm).toFixed(1)+'mm':'');const valid=!!c.valid;if(valid){const trims=c.slope_trim_percent||[0,0,0,0];$('calibrationInfo').innerHTML='<span class="pill ok">已校准</span> 工作区 '+Number(c.workspace_width??c.effective_side).toFixed(1)+' × '+Number(c.workspace_height??c.effective_side).toFixed(1)+' mm<br>公共斜率绝对值 '+Number(c.common_slope_abs||Math.max(...c.slope.map(Math.abs))).toFixed(4)+'°/mm<br>'+c.slope.map((s,i)=>'M'+(i+1)+': 公共系数 '+Number(s).toFixed(4)+'°/mm，微调 '+Number(trims[i]||0).toFixed(1)+'%，实际 '+Number(s*(1+Number(trims[i]||0)/100)).toFixed(4)+'°/mm，截距 '+Number(c.intercept[i]).toFixed(1)+'°，R² '+Number(c.r2[i]).toFixed(3)).join('<br>')}else{$('calibrationInfo').innerHTML='<span class="pill bad">未校准</span> '+(c.quality_error||'请先完成四角自动校准')}renderDirectionInfo(j);renderTrajectoryPaths(j)}
	function renderTrajectoryPaths(j){const c=j.calibration||{},w=Number(c.workspace_width||c.effective_side||140),h=Number(c.workspace_height||c.effective_side||140);trajectoryTarget=(j.trajectory_target_path||[]).map(p=>({x:Number(p[0])/w,y:1-Number(p[1])/h}));paint()}async function refreshAuto(){try{const j=await getJson('/api/auto');renderAuto(j);if(j.telemetry)renderMotors(Object.values(j.telemetry))}catch(e){}}
async function allAction(action){if(!safe())return;try{log(await post({action,confirmed:true}))}catch(e){log(action+' 失败：'+e.message)}finally{refreshStatus(true)}}
async function stopAll(){try{log(await post({action:'stop'}))}catch(e){log('急停失败：'+e.message)}finally{refreshStatus(true)}}
window.addEventListener('keydown',e=>{if(e.code!=='Space')return;e.preventDefault();if(e.repeat||spaceHeld)return;spaceHeld=true;$('trajectoryState').textContent='空格键急停：正在停止并失能四台电机……';void stopAll()});window.addEventListener('keyup',e=>{if(e.code==='Space'){e.preventDefault();spaceHeld=false}});window.addEventListener('blur',()=>{spaceHeld=false});
function idConfigSafe(){return sessionSafetyConfirmed}
async function scanIds(){if(scanBusy)return;const start=+$('scanStart').value,end=+$('scanEnd').value;if(!Number.isInteger(start)||!Number.isInteger(end)||start<0||end>255||start>end){alert('扫描范围必须是 0..255，且起始不大于结束。');return}scanBusy=true;$('scanButton').disabled=true;$('idState').textContent='正在扫描 ID '+start+'..'+end+'，实时状态刷新已暂停，请等待……';try{const j=await post({action:'scan',start,end});$('idState').textContent=j.message+'；发现：'+(j.found&&j.found.length?j.found.join(', '):'无');log(j)}catch(e){$('idState').textContent='扫描失败：'+e.message;log('扫描 ID 失败：'+e.message)}finally{scanBusy=false;$('scanButton').disabled=false;refreshStatus(true)}}
async function setId(){if(!idConfigSafe()){alert('请先点击“确认机构安全，开始自动准备”；缺失 ID 时可在自动准备失败后直接回到这里修改。');return}const old=+$('oldId').value,newId=+$('newId').value;if(!Number.isInteger(old)||!Number.isInteger(newId)||old<1||newId<1||old>255||newId>255||old===newId){alert('旧 ID、新 ID 必须是不同的 1..255 整数。');return}try{const j=await post({action:'set-id',old,new:newId,confirmed:true});$('idState').textContent=j.message;controlsLocked=true;sessionSafetyConfirmed=false;updateControlLock();$('prepareState').textContent='ID 已修改；请断电重启电机后重新点击“一键自动准备”。';log(j)}catch(e){$('idState').textContent='设置 ID 失败：'+e.message;log('设置 ID 失败：'+e.message)}}
async function motorSimple(action){if(action!=='disable'&&action!=='clear'&&action!=='release'&&action!=='abort-home'&&action!=='set-home'&&!safe())return;if(['clear','release','abort-home','set-home'].includes(action)&&!confirm('确认对选中的电机发送 '+action+'？'))return;try{log(await post({action,id:+$('motorId').value,confirmed:true}))}catch(e){log(action+' 失败：'+e.message)}finally{refreshStatus(true)}}
async function motorCommand(action){if(!safe())return;const b={action,id:+$('motorId').value,confirmed:true};if(action==='home')b.mode=+$('homeMode').value;if(action==='speed'){b.direction=+$('speedDirection').value;b.rpm=+$('speedRpm').value;b.acceleration=+$('speedAcc').value}if(action==='torque'){b.direction=+$('torqueDirection').value;b.current=+$('torqueCurrent').value;b.slope=+$('torqueSlope').value;b.max_rpm=+$('torqueMaxRpm').value}if(action==='position'){b.direction=+$('positionDirection').value;b.degrees=+$('positionDegrees').value;b.rpm=+$('positionRpm').value;b.acceleration=+$('positionAcc').value;b.mode=+$('positionMode').value}try{log(await post(b))}catch(e){log(action+' 失败：'+e.message)}finally{refreshStatus(true)}}
async function rawCommand(){if(!safe())return;try{log(await post({action:'raw',hex_frame:$('rawFrame').value,confirmed:true}))}catch(e){log('原始帧失败：'+e.message)}}
async function saveCalibrationSettings(){if(!safe())return;try{const j=await post({action:'calibration-settings',side:+$('calSide').value,ball:+$('calBall').value,strong:+$('calStrong').value,weak:+$('calWeak').value,trajectory_current:+$('calTrajectoryCurrent').value,slope_trim_percent:[1,2,3,4].map(i=>+$('slopeTrim'+i).value),confirmed:true},'/api/auto');log(j);await refreshAuto()}catch(e){log('保存校准参数失败：'+e.message)}}
async function calibrate(){if(!safe())return;try{const j=await post({action:'calibrate',side:+$('calSide').value,ball:+$('calBall').value,strong:+$('calStrong').value,weak:+$('calWeak').value,trajectory_current:+$('calTrajectoryCurrent').value,confirmed:true},'/api/auto');log(j);await refreshAuto()}catch(e){log('启动校准失败：'+e.message)}}
async function autoAction(action){if(action!=='stop_loop'&&action!=='abort'&&!safe())return;try{log(await post({action,confirmed:true},'/api/auto'))}catch(e){log(action+' 失败：'+e.message)}}
async function loadProcessLog(){try{const j=await getJson('/api/process-log');log({file:j.file,events:j.events})}catch(e){log('读取过程数据失败：'+e.message)}}
function canvasPoint(e){const r=$('canvas').getBoundingClientRect();return{x:Math.max(0,Math.min(1,(e.clientX-r.left)/r.width)),y:Math.max(0,Math.min(1,(e.clientY-r.top)/r.height))}}
		function drawTrace(x,trace,color,width){if(!trace.length)return;const q=p=>[p.x*x.canvas.width,p.y*x.canvas.height];x.strokeStyle=color;x.lineWidth=width;x.lineJoin='round';x.lineCap='round';x.beginPath();if(trace.length===1){x.arc(...q(trace[0]),2,0,Math.PI*2);x.fillStyle=color;x.fill();return}x.moveTo(...q(trace[0]));for(let i=1;i<trace.length-1;i++){const p=q(trace[i]),n=q(trace[i+1]),mid=[(p[0]+n[0])/2,(p[1]+n[1])/2];x.quadraticCurveTo(p[0],p[1],mid[0],mid[1])}const last=q(trace[trace.length-1]);x.quadraticCurveTo(last[0],last[1],last[0],last[1]);x.stroke()}function paint(){const c=$('canvas'),x=c.getContext('2d');x.clearRect(0,0,c.width,c.height);x.strokeStyle='#9bb1cb';x.lineWidth=1;x.beginPath();x.moveTo(c.width/2,0);x.lineTo(c.width/2,c.height);x.moveTo(0,c.height/2);x.lineTo(c.width,c.height/2);x.stroke();drawTrace(x,trajectoryTarget,'#16a34a',3);if(points.length){x.strokeStyle='#2563eb';x.lineWidth=4;x.beginPath();points.forEach((p,i)=>i?x.lineTo(p.x*c.width,p.y*c.height):x.moveTo(p.x*c.width,p.y*c.height));x.stroke()}}
const canvas=$('canvas');canvas.addEventListener('pointerdown',e=>{drawing=true;points=[canvasPoint(e)];canvas.setPointerCapture(e.pointerId);paint()});canvas.addEventListener('pointermove',e=>{if(!drawing)return;const p=canvasPoint(e);if(!points.length||Math.hypot(p.x-points.at(-1).x,p.y-points.at(-1).y)>0.004){points.push(p);paint()}});canvas.addEventListener('pointerup',()=>drawing=false);canvas.addEventListener('pointercancel',()=>drawing=false);function clearPath(){points=[];paint()}
async function runCustom(){if(!safe())return;if(points.length<2){alert('请先在画布上绘制至少两个点。');return}try{log(await post({action:'path',points,speed:+$('pathSpeed').value,repeat:$('repeat').checked,confirmed:true},'/api/path'))}catch(e){log('手绘轨迹失败：'+e.message)}}
 async function boot(){updateControlLock();await refreshPorts();await refreshAuto();paint();setInterval(()=>{refreshAuto()},250);setInterval(()=>{refreshStatus()},1200)}boot();
</script></body></html>'''


def api_error(exc: Exception) -> dict:
    return {"ok": False, "error": str(exc)}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, _format: str, *_args: object) -> None:
        return

    def send_json(self, value: object, status: int = 200) -> None:
        body = json_bytes(value)
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        path = self.path.split("?", 1)[0]
        try:
            if path == "/":
                body = PAGE.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if path == "/api/ports":
                self.send_json(execute({"action": "ports"}))
                return
            if path == "/api/status":
                self.send_json(execute({"action": "status", "port": current_port(), "baud": current_baud()}))
                return
            if path == "/api/auto":
                self.send_json(state_snapshot())
                return
            if path == "/api/process-log":
                self.send_json({"ok": True, "file": str(PROCESS_LOG_FILE), "events": recent_process_log()})
                return
            self.send_error(404)
        except Exception as exc:
            self.send_json(api_error(exc), 400)

    def do_POST(self) -> None:
        try:
            length = int(self.headers.get("Content-Length", "0"))
            data = json.loads(self.rfile.read(length).decode("utf-8") or "{}")
            path = self.path.split("?", 1)[0]
            if path == "/api/command":
                result = execute(data)
            elif path == "/api/arm":
                result = set_control_lock(data)
            elif path == "/api/auto":
                result = start_auto(data)
            elif path == "/api/path":
                result = handle_path(data)
            else:
                self.send_error(404)
                return
            self.send_json(result)
        except Exception as exc:
            self.send_json(api_error(exc), 400)


def run() -> None:
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"Y42 direct web: http://{HOST}:{PORT}/", flush=True)
    threading.Thread(target=auto_prepare_on_boot, name="y42-boot-prepare", daemon=True).start()
    server.serve_forever()


if __name__ == "__main__":
    run()
