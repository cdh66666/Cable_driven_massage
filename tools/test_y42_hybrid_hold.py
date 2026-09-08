#!/usr/bin/env python3
"""Low-energy hardware handshake for the 3-position + 1-torque 0xAA frame.

The three position motors are commanded to their current encoder values; M4 is
given only 50 mA pull torque for 0.30 s.  Every exit path stops and disables all
four motors.  This is a protocol/transition check, not a calibration motion.
"""

from __future__ import annotations

import json
import time

import y42_pc_direct as direct
import y42_pc_direct_web as web


def telemetry_dict(values: list[direct.Telemetry]) -> list[dict]:
    return [
        {
            "id": item.address,
            "position_deg": item.position,
            "rpm": item.rpm,
            "phase_current_ma": item.phase_current,
            "temperature_c": item.temperature,
            "flags": item.flags,
        }
        for item in values
    ]


def main() -> int:
    port = web.current_port() or "COM7"
    baud = web.current_baud() or 921600
    result: dict = {"port": port, "baud": baud, "passive_motor": 4, "torque_ma": 50}
    connection = None
    try:
        connection = direct.open_serial(port, baud)
        stopped = web.stop_all(connection)
        if not all(item.get("accepted") for item in stopped):
            raise RuntimeError("启动前急停未全部确认")
        before = web.read_all_reliable(connection, attempts=2)
        error = web.safety_error(before, current_limit=300)
        if error:
            raise RuntimeError("启动检查：" + error)
        targets = [item.position for item in before]
        if any(value is None for value in targets):
            raise RuntimeError("四台初始编码器位置不完整")
        for address in web.MOTOR_IDS:
            accepted, _, reply = web.enable_address(connection, address)
            if not accepted:
                raise RuntimeError(f"M{address} 使能失败: {direct.hex_bytes(reply)}")
        packet = web.three_position_one_torque_frame(
            [float(value) for value in targets],
            passive_index=3,
            passive_current_ma=50,
            rpms=[20.0] * 4,
            position_current=[300] * 4,
        )
        started = time.perf_counter()
        next_tick = started
        frames = 0
        while time.perf_counter() - started < 0.30:
            now = time.perf_counter()
            if now < next_tick:
                time.sleep(min(0.002, next_tick - now))
                continue
            web.send_fast(connection, packet)
            frames += 1
            next_tick += web.TRAJECTORY_INTERVAL
        after = web.read_all_reliable(connection, attempts=2)
        error = web.safety_error(after, current_limit=300)
        if error:
            raise RuntimeError("握手后检查：" + error)
        position_delta = [
            float(new.position) - float(old.position)
            for old, new in zip(before, after)
            if old.position is not None and new.position is not None
        ]
        result.update(
            {
                "ok": True,
                "frames": frames,
                "packet_bytes": len(packet),
                "before": telemetry_dict(before),
                "after": telemetry_dict(after),
                "position_delta_deg": position_delta,
                "max_position_delta_deg": max((abs(value) for value in position_delta), default=None),
            }
        )
    except Exception as exc:
        result.update({"ok": False, "error": str(exc)})
    finally:
        if connection is not None:
            try:
                result["disable"] = web.disable_all(connection)
            except Exception as exc:
                result["disable_error"] = str(exc)
            try:
                connection.close()
            except Exception:
                pass
        recovery = web.disable_all_with_reconnect(port, baud, announce=False)
        result["recovery"] = recovery
        if not recovery.get("ok"):
            result["ok"] = False
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
