"""Recorded, bounded diagnosis of the observed mixed-rate four-motor bus.

Run only with the web backend stopped. Default operation is read-only.
Address labels are bus addresses, not proof of physical motor identity.
"""
from __future__ import annotations

import json
from pathlib import Path
import socket
import time
from datetime import datetime
import uuid
import argparse
import hashlib
import shutil

import y42_pc_direct as direct
from y42_position_probe_io import _RecordedConnection


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--restore-x", action="store_true",
                        help="Stop/disable the four observed axes, then restore only the slow-bus axis to X firmware")
    parser.add_argument("--discover", action="store_true", help="Read both candidate rates/addresses, skipping absent replies")
    parser.add_argument("--restore-address-config", action="store_true",
                        help="Restore observed slow-bus X axis to archived ID4/921600 configuration, without motion")
    parser.add_argument("--restore-config-at-4", action="store_true",
                        help="Resume only configuration restore after read-only confirmation that ID4 is active at 115200")
    parser.add_argument("--acknowledge-power", action="store_true",
                        help="After fresh calibrated/stopped/disabled checks on the unified bus, acknowledge recorded power-loss flags")
    args = parser.parse_args()
    if sum((args.restore_x, args.discover, args.restore_address_config, args.restore_config_at_4, args.acknowledge_power)) > 1:
        parser.error("Discovery cannot be combined with a configuration write")
    try:
        with socket.create_connection(("127.0.0.1", 8787), timeout=0.3):
            raise RuntimeError("Web backend still owns hardware; no serial port opened")
    except (ConnectionRefusedError, TimeoutError):
        pass
    root = Path(__file__).parent / "manual_test_runs" / (
        datetime.now().strftime("%Y%m%d_%H%M%S") + "_mixed_bus_" + ("restore_x_" if args.restore_x else "restore_address_" if args.restore_address_config else "restore_config_" if args.restore_config_at_4 else "ack_power_" if args.acknowledge_power else "readonly_") + uuid.uuid4().hex[:8]
    )
    root.mkdir(parents=True, exist_ok=False)
    (root / "source").mkdir()
    for source_name in ("y42_mixed_bus_recovery.py", "y42_pc_direct.py", "y42_position_probe_io.py"):
        shutil.copyfile(Path(__file__).with_name(source_name), root / "source" / source_name)
    results = []
    failure = None
    restored_verified = None
    with (root / "events.jsonl").open("x", encoding="utf-8") as events:
        def emit(event, **fields):
            events.write(json.dumps({"event": event, "wall_time": time.time(),
                                     "monotonic_ns": time.perf_counter_ns(), **fields},
                                    ensure_ascii=False) + "\n")
            events.flush()

        def query(connection, address, command, length, extra=()):
            tx = bytes((address, command, *extra, 0x6B))
            rx = direct.transact(connection, tx, first_byte_timeout=0.15,
                                 inter_byte_quiet=0.004,
                                 deadline=time.perf_counter() + 0.35)
            emit("query", baud=connection.baudrate, address=address,
                 tx=tx.hex(), rx=rx.hex())
            if len(rx) not in length or rx[:2] != tx[:2] or rx[-1:] != b"\x6b":
                raise RuntimeError(f"Invalid reply baud={connection.baudrate} addr={address} cmd={command:02x}: {rx.hex()}")
            return rx

        try:
            topology = ((921600, (1, 2, 3, 4)), (115200, (1, 4))) if args.discover else ((921600, (1, 2, 3)), (115200, (4,) if args.restore_config_at_4 else (1,)))
            if args.acknowledge_power:
                topology = ((921600, (1, 2, 3, 4)),)
            for baud, addresses in topology:
                with direct.open_serial("COM7", baud) as raw:
                    connection = _RecordedConnection(raw, emit)
                    for address in addresses:
                        try:
                            config = query(connection, address, 0x42, (33, 37), (0x6C,))
                        except RuntimeError:
                            if args.discover:
                                continue
                            raise
                        version = query(connection, address, 0x1F, (7,))
                        calibration = query(connection, address, 0x3B, (4,))
                        flags = query(connection, address, 0x3A, (4,))
                        speed = query(connection, address, 0x35, (6,))
                        current = query(connection, address, 0x27, (5,))
                        position = query(connection, address, 0x36, (8,))
                        is_x = len(config) == 37 and config[2:4] == b"\x25\x18"
                        is_emm = len(config) == 33 and config[2:4] == b"\x21\x15"
                        if not (is_x or is_emm):
                            raise RuntimeError("Unrecognized configuration format")
                        row = {
                            "baud": baud, "address": address,
                            "firmware": "X" if is_x else "Emm",
                            "config_hex": config.hex(), "version_hex": version.hex(),
                            "calibration_flags": calibration[2],
                            "encoder_ready": bool(calibration[2] & 1),
                            "motor_calibrated": bool(calibration[2] & 2),
                            "flags": flags[2],
                            "rpm": (-1 if speed[2] else 1) * int.from_bytes(speed[3:5], "big") / (10 if is_x else 1),
                            "phase_current_ma": int.from_bytes(current[2:4], "big"),
                            "position_deg": (-1 if position[2] else 1) * int.from_bytes(position[3:7], "big") * (0.1 if is_x else 360 / 65536),
                        }
                        results.append(row)
                        emit("axis_snapshot", **row)
            if args.acknowledge_power:
                if any(r["firmware"] != "X" or not r["motor_calibrated"]
                       or not r["encoder_ready"] or r["rpm"] != 0
                       or r["phase_current_ma"] > 100 or r["flags"] & 0x0D
                       for r in results):
                    raise RuntimeError("Cannot acknowledge power event without fresh four-axis readiness")
                with direct.open_serial("COM7", 921600) as raw:
                    connection = _RecordedConnection(raw, emit)
                    for row in results:
                        if not row["flags"] & 0x80:
                            continue
                        address = row["address"]
                        flags = query(connection, address, 0x3A, (4,))
                        speed = query(connection, address, 0x35, (6,))
                        if flags[2] & 0x0D or int.from_bytes(speed[3:5], "big"):
                            raise RuntimeError("Axis no longer stopped and disabled")
                        ack = query(connection, address, 0x50, (4,), (0,))
                        actual = query(connection, address, 0x3A, (4,))
                        if ack[2] != 2 or actual[2] != flags[2] & 0x7F:
                            raise RuntimeError("Power flag acknowledgement mismatch")
                        emit("power_event_acknowledged", address=address,
                             before=flags[2], after=actual[2],
                             cable_calibration_reused=False)
            if args.restore_x:
                expected = [(921600, 1, "X"), (921600, 2, "X"),
                            (921600, 3, "X"), (115200, 1, "Emm")]
                if [(r["baud"], r["address"], r["firmware"]) for r in results] != expected:
                    raise RuntimeError("Observed topology changed; no configuration write")
                if any(not r["motor_calibrated"] or not r["encoder_ready"]
                       or abs(r["rpm"]) > 0.1 or r["phase_current_ma"] > 100
                       or r["flags"] & 0x0C for r in results):
                    raise RuntimeError("Expected calibrated, stationary, low-current axes; no configuration write")
                for row in results:
                    with direct.open_serial("COM7", row["baud"]) as raw:
                        connection = _RecordedConnection(raw, emit)
                        address = row["address"]
                        stopped = query(connection, address, 0xFE, (4,), (0x98, 0))
                        if stopped[2] not in (2, 0xE2):
                            raise RuntimeError("STOP not acknowledged")
                        disabled = query(connection, address, 0xF3, (4,), (0xAB, 0, 0))
                        if disabled[2] != 2:
                            raise RuntimeError("DISABLE not acknowledged")
                        flags = query(connection, address, 0x3A, (4,))
                        speed = query(connection, address, 0x35, (6,))
                        if flags[2] & 0x0D or int.from_bytes(speed[3:5], "big") != 0:
                            raise RuntimeError("Fresh stopped/disabled verification failed")
                        emit("stopped_disabled", baud=row["baud"], address=address,
                             flags=flags[2], speed_hex=speed.hex())
                # Manufacturer p62: change firmware only while stopped. No
                # encoder calibration, motor-type, ID, baud, enable or motion
                # write is part of this narrowly scoped stage.
                with direct.open_serial("COM7", 115200) as raw:
                    connection = _RecordedConnection(raw, emit)
                    flags = query(connection, 1, 0x3A, (4,))
                    speed = query(connection, 1, 0x35, (6,))
                    if flags[2] & 0x0D or int.from_bytes(speed[3:5], "big") != 0:
                        raise RuntimeError("Slow-bus axis no longer stopped and disabled")
                    switched = query(connection, 1, 0xD5, (4,), (0x69, 1, 0))
                    if switched[2] != 2:
                        raise RuntimeError("Firmware selector write not acknowledged")
                    emit("firmware_x_selected", source_baud=115200, source_address=1,
                         ack_hex=switched.hex(), runtime_verification_pending=True)
            if args.restore_address_config or args.restore_config_at_4:
                expected = [(921600, 1, "X"), (921600, 2, "X"),
                            (921600, 3, "X"), (115200, 4 if args.restore_config_at_4 else 1, "X")]
                if [(r["baud"], r["address"], r["firmware"]) for r in results] != expected:
                    raise RuntimeError("Observed topology changed; no address write")
                if any(not r["motor_calibrated"] or not r["encoder_ready"]
                       or r["rpm"] != 0 or r["phase_current_ma"] > 100
                       or r["flags"] & 0x0D for r in results):
                    raise RuntimeError("All four axes must be calibrated, stopped and disabled")
                # Exact M4 readback from archived 13:49 probe, events line 1512.
                target = bytes.fromhex("044225180101010202011001000004b00bb80bb803e80807000100010008089807d000086b")
                current_config = bytes.fromhex(results[-1]["config_hex"])
                changes = [i for i in range(1, 37) if current_config[i] != target[i]]
                if any(i not in (4, 9, 22) for i in changes):
                    raise RuntimeError(f"Unexpected configuration differences {changes}; no write")
                emit("configuration_plan", source_hex=current_config.hex(), target_hex=target.hex(),
                     changed_indices=changes, id_from=1, id_to=4,
                     source_archive="20260905_134927_465_position_probe_worker_b5ad89a6/events.jsonl:1512")
                with direct.open_serial("COM7", 921600) as raw:
                    connection = _RecordedConnection(raw, emit)
                    absent = direct.transact(connection, bytes.fromhex("043a6b"),
                                             first_byte_timeout=0.15, deadline=time.perf_counter()+0.35)
                    if absent:
                        raise RuntimeError("Destination ID4 already responds at 921600; no write")
                with direct.open_serial("COM7", 115200) as raw:
                    connection = _RecordedConnection(raw, emit)
                    if not args.restore_config_at_4:
                        # Do not write an address already present on this rate.
                        absent = direct.transact(connection, bytes.fromhex("043a6b"),
                                                 first_byte_timeout=0.15, deadline=time.perf_counter()+0.35)
                        if absent:
                            raise RuntimeError("ID4 already responds at 115200; no address write")
                        flags = query(connection, 1, 0x3A, (4,))
                        speed = query(connection, 1, 0x35, (6,))
                        if flags[2] & 0x0D or int.from_bytes(speed[3:5], "big"):
                            raise RuntimeError("Axis no longer stopped/disabled")
                        tx = bytes.fromhex("01ae4b01046b")
                        ack = direct.transact(connection, tx, first_byte_timeout=0.15,
                                              deadline=time.perf_counter()+0.35)
                        emit("id_write", tx=tx.hex(), rx=ack.hex())
                        if ack not in (bytes.fromhex("01ae026b"), bytes.fromhex("04ae026b")):
                            raise RuntimeError("ID write outcome uncertain; no further write")
                        changed = query(connection, 4, 0x42, (37,), (0x6C,))
                        if changed != bytes((4,)) + current_config[1:]:
                            raise RuntimeError("New ID config mismatch; no further write")
                    old = direct.transact(connection, bytes.fromhex("013a6b"),
                                          first_byte_timeout=0.15, deadline=time.perf_counter()+0.35)
                    if old:
                        raise RuntimeError("Old ID still responds; no further write")
                    flags = query(connection, 4, 0x3A, (4,))
                    speed = query(connection, 4, 0x35, (6,))
                    if flags[2] & 0x0D or int.from_bytes(speed[3:5], "big"):
                        raise RuntimeError("New ID not stopped/disabled; no config write")
                    tx = bytes((4, 0x48, 0xD1, 1)) + target[4:]
                    ack = direct.transact(connection, tx, first_byte_timeout=0.15,
                                          deadline=time.perf_counter()+0.35)
                    emit("config_write", tx=tx.hex(), rx=ack.hex(), verification_pending=True)
                    # A baud change may move the reply itself to the new rate.
                    # Never retry the write on a missing ACK: verify read-only.
                with direct.open_serial("COM7", 921600) as raw:
                    connection = _RecordedConnection(raw, emit)
                    actual = query(connection, 4, 0x42, (37,), (0x6C,))
                    flags = query(connection, 4, 0x3A, (4,))
                    speed = query(connection, 4, 0x35, (6,))
                    cal = query(connection, 4, 0x3B, (4,))
                    if actual != target or flags[2] & 0x0D or int.from_bytes(speed[3:5], "big") or cal[2] & 3 != 3:
                        raise RuntimeError("Final ID4 configuration/calibration/disabled verification failed")
                    emit("restored_verified", baud=921600, address=4, config_hex=actual.hex(),
                         flags=flags[2], calibration_flags=cal[2], speed_hex=speed.hex())
                    restored_verified = {"baud": 921600, "address": 4,
                                         "config_hex": actual.hex(), "flags": flags[2],
                                         "calibration_flags": cal[2], "rpm": 0}
        except Exception as exc:
            failure = f"{type(exc).__name__}: {exc}"
            emit("failed", error=failure)
            raise
        finally:
            outcome = {"archive": str(root), "read_only": not (args.restore_x or args.restore_address_config or args.restore_config_at_4 or args.acknowledge_power),
                       "motion_commands_sent": False, "results": results,
                       "failure": failure, "restored_verified": restored_verified}
            (root / "result.json").write_text(json.dumps(outcome, ensure_ascii=False, indent=2), encoding="utf-8")
            events.flush()
            hashes = {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
                      for path in root.rglob("*") if path.is_file()}
            (root / "integrity.json").write_text(json.dumps(hashes, indent=2), encoding="utf-8")
            print(json.dumps(outcome, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
