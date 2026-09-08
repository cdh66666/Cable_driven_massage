"""Restore only archived volatile current/RPM caps; default operation is read-only.

No hardware is opened on import. The fixed source archive is data only; none of
its source files or event contents are executed. Stop the web backend first.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import errno
import hashlib
import json
from pathlib import Path
import socket
import time
import uuid
from contextlib import contextmanager

from y42_position_probe_io import _RecordedConnection


IDS = (1, 2, 3, 4)
RUN_ID = "20260905_224856_441_position_microprobe_worker_cdc1d3fb"
SOURCE_ARCHIVE = Path(__file__).parent / "manual_test_runs" / RUN_ID
PINNED_HASHES = {
    "events.jsonl": "c9736716ce8bde2f7a64ba10b88ecd3bb83774b4d12fa37c02c03491f88011a3",
    "manifest.json": "26dfc253d67781222c3e65e32a36b5759f1f7ae57dd472135b97dbdbcc0b1f36",
    "final_state.json": "6fb05c96836888938091d2de8112cd61d9dfd7e3963233c65cea4dcadc4e4e51",
}


def config_bytes(value, address):
    result = bytes.fromhex(value)
    if len(result) != 37 or result[:4] != bytes((address, 0x42, 0x25, 0x18)) or result[-1] != 0x6B:
        raise RuntimeError(f"M{address}归档完整X配置格式无效")
    return result


def outside_caps(config):
    return config[:16] + config[20:]


def config_write(address, config):
    return bytes((address, 0x48, 0xD1, 0)) + config[4:36] + b"\x6b"


def derive_residual_configs(events, originals):
    """Require an uninterrupted, recorded write/ACK/full-readback chain."""
    last = dict(originals)
    pending, original_seen = {}, set()
    for event in events:
        kind, address = event.get("event"), event.get("id")
        if kind == "position_probe_original_configuration":
            if address not in IDS or address in original_seen or config_bytes(event["raw_hex"], address) != originals[address]:
                raise RuntimeError("原配置事件不完整或与run_end不一致")
            original_seen.add(address)
        elif kind == "position_probe_configuration_change":
            if address not in IDS or address in pending or address not in original_seen:
                raise RuntimeError("配置写入顺序未知，禁止猜测残留值")
            expected = config_bytes(event["expected_hex"], address)
            if (event.get("store") != 0 or config_bytes(event["before_hex"], address) != last[address]
                    or config_bytes(event["original_hex"], address) != originals[address]
                    or outside_caps(expected) != outside_caps(originals[address])):
                raise RuntimeError("归档配置写入超出临时电流/转速字段或前态不符")
            pending[address] = {"phase": event["phase"], "expected": expected,
                                "written": False, "ack": False, "readback": False}
        elif kind == "position_probe_serial_write":
            packet = bytes.fromhex(event.get("attempted_tx_hex", ""))
            if len(packet) >= 2 and packet[1] == 0x48:
                address = packet[0]
                change = pending.get(address)
                if (change is None or change["written"] or packet != config_write(address, change["expected"])
                        or event.get("error") is not None or event.get("bytes_written") != len(packet)):
                    raise RuntimeError("存在未知/重复/不完整配置写入，禁止恢复")
                change["written"] = True
        elif kind == "position_probe_transaction":
            packet = bytes.fromhex(event.get("tx_hex", ""))
            if len(packet) >= 2 and packet[1] == 0x48:
                address, change = packet[0], pending.get(packet[0])
                if (change is None or not change["written"] or packet != config_write(address, change["expected"])
                        or event.get("error") is not None
                        or event.get("rx_hex") != bytes((address, 0x48, 2, 0x6B)).hex()):
                    raise RuntimeError("归档配置ACK未确认，禁止猜测应用结果")
                change["ack"] = True
            elif len(packet) >= 2 and packet[1] == 0x42 and packet[0] in pending:
                change = pending[packet[0]]
                if event.get("phase") == change["phase"] + "_readback":
                    if event.get("error") is not None or event.get("rx_hex") != change["expected"].hex():
                        raise RuntimeError("归档完整配置读回失败")
                    change["readback"] = True
        elif kind == "position_probe_configuration_readback":
            change = pending.get(address)
            if (change is None or not all(change[key] for key in ("written", "ack", "readback"))
                    or event.get("phase") != change["phase"] or event.get("exact_match") is not True
                    or config_bytes(event["actual_hex"], address) != change["expected"]
                    or config_bytes(event["expected_hex"], address) != change["expected"]):
                raise RuntimeError("归档配置写入缺少匹配的完整读回证据")
            last[address] = change["expected"]
            del pending[address]
    if pending or original_seen != set(IDS):
        raise RuntimeError("归档有未确认配置写入或缺失原配置，禁止恢复")
    return last


def load_archive(path=SOURCE_ARCHIVE):
    path = Path(path)
    blobs = {name: (path / name).read_bytes() for name in (*PINNED_HASHES, "integrity.json")}
    integrity = json.loads(blobs["integrity.json"])
    if integrity.get("schema") != "y42-manual-test-integrity-v1" or integrity.get("run_id") != RUN_ID:
        raise RuntimeError("归档完整性清单身份不符")
    declared = {"events.jsonl": integrity["events"]["sha256"],
                "manifest.json": integrity["manifest_sha256"], "final_state.json": integrity["final_state_sha256"]}
    for name, expected in PINNED_HASHES.items():
        if hashlib.sha256(blobs[name]).hexdigest() != expected or declared[name] != expected:
            raise RuntimeError(f"归档{name}哈希不符，未访问硬件")
    events = [json.loads(line) for line in blobs["events.jsonl"].splitlines() if line.strip()]
    if (integrity["events"].get("file") != "events.jsonl"
            or integrity["events"].get("bytes") != len(blobs["events.jsonl"])
            or integrity["events"].get("records") != len(events)
            or any(event.get("sequence") != index for index, event in enumerate(events, 1))):
        raise RuntimeError("归档事件计数/序列/字节数不符")
    manifest, final = json.loads(blobs["manifest.json"]), json.loads(blobs["final_state.json"])
    if (manifest.get("run_id") != RUN_ID or manifest.get("worker") != "position_microprobe_worker"
            or manifest.get("state_before", {}).get("port") != "COM7"
            or manifest.get("state_before", {}).get("baud") != 921600
            or final.get("port") != "COM7" or final.get("baud") != 921600
            or sum(event.get("event") == "run_end" for event in events) != 1
            or events[-1].get("event") != "run_end"):
        raise RuntimeError("归档运行/设备身份或终止事件不符")
    result = events[-1]["result"]
    if result.get("stage") != "position_microprobe" or result.get("success") is not False:
        raise RuntimeError("源归档不是指定失败的微步测试")
    raw_originals = result["original_configurations_hex"]
    if set(raw_originals) != {str(address) for address in IDS} or final.get("position_probe", {}).get("original_configurations_hex") != raw_originals:
        raise RuntimeError("run_end与final_state的四轴原配置不一致")
    originals = {address: config_bytes(raw_originals[str(address)], address) for address in IDS}
    if any(value[16:20] != b"\x0b\xb8\x0b\xb8" for value in originals.values()):
        raise RuntimeError("归档原限值不是3000mA/3000RPM")
    residuals = derive_residual_configs(events, originals)
    return originals, residuals, blobs


def refuse_backend(connect=socket.create_connection):
    for host in ("127.0.0.1", "::1"):
        try:
            with connect((host, 8787), timeout=0.25):
                raise RuntimeError("8787端口仍有监听，禁止打开COM7；请先停止网页后端")
        except ConnectionRefusedError:
            continue
        except OSError as exc:
            if exc.errno == errno.ECONNREFUSED or getattr(exc, "winerror", None) == 10061:
                continue
            raise RuntimeError("无法确认8787端口无人占用，未访问串口") from exc


@contextmanager
def exclusive_backend_guard():
    """Reserve the stopped backend's loopback ports for the recovery lifetime.

    A refused TCP connect is not guaranteed on this Windows host (even an
    unused port timed out). Exclusive bind both proves absence of a listener
    and prevents a new backend from starting while COM7 is owned here.
    """
    reservations = []
    try:
        for family, host in ((socket.AF_INET, "127.0.0.1"), (socket.AF_INET6, "::1")):
            reservation = socket.socket(family, socket.SOCK_STREAM)
            reservations.append(reservation)
            if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                reservation.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            if family == socket.AF_INET6:
                reservation.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 1)
            reservation.bind((host, 8787))
        yield
    finally:
        for reservation in reversed(reservations):
            reservation.close()


class RecordedRecoveryConnection(_RecordedConnection):
    """Reuse raw recording, preserving pending RX before each transaction reset."""
    def __init__(self, connection, emit, check):
        super().__init__(connection, emit)
        object.__setattr__(self, "budget_check", check)

    def write(self, packet):
        self.budget_check()
        result = super().write(packet)
        self.budget_check()
        return result

    def read(self, size=1):
        self.budget_check()
        result = super().read(size)
        self.budget_check()
        return result

    def reset_input_buffer(self):
        self.budget_check()
        drained = 0
        while int(getattr(self.connection, "in_waiting", 0) or 0):
            waiting = int(self.connection.in_waiting)
            if drained + waiting > 4096:
                raise RuntimeError("串口遗留接收数据超过4096字节，禁止丢弃")
            chunk = self.read(min(waiting, 512))
            if not chunk:
                raise RuntimeError("串口遗留数据无法完整归档")
            drained += len(chunk)
        self.emit("recovery_rx_reset", archived_bytes=drained)
        self.connection.reset_input_buffer()


def restore_envelope(raw, originals, residuals, *, transact, emit, apply=False,
                     clock=time.perf_counter, deadline=None):
    """Injected IO core; only queries and store=0 0x48 cap restoration allowed."""
    if type(apply) is not bool or set(originals) != set(IDS) or set(residuals) != set(IDS):
        raise ValueError("固定恢复参数无效")
    for address in IDS:
        config_bytes(originals[address].hex(), address)
        config_bytes(residuals[address].hex(), address)
        if originals[address][16:20] != b"\x0b\xb8\x0b\xb8" or outside_caps(originals[address]) != outside_caps(residuals[address]):
            raise ValueError("恢复只能改变归档电流/转速限值")
    deadline = float(clock()) + 20.0 if deadline is None else min(deadline, float(clock()) + 20.0)
    result = {"success": False, "read_only": not apply, "restored": [], "already_original": [],
              "unknown": [], "snapshots": [], "motion_commands_sent": False}

    def check():
        if float(clock()) >= deadline:
            raise TimeoutError("恢复流程超过20秒总预算，停止且不重试")

    connection = RecordedRecoveryConnection(raw, emit, check)

    def query(packet, length, phase):
        check()
        connection.phase = phase
        due = min(deadline, float(clock()) + 0.35)
        reply, error = b"", None
        try:
            reply = bytes(transact(connection, packet, first_byte_timeout=0.15,
                                   inter_byte_quiet=0.004, deadline=due))
            if float(clock()) >= due:
                raise TimeoutError("单次恢复事务超过0.35秒截止时间")
            check()
            if len(reply) != length or reply[:2] != packet[:2] or reply[-1:] != b"\x6b":
                raise RuntimeError(f"M{packet[0]}命令{packet[1]:02X}回包无效/缺失，禁止重试")
            return reply
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            emit("recovery_transaction", phase=phase, tx_hex=packet.hex(), rx_hex=reply.hex(),
                 error=error, deadline=due)

    def all_disabled(phase):
        rows = []
        for address in IDS:
            config = query(bytes((address, 0x42, 0x6C, 0x6B)), 37, phase)
            config_bytes(config.hex(), address)
            if config not in (originals[address], residuals[address]):
                raise RuntimeError(f"M{address}当前配置不是归档末次完整读回或原配置，禁止覆盖")
            calibrated = query(bytes((address, 0x3B, 0x6B)), 4, phase)
            if calibrated[2] != 3:
                raise RuntimeError(f"M{address}编码器/电机自身校准状态不是3")
            flags = query(bytes((address, 0x3A, 0x6B)), 4, phase)
            if flags[2] != 2:
                raise RuntimeError(f"M{address}必须严格处于flags=2的失能无故障状态")
            rpm = query(bytes((address, 0x35, 0x6B)), 6, phase)
            if rpm[2] not in (0, 1) or int.from_bytes(rpm[3:5], "big") != 0:
                raise RuntimeError(f"M{address}转速必须严格为0RPM")
            current = query(bytes((address, 0x27, 0x6B)), 5, phase)
            if int.from_bytes(current[2:4], "big") >= 100:
                raise RuntimeError(f"M{address}相电流必须小于100mA")
            rows.append({"id": address, "config_hex": config.hex(), "calibration_flags": 3,
                         "flags": 2, "rpm": 0, "phase_current_ma": int.from_bytes(current[2:4], "big")})
        result["snapshots"].append({"phase": phase, "axes": rows})
        emit("recovery_all_four_disabled", phase=phase, axes=rows)
        return {row["id"]: bytes.fromhex(row["config_hex"]) for row in rows}

    try:
        live = all_disabled("initial_readonly")
        emit("recovery_plan", apply=apply, changed_indices=[16, 17, 18, 19], store=0,
             originals={str(key): value.hex() for key, value in originals.items()},
             residuals={str(key): value.hex() for key, value in residuals.items()})
        if apply:
            for address in IDS:
                live = all_disabled(f"before_restore_M{address}")
                if live[address] == originals[address]:
                    result["already_original"].append(address)
                    continue
                expected = bytearray(live[address])
                expected[16:20] = originals[address][16:20]
                if bytes(expected) != originals[address]:
                    raise RuntimeError("恢复计划包含限值外的字段变化")
                result["unknown"].append(address)
                ack = query(config_write(address, bytes(expected)), 4, f"restore_M{address}")
                if ack != bytes((address, 0x48, 2, 0x6B)):
                    raise RuntimeError(f"M{address}恢复ACK未确认，应用状态未知，停止且不重试")
                actual = query(bytes((address, 0x42, 0x6C, 0x6B)), 37, f"restore_M{address}_readback")
                if actual != originals[address]:
                    raise RuntimeError(f"M{address}恢复后完整37字节配置不符，停止且不重试")
                result["unknown"].remove(address)
                result["restored"].append(address)
            live = all_disabled("final_disabled")
            if any(live[address] != originals[address] for address in IDS):
                raise RuntimeError("最终四轴配置未全部恢复原值")
        result["success"] = True
    except Exception as exc:
        result["failure"] = f"{type(exc).__name__}: {exc}"
        emit("recovery_failed", **result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--apply", action="store_true", help="仅恢复该故障归档的临时电流/转速上限；默认只读")
    args = parser.parse_args(argv)
    started = time.perf_counter()
    run = Path(__file__).parent / "manual_test_runs" / (
        datetime.now().strftime("%Y%m%d_%H%M%S") + "_restore_probe_envelope_"
        + ("apply_" if args.apply else "readonly_") + uuid.uuid4().hex[:8])
    run.mkdir(parents=True, exist_ok=False)
    result = {"success": False, "read_only": not args.apply, "archive": str(run),
              "source_archive": str(SOURCE_ARCHIVE), "motion_commands_sent": False}
    with (run / "events.jsonl").open("x", encoding="utf-8") as events:
        def emit(event, **fields):
            events.write(json.dumps({"event": event, "wall_time": time.time(),
                                     "monotonic_ns": time.perf_counter_ns(), **fields}, ensure_ascii=False) + "\n")
            events.flush()
        try:
            (run / "source").mkdir()
            for name in (Path(__file__).name, "y42_pc_direct.py", "y42_position_probe_io.py"):
                (run / "source" / name).write_bytes(Path(__file__).with_name(name).read_bytes())
            originals, residuals, blobs = load_archive()
            (run / "input_archive").mkdir()
            for name, data in blobs.items():
                (run / "input_archive" / name).write_bytes(data)
            emit("source_archive_verified", run_id=RUN_ID, sha256=PINNED_HASHES,
                 residual_caps={str(key): list(int.from_bytes(value[index:index + 2], "big")
                                for index in (16, 18)) for key, value in residuals.items()})
            with exclusive_backend_guard():
                if time.perf_counter() >= started + 20.0:
                    raise TimeoutError("归档核验已用尽20秒总预算，未打开串口")
                import y42_pc_direct as direct
                with direct.open_serial("COM7", 921600) as raw:
                    result.update(restore_envelope(raw, originals, residuals, transact=direct.transact,
                                                   emit=emit, apply=args.apply, deadline=started + 20.0))
        except Exception as exc:
            result.update(success=False, failure=f"{type(exc).__name__}: {exc}")
            emit("recovery_setup_failed", **result)
        finally:
            result["elapsed_seconds"] = time.perf_counter() - started
            emit("run_end", result=result)
            (run / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
            events.flush()
            hashes = {str(path.relative_to(run)): hashlib.sha256(path.read_bytes()).hexdigest()
                      for path in run.rglob("*") if path.is_file()}
            (run / "integrity.json").write_text(json.dumps(hashes, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
