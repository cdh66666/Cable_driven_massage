"""Bounded CB admission checks; no controller import or trajectory execution.

Default: disabled-only handshake.  The explicit zero-current branch enables
only after all global caps read back zero; neither branch raises working current.
"""
from __future__ import annotations

import math
import time


def disabled_position_seed(address, position, positive_direction):
    """14-byte absolute CB, 20 mA / 1 RPM; never the main 300 mA builder."""
    if address not in (1, 2, 3, 4) or positive_direction not in (0, 1):
        raise ValueError("invalid motor/direction")
    if not math.isfinite(position) or abs(position) > 429496729.5:
        raise ValueError("invalid absolute encoder position")
    return (bytes((address, 0xCB, int(position < 0) ^ positive_direction, 0, 10))
            + int(round(abs(position) * 10)).to_bytes(4, "big")
            + bytes((1, 0, 0, 20, 0x6B)))


class _RecordedConnection:
    """Record every attempted physical write, including partial/failed writes."""
    def __init__(self, connection, emit):
        object.__setattr__(self, "connection", connection)
        object.__setattr__(self, "emit", emit)
        object.__setattr__(self, "phase", "preflight")

    def __getattr__(self, name):
        return getattr(self.connection, name)

    def __setattr__(self, name, value):
        if name in ("connection", "emit", "phase"):
            object.__setattr__(self, name, value)
        else:
            setattr(self.connection, name, value)

    def write(self, packet):
        packet = bytes(packet)
        started = time.perf_counter_ns()
        written = None
        error = None
        try:
            existing = self.connection.write_timeout
            self.connection.write_timeout = (min(0.1, existing)
                if isinstance(existing, (int, float)) and math.isfinite(existing) and existing >= 0
                else 0.1)
            written = self.connection.write(packet)
            if written != len(packet):
                raise RuntimeError(f"串口仅写入 {written}/{len(packet)} 字节")
            return written
        except Exception as exc:
            error = str(exc)
            raise
        finally:
            self.emit("position_probe_serial_write", phase=self.phase,
                      attempted_tx_hex=packet.hex(), bytes_written=written,
                      expected_bytes=len(packet), error=error,
                      started_monotonic_ns=started,
                      completed_monotonic_ns=time.perf_counter_ns())

    def read(self, size=1):
        data = self.connection.read(size)
        if data:
            self.emit("position_probe_raw_rx", phase=self.phase,
                      raw_packet_hex=bytes(data).hex(),
                      received_monotonic_ns=time.perf_counter_ns())
        return data

    def flush(self):
        raise RuntimeError("探针禁止无界 flush")


def run_hardware_position_probe(api, *, zero_current_takeover=False):
    """Fixed first-stage handshake only; always success=False, no precision claim."""
    if type(zero_current_takeover) is not bool:
        raise ValueError("零电流接管只能由固定布尔入口选择")
    started = time.perf_counter()
    deadline = started + 15.0
    original = {}
    verified = {}
    touched = set()
    restored = []
    unknown = []
    not_attempted = []
    seeds = []
    owned = False
    final_disabled = False
    restore_complete = False
    reason = "尚未开始"
    cleanup_errors = []
    q0 = {}
    initial_positions = {}
    seed_attempted = 0
    motion_observed = None
    handshake_passed = False
    enable_attempted = []
    enabled_axes = set()
    current_bounds = {}
    rows = []

    def emit(event, **fields):
        api.manual_test_emit(event, **fields)

    def check(cleanup=False):
        if time.perf_counter() >= deadline:
            raise RuntimeError("接管验证/收尾超过本阶段时间预算")
        if not cleanup and api.STOP_EVENT.is_set():
            raise RuntimeError("用户停止接管验证")

    def transaction(connection, tx, phase, *, cleanup=False):
        check(cleanup)
        connection.phase = phase
        reply = b""
        error = None
        try:
            reply = bytes(api.direct.transact(connection, bytes(tx),
                first_byte_timeout=0.10, inter_byte_quiet=0.001,
                deadline=min(deadline, time.perf_counter() + 0.35)))
            return reply
        except Exception as exc:
            error = str(exc)
            raise
        finally:
            emit("position_probe_transaction", phase=phase,
                 tx_hex=bytes(tx).hex(), rx_hex=reply.hex(), error=error)

    def read_config(connection, address, phase, cleanup=False):
        reply = transaction(connection, bytes((address, 0x42, 0x6C, 0x6B)),
                            phase, cleanup=cleanup)
        if len(reply) != 37 or reply[:4] != bytes((address, 0x42, 0x25, 0x18)) or reply[-1] != 0x6B:
            raise RuntimeError(f"M{address}完整 X 配置无效")
        return reply

    def read_quiet(connection, phase, cleanup=False, *, expected_enabled=(),
                   q0_guard=None, phase_current_bound=None, rpm_bound=0.6):
        nonlocal final_disabled, motion_observed
        final_disabled = False
        readings = []
        for address in (1, 2, 3, 4):
            row = {"id": address}
            for command, length in ((0x36, 8), (0x35, 6), (0x27, 5), (0x39, 5), (0x3A, 4)):
                reply = transaction(connection, bytes((address, command, 0x6B)),
                                    phase, cleanup=cleanup)
                if len(reply) != length or reply[:2] != bytes((address, command)) or reply[-1] != 0x6B:
                    raise RuntimeError(f"M{address}新鲜反馈 {command:02X} 缺失/无效")
                if command in (0x36, 0x35):
                    if reply[2] not in (0, 1):
                        raise RuntimeError(f"M{address}编码器符号无效")
                    row["position" if command == 0x36 else "rpm"] = (
                        int.from_bytes(reply[3:-1], "big") / 10 * (-1 if reply[2] else 1))
                elif command == 0x27:
                    row["phase_current"] = int.from_bytes(reply[2:4], "big")
                elif command == 0x39:
                    if reply[2] not in (0, 1):
                        raise RuntimeError(f"M{address}温度符号无效")
                    row["temperature"] = reply[3]
                else:
                    row["flags"] = reply[2]
            row["sampled_at"] = time.perf_counter()
            readings.append(row)
            emit("position_probe_fresh_axis", phase=phase, **row)
            if q0_guard is not None and abs(row["position"] - q0_guard[address]) > 0.2 + 1e-9:
                motion_observed = True
                raise RuntimeError(f"M{address}零电流接管位置变化超过 0.2°")
            current_bound = 350 if phase_current_bound is None else phase_current_bound[address]
            if (row["flags"] & ~0x3F or row["flags"] & 0x08
                    or bool(row["flags"] & 1) != (address in expected_enabled)
                    or abs(row["rpm"]) > rpm_bound or row["phase_current"] > current_bound
                    or row["temperature"] >= 45):
                raise RuntimeError(f"M{address}未确认预期使能集合/停稳/反馈安全，禁止继续")
        final_disabled = not expected_enabled
        return readings

    def read_stopped_disabled(connection, phase, cleanup=False):
        return read_quiet(connection, phase, cleanup)

    def write_config(connection, address, expected, phase, cleanup=False):
        before = read_config(connection, address, phase + "_before", cleanup)
        if before != verified[address]:
            raise RuntimeError(f"M{address}配置已变化，拒绝覆盖")
        # Set uncertainty before the write: timeout never proves non-application.
        if cleanup and address in not_attempted:
            not_attempted.remove(address)
        touched.add(address)
        if address not in unknown:
            unknown.append(address)
        tx = bytes((address, 0x48, 0xD1, 0)) + expected[4:36] + b"\x6b"
        emit("position_probe_configuration_change", phase=phase, id=address,
             original_hex=original[address].hex(), before_hex=before.hex(),
             expected_hex=expected.hex(), store=0)
        reply = transaction(connection, tx, phase + "_write", cleanup=cleanup)
        if reply != bytes((address, 0x48, 2, 0x6B)):
            raise RuntimeError(f"M{address}配置 ACK 未确认，应用结果未知")
        after = read_config(connection, address, phase + "_readback", cleanup)
        emit("position_probe_configuration_readback", phase=phase, id=address,
             actual_hex=after.hex(), expected_hex=expected.hex(), exact_match=after == expected)
        if after != expected:
            raise RuntimeError(f"M{address}完整配置读回不匹配")
        verified[address] = after
        unknown.remove(address)

    with api.SERIAL_LOCK:
        try:
            with api.direct.open_serial(api.current_port(), api.current_baud()) as raw:
                connection = _RecordedConnection(raw, emit)
                try:
                    directions = api.calibration_data().get("encoder_positive_direction")
                    if directions != [0, 0, 1, 1]:
                        raise RuntimeError("保存的编码器方向不是已核验 [0,0,1,1]")
                    rows = read_stopped_disabled(connection, "initial_disabled")
                    initial_positions = {row["id"]: row["position"] for row in rows}
                    current_bounds = {row["id"]: min(80, max(row["phase_current"] + 30, 50)) for row in rows}
                    for address in (1, 2, 3, 4):
                        config = read_config(connection, address, "original_config")
                        original[address] = config
                        verified[address] = config
                        emit("position_probe_original_configuration", id=address, raw_hex=config.hex())
                        if (config[5] != 1 or config[25] not in (1, 3) or config[26] != 0
                                or int.from_bytes(config[16:18], "big") < 150
                                or int.from_bytes(config[18:20], "big") < 1):
                            raise RuntimeError(f"M{address}模式/ACK/角度量纲/原限值不符合接管前提")
                    read_stopped_disabled(connection, "before_envelope")
                    owned = True
                    for address in (1, 2, 3, 4):
                        read_stopped_disabled(connection, "envelope_disabled_gate")
                        expected = bytearray(original[address])
                        expected[16:20] = bytes((0, 0 if zero_current_takeover else 20, 0, 1))
                        write_config(connection, address, bytes(expected), "envelope")
                    rows = (read_quiet(connection, "before_zero_current_enable",
                                phase_current_bound=current_bounds, rpm_bound=0.3)
                            if zero_current_takeover else read_stopped_disabled(connection, "before_disabled_seed"))
                    if any(abs(row["position"] - initial_positions[row["id"]]) > 0.3 for row in rows):
                        raise RuntimeError("预置前球体/编码器已移动，拒绝使用过期 q0")
                    q0 = {row["id"]: row["position"] for row in rows}
                    for address in (1, 2, 3, 4):
                        tx = disabled_position_seed(address, q0[address], directions[address - 1])
                        if zero_current_takeover:
                            # All four exact-read global caps are still zero. Never increase here.
                            enable_attempted.append(address)
                            reply = transaction(connection, bytes((address, 0xF3, 0xAB, 1, 0, 0x6B)),
                                                "zero_current_enable")
                            if reply != bytes((address, 0xF3, 2, 0x6B)):
                                raise RuntimeError(f"M{address}零电流使能 ACK 未确认，停止接管")
                            enabled_axes.add(address)
                        seed_attempted += 1
                        reply = transaction(connection, tx,
                            "zero_current_cb_seed" if zero_current_takeover else "disabled_cb_seed")
                        accepted = reply == bytes((address, 0xCB, 2, 0x6B))
                        seeds.append({"id": address, "q0": q0[address], "tx_hex": tx.hex(),
                                      "rx_hex": reply.hex(), "accepted": accepted})
                        emit("position_probe_zero_current_seed" if zero_current_takeover
                             else "position_probe_disabled_seed", **seeds[-1])
                        if not accepted:
                            raise RuntimeError(f"M{address}{'零电流接管' if zero_current_takeover else '失能状态'} CB 预置被拒绝/未确认；停止，未绕过")
                        if zero_current_takeover:
                            rows = read_quiet(connection, "zero_current_axis_verified",
                                expected_enabled=enabled_axes, q0_guard=q0,
                                phase_current_bound=current_bounds, rpm_bound=0.3)
                    rows = (read_quiet(connection, "zero_current_all_verified",
                                expected_enabled=enabled_axes, q0_guard=q0,
                                phase_current_bound=current_bounds, rpm_bound=0.3)
                            if zero_current_takeover else read_stopped_disabled(connection, "after_disabled_seed"))
                    motion_observed = any(abs(row["position"] - q0[row["id"]]) > 0.2 + 1e-9 for row in rows)
                    if motion_observed:
                        raise RuntimeError("失能 CB 预置后编码器移动超过 0.2°，未通过无意外响应核验")
                    handshake_passed = True
                    reason = ("零电流当前位置接管通过；没有升流或运行位置探针，尚未验证运动效果"
                              if zero_current_takeover else "失能 CB 接管握手通过；本阶段不使能、不运行位置探针，尚未验证运动效果")
                except Exception as exc:
                    reason = str(exc)
                finally:
                    if owned:
                        deadline = time.perf_counter() + 15.0
                        # Cleanup always disables before restoring any original limit.
                        for action in ("stop", "disable"):
                            for address in (1, 2, 3, 4):
                                try:
                                    transaction(connection, api.command_frame(address, action, {}),
                                                "cleanup_" + action, cleanup=True)
                                except Exception as exc:
                                    cleanup_errors.append(f"M{address} {action}: {exc}")
                        not_attempted = sorted(touched)
                        try:
                            read_stopped_disabled(connection, "cleanup_disabled", True)
                            for address in sorted(touched):
                                read_stopped_disabled(connection, "restore_disabled_gate", True)
                                current = read_config(connection, address, "restore_current", True)
                                if current[:16] + current[20:] != original[address][:16] + original[address][20:]:
                                    raise RuntimeError(f"M{address}其他配置字段已变，禁止全量恢复覆盖")
                                verified[address] = current
                                write_config(connection, address, original[address], "restore_original", True)
                                restored.append(address)
                            read_stopped_disabled(connection, "final_disabled", True)
                            restore_complete = len(restored) == len(touched)
                        except Exception as exc:
                            cleanup_errors.append(str(exc))
                            # Revoke stale prior confirmation on any incomplete cleanup.
                            final_disabled = False
                            try:
                                read_stopped_disabled(connection, "cleanup_failure_disabled", True)
                            except Exception as final_exc:
                                cleanup_errors.append(str(final_exc))
        except Exception as exc:
            reason = str(exc)
            final_disabled = False

    result = dict(success=False, stage="zero_current_takeover" if zero_current_takeover else "disabled_cb_handshake_only",
                  reason=reason, motion_command_attempted=seed_attempted > 0,
                  motion_observed=motion_observed, enable_sent=bool(enable_attempted),
                  enable_attempted=enable_attempted, current_bounds=current_bounds,
                  handshake_passed=handshake_passed,
                  takeover_passed=zero_current_takeover and handshake_passed,
                  q0=q0, initial_positions=initial_positions,
                  disabled_seed_results=seeds, final_disabled_confirmed=final_disabled,
                  configuration_restored=restore_complete,
                  configuration_restore={"restored": restored, "unknown": sorted(unknown),
                                         "not_attempted": not_attempted},
                  original_configurations_hex={str(k): v.hex() for k, v in original.items()},
                  cleanup_errors=cleanup_errors, elapsed_seconds=time.perf_counter() - started,
                  requires_hardware_stop=owned and not final_disabled,
                  xyz_precision_measured=False, tension_verified=False)
    emit("position_probe_adapter_result", result=result)
    return result
