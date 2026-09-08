"""Fixed, bounded motor-angle microprobe; imports no controller or serial driver.

The caller injects the already-owned service API. Nothing runs on import. This
adapter deliberately has no amplitude, current, RPM, or duration overrides.
Its only motion is the existing pure probe: one axis at a time, 0.5 degrees,
20 -> 150 -> 20 mA, 1 RPM. Encoder response does not establish rope tension.

Admission is bounded to 30 seconds, the pure probe to 8 seconds, and cleanup to
15 seconds. Synchronous API callbacks must honor their supplied IO deadlines;
Python cannot interrupt an arbitrary blocked native callback.
"""
from __future__ import annotations

import math
import time
import hashlib
import sys
from pathlib import Path
from types import SimpleNamespace, ModuleType

from y42_position_probe_io import disabled_position_seed


def _load_probe_core():
    # Immutable core generation; existing workers retain their old functions.
    path = Path(__file__).with_name("y42_position_probe.py")
    source = path.read_bytes()
    digest = hashlib.sha256(source).hexdigest()
    name = "_y42_probe_core_" + digest
    if name not in sys.modules:
        module = ModuleType(name)
        module.__file__ = str(path)
        sys.modules[name] = module
        try:
            exec(compile(source, str(path), "exec"), module.__dict__)
        except BaseException:
            sys.modules.pop(name, None)
            raise
    return sys.modules[name], digest


_core, CORE_SHA256 = _load_probe_core()
ProbeConfig, ProbeSafetyError, run_position_probe = _core.ProbeConfig, _core.ProbeSafetyError, _core.run_position_probe


IDS = (1, 2, 3, 4)
COMMAND_HZ = 25.0
MAX_COMMAND_GAP_SECONDS = 3.0 / COMMAND_HZ
MAX_COMMAND_WRITE_SECONDS = 0.010
CADENCE_MIN_HZ = 0.90 * COMMAND_HZ
CADENCE_MAX_HZ = 1.10 * COMMAND_HZ
CADENCE_MAX_INTERVAL_SECONDS = 1.5 / COMMAND_HZ


def decode_quiet_43(reply, address, sampled_at):
    """One complete reply replaces five independent scalar queries."""
    if (len(reply) != 37 or reply[:4] != bytes((address, 0x43, 0x25, 0x0C))
            or reply[-1] != 0x6B or any(reply[i] not in (0, 1) for i in (14, 19, 22, 27, 32))):
        raise RuntimeError(f"M{address}完整0x43反馈缺失/无效")
    signed = lambda sign, start, end: int.from_bytes(reply[start:end], "big") / 10 * (-1 if reply[sign] else 1)
    return {"id": address, "position": signed(22, 23, 27), "rpm": signed(19, 20, 22),
            "phase_current": int.from_bytes(reply[8:10], "big"), "temperature": reply[33],
            "flags": reply[35], "sampled_at": sampled_at, "raw_packet_hex": reply.hex()}


def valid_encoder_directions(directions):
    """Accept the four saved calibrated bits, never an invented fallback."""
    return (isinstance(directions, (list, tuple)) and len(directions) == 4
            and all(type(bit) is int and bit in (0, 1) for bit in directions))


def microprobe_position_frame(targets, current_ma, directions):
    """Documented 61-byte 0xAA containing four absolute CB commands."""
    if (len(targets) != 4 or not valid_encoder_directions(directions)
            or isinstance(current_ma, bool) or not isinstance(current_ma, int)
            or not 20 <= current_ma <= 150):
        raise ValueError("fixed microprobe frame contract violated")
    payload = bytearray()
    for address, position, direction in zip(IDS, targets, directions):
        command = bytearray(disabled_position_seed(address, float(position), direction))
        command[11:13] = current_ma.to_bytes(2, "big")
        payload.extend(command)
    return b"\x00\xaa\x00\x3d" + bytes(payload) + b"\x6b"


class _RecordedConnection:
    """Archive every read/write, bound IO, and never call native flush()."""
    def __init__(self, connection, emit, clock, check):
        object.__setattr__(self, "connection", connection)
        object.__setattr__(self, "emit", emit)
        object.__setattr__(self, "clock", clock)
        object.__setattr__(self, "check", check)
        object.__setattr__(self, "phase", "preflight")
        object.__setattr__(self, "observe_raw", None)
        object.__setattr__(self, "write_budget", 0.10)

    def __getattr__(self, name):
        return getattr(self.connection, name)

    def __setattr__(self, name, value):
        if name in ("connection", "emit", "clock", "check", "phase", "observe_raw", "write_budget"):
            object.__setattr__(self, name, value)
        else:
            setattr(self.connection, name, value)

    def prepare_write_timeout(self):
        self.check()
        existing = self.connection.write_timeout
        bounded = min(self.write_budget, existing) if (
            isinstance(existing, (int, float)) and math.isfinite(existing) and existing >= 0
        ) else self.write_budget
        # PySerial's Windows setter reconfigures the whole port even when the
        # value is unchanged. Never do that on every 100 Hz command tick.
        if existing != bounded:
            self.connection.write_timeout = bounded
        self.check()

    def write(self, packet):
        self.check()
        packet = bytes(packet)
        started, written, error = self.clock(), None, None
        try:
            self.prepare_write_timeout()
            written = self.connection.write(packet)
            if written != len(packet):
                raise RuntimeError(f"串口仅写入 {written}/{len(packet)} 字节")
            self.check()
            if self.clock() - started > self.write_budget + 1e-9:
                raise TimeoutError("串口写入超过固定截止时间")
            return written
        except Exception as exc:
            error = str(exc)
            raise
        finally:
            self.emit("position_probe_serial_write", phase=self.phase,
                      attempted_tx_hex=packet.hex(), bytes_written=written,
                      expected_bytes=len(packet), error=error,
                      started_monotonic_ns=round(started * 1e9),
                      completed_monotonic_ns=round(self.clock() * 1e9))

    def read(self, size=1):
        self.check()
        started, data, error = self.clock(), b"", None
        try:
            data = bytes(self.connection.read(size))
            if data:
                self.emit("position_probe_raw_rx", phase=self.phase,
                          raw_packet_hex=data.hex(), received_monotonic_ns=round(self.clock() * 1e9))
                if self.observe_raw is not None:
                    self.observe_raw(data)
            self.check()
            return data
        except Exception as exc:
            error = str(exc)
            raise
        finally:
            if error:
                self.emit("position_probe_serial_read_error", phase=self.phase,
                          raw_packet_hex=data.hex(), error=error,
                          started_monotonic_ns=round(started * 1e9))

    def reset_input_buffer(self):
        # Helpers normally discard startup frames here. Archive and guard them
        # first; never discard an unbounded or unread pending receive buffer.
        self.check()
        drained = 0
        while int(getattr(self.connection, "in_waiting", 0) or 0):
            waiting = int(self.connection.in_waiting)
            if drained + waiting > 4096:
                raise RuntimeError("接收缓存超出归档清空上限")
            chunk = self.read(min(waiting, 512))
            if not chunk:
                raise RuntimeError("接收缓存无法读出，禁止静默丢弃")
            drained += len(chunk)
        self.emit("position_probe_rx_reset", phase=self.phase, archived_bytes=drained)
        self.connection.reset_input_buffer()

    def flush(self):
        # Existing stream helpers call send_fast(). Its subsequent ACK/frame
        # wait provides completion evidence; a native flush has no time bound.
        self.check()
        self.emit("position_probe_flush_omitted", phase=self.phase)


def audit_sync_tail(tx, reply):
    """Describe extra packets without accepting or deleting any received byte."""
    lengths = {0x36: 8, 0x35: 6, 0x27: 5, 0x39: 5, 0x3A: 4, 0x42: 37, 0x43: 37,
               0x45: 4, 0x48: 4, 0xF3: 4, 0xCB: 4, 0xFE: 4}
    expected = lengths.get(tx[1]) if len(tx) >= 2 else None
    if expected is None or len(reply) <= expected: return None
    body, tail = reply[:expected], reply[expected:]
    if body[:2] != tx[:2] or body[-1] != 0x6B: return None
    error_axis = tail[0] if len(tail) == 4 and tail[0] in IDS and tail[1:] == b'\x00\xee\x6b' else None
    return dict(expected_bytes=expected, body_hex=body.hex(), tail_hex=tail.hex(),
                error_axis=error_axis, accepted=False)


def run_hardware_position_microprobe(api, *, clock=time.perf_counter, sleep=None):
    return _run_hardware_probe(api, clock=clock, sleep=sleep, torque_mode=False)


def run_hardware_torque_admission(api, *, clock=time.perf_counter, sleep=None):
    """Fixed low-force C5 handoff prerequisite; does not perform calibration."""
    return _run_hardware_probe(api, clock=clock, sleep=sleep, torque_mode=True)


def run_hardware_corner_probe(api, phase, *, clock=time.perf_counter, sleep=None, position_mode=False):
    """Run an allowlisted bounded phase under the existing serial/cleanup owner."""
    if not callable(phase):
        raise TypeError("Corner phase callback is required")
    return _run_hardware_probe(api, clock=clock, sleep=sleep, torque_mode=True, corner_phase=phase,
                               corner_position_mode=position_mode)


def _run_hardware_probe(api, *, clock=time.perf_counter, sleep=None, torque_mode=False, corner_phase=None,
                        corner_position_mode=False):
    """Perform fixed admission, angle probe, STOP/DISABLE and exact restoration.

    ``api`` supplies SERIAL_LOCK, STOP_EVENT, direct.open_serial/transact,
    calibration_data, current_port/current_baud, manual_test_emit, the existing
    start/stop_timed_feedback_stream, AsyncTimedFeedbackReader, and
    _consume_timed_position_buffer helpers. No web module is imported here.
    The async reader must invoke on_record for every decoded original frame.
    """
    started = float(clock())
    deadline = started + 30.0
    cleaning = False
    original, verified, q0 = {}, {}, {}
    touched, unknown, enabled_axes = set(), set(), set()
    restored, not_attempted, cleanup_errors, seeds, command_stamps = [], [], [], [], []
    last_position_command_at = None
    initial_positions, current_bounds = {}, {}
    owned = final_disabled = restore_complete = takeover_passed = False
    probe_result, reader, connection = None, None, None
    stream_attempted = False
    raw_failure = None
    latest, stream_runs = {}, []
    motion_observed = False
    unexpected_motion_observed = False
    cleanup_stop_e2_axes = []
    cleanup_stop_e2_disabled_confirmed = False
    timer_enabled = False
    guard_position_bound, guard_rpm_bound, guard_currents = 0.2, 0.3, None
    reason = "尚未开始"
    last_transaction_completed = None
    preparation_quiet_s = 0.050 if corner_phase is not None else 0.0
    preparation_read_recoveries = 0
    poll_index = 0

    def emit(event, **fields):
        api.manual_test_emit(event, **fields)

    def check():
        if float(clock()) >= deadline:
            raise TimeoutError("位置微幅验证本阶段超过固定时间预算")
        if not cleaning and api.STOP_EVENT.is_set():
            raise RuntimeError("用户停止位置微幅验证")
        if not cleaning and raw_failure is not None:
            raise RuntimeError(raw_failure)

    emit("position_probe_core_version", source_sha256=CORE_SHA256,
         command_hz=COMMAND_HZ, feedback_mode="master_polled_two_axes_per_tick")

    def bounded_wait(seconds):
        if sleep is not None:
            sleep(seconds)
            check()
            return
        due = min(deadline, float(clock()) + seconds)
        while float(clock()) < due:
            check()
            if reader is not None:
                check_reader()
            remaining = due - float(clock())
            # A 10 ms Windows sleep was measured as 15.6 ms on this machine.
            # Preserve the real clock; bounded short waits check STOP/reader.
            if remaining > 0.016:
                time.sleep(min(0.005, remaining - 0.010))
        check()

    def transaction(tx, phase, cleanup=False, read_retry=False):
        nonlocal preparation_read_recoveries
        try:
            reply = transaction_once(tx, phase, cleanup)
            if reply or corner_phase is None:
                return reply
            raise TimeoutError('只读准备事务无回复')
        except Exception as exc:
            # Only preparation reads may be reissued, once, with a round cap.
            # A command/configuration write is NEVER retried after an uncertain
            # ACK. Movement polling and emergency cleanup remain fail-fast.
            readonly = (len(tx) in (3,4) and tx[0] in IDS
                        and tx[1] in (0x36,0x35,0x27,0x39,0x3A,0x42,0x43))
            reply = last_read_failure.get('reply',b'')
            audit = audit_sync_tail(bytes(tx),reply)
            recoverable = (not reply or (audit is not None and audit['error_axis'] in IDS
                                        and audit['error_axis'] != tx[0]))
            if (corner_phase is None or cleaning or cleanup or read_retry or not readonly
                    or not recoverable or preparation_read_recoveries >= 3
                    or api.STOP_EVENT.is_set()):
                raise
            preparation_read_recoveries += 1
            emit('corner_preparation_read_recovery',phase=phase,tx_hex=bytes(tx).hex(),
                 rejected_rx_hex=reply.hex(),reason=str(exc),attempt=1,
                 recoveries_this_round=preparation_read_recoveries,
                 policy='archive_then_reissue_read_only_require_new_complete_reply')
            bounded_wait(0.005)
            return transaction(tx,phase+'_read_retry',cleanup,True)

    last_read_failure = {}

    def transaction_once(tx, phase, cleanup=False):
        nonlocal last_transaction_completed
        check()
        # Controlled preparation-only turnaround experiment. Never delay the
        # asynchronous motion stream or emergency cleanup transactions.
        quiet_wait = 0.0
        if not cleaning and reader is None and last_transaction_completed is not None:
            quiet_wait = max(0.0, preparation_quiet_s - (float(clock()) - last_transaction_completed))
            if quiet_wait:
                bounded_wait(quiet_wait)
        connection.phase = phase
        query_deadline = min(deadline, float(clock()) + 0.35)
        reply, error = b"", None
        try:
            reply = bytes(api.direct.transact(connection, bytes(tx), first_byte_timeout=0.10,
                inter_byte_quiet=0.001, deadline=query_deadline))
            if float(clock()) >= query_deadline:
                raise TimeoutError("单次串口事务超过截止时间")
            check()
            audit = audit_sync_tail(bytes(tx), reply)
            if audit is not None:
                emit("position_probe_response_tail", phase=phase, tx_hex=bytes(tx).hex(), **audit)
                source = f"M{audit['error_axis']}协议错误" if audit['error_axis'] else "额外未归属数据"
                raise RuntimeError(f"M{tx[0]}回复后附加{source} {audit['tail_hex']}；整笔事务未放行")
            return reply
        except Exception as exc:
            error = str(exc)
            raise
        finally:
            last_read_failure['reply'] = reply
            last_transaction_completed = float(clock())
            emit("position_probe_transaction", phase=phase, tx_hex=bytes(tx).hex(),
                 rx_hex=reply.hex(), error=error, deadline=query_deadline,
                 preparation_quiet_s=preparation_quiet_s, quiet_wait_s=quiet_wait)

    def read_config(address, phase):
        reply = transaction(bytes((address, 0x42, 0x6C, 0x6B)), phase)
        if len(reply) != 37 or reply[:4] != bytes((address, 0x42, 0x25, 0x18)) or reply[-1] != 0x6B:
            raise RuntimeError(f"M{address}完整 X 配置无效")
        return reply

    def write_config(address, expected, phase):
        before = read_config(address, phase + "_before")
        if before != verified[address]:
            raise RuntimeError(f"M{address}配置已变化，拒绝覆盖")
        if cleaning and address in not_attempted:
            not_attempted.remove(address)
        touched.add(address)
        unknown.add(address)
        emit("position_probe_configuration_change", phase=phase, id=address,
             original_hex=original[address].hex(), before_hex=before.hex(),
             expected_hex=expected.hex(), store=0)
        if phase in ("global_20", "global_150"):
            # Change only current, with the documented temporary short command.
            # A full-0x48 compatibility A/B did not remove the extra error frame.
            if before[:16] + before[18:] != expected[:16] + expected[18:]:
                raise RuntimeError("运行中只允许修改闭环电流上限")
            tx = bytes((address, 0x45, 0x66, 0)) + expected[16:18] + b"\x6b"
            command = 0x45
        else:
            tx = bytes((address, 0x48, 0xD1, 0)) + expected[4:36] + b"\x6b"
            command = 0x48
        reply = transaction(tx, phase + "_write")
        if reply != bytes((address, command, 2, 0x6B)):
            raise RuntimeError(f"M{address}配置 ACK 未确认，应用结果未知")
        actual = read_config(address, phase + "_readback")
        emit("position_probe_configuration_readback", phase=phase, id=address,
             actual_hex=actual.hex(), expected_hex=expected.hex(), exact_match=actual == expected)
        if actual != expected:
            raise RuntimeError(f"M{address}完整配置读回不匹配")
        verified[address] = actual
        unknown.remove(address)

    def read_quiet(phase, expected_enabled=(), position_guard=None, currents=None, rpm_bound=0.6, settling_axis=None):
        nonlocal final_disabled, motion_observed, unexpected_motion_observed
        final_disabled = False
        rows = []
        for address in IDS:
            # Explicit scalar-query comparison, not fallback after a bad 0x43.
            # Keep every field and every guard; unsolicited bytes still abort.
            row = {"id": address}
            queries = ((0x43,37),) if corner_phase is not None else ((0x36, 8), (0x35, 6), (0x27, 5), (0x39, 5), (0x3A, 4))
            for command, length in queries:
                packet = bytes((address,0x43,0x7A,0x6B)) if command==0x43 else bytes((address, command, 0x6B))
                reply = transaction(packet, phase)
                # An empty read-only response during preparation/cleanup can
                # be retried without repeating any motor command. Never hide
                # malformed replies or widen the enclosing stage deadline.
                for retry in range(2):
                    if reply:
                        break
                    check()
                    emit("position_probe_empty_read_retry", phase=phase,
                         id=address, command=command, attempt=retry + 1)
                    bounded_wait(0.01)
                    reply = transaction(packet, phase)
                if len(reply) != length or reply[:2] != bytes((address, command)) or reply[-1] != 0x6B:
                    raise RuntimeError(f"M{address}新鲜0x{command:02X}反馈缺失/无效")
                if command == 0x43:
                    row.update(decode_quiet_43(reply,address,float(clock())))
                elif command in (0x36, 0x35):
                    if reply[2] not in (0, 1):
                        raise RuntimeError(f"M{address}编码器符号无效")
                    row["position" if command == 0x36 else "rpm"] = int.from_bytes(reply[3:-1], "big") / 10 * (-1 if reply[2] else 1)
                elif command == 0x27:
                    row["phase_current"] = int.from_bytes(reply[2:4], "big")
                elif command == 0x39:
                    if reply[2] not in (0, 1):
                        raise RuntimeError(f"M{address}温度符号无效")
                    row["temperature"] = reply[3]
                else:
                    row["flags"] = reply[2]
            row["sampled_at"] = float(clock())
            row["query_mode"] = "full_43_preparation" if corner_phase is not None else "scalar_comparison"
            position_bound = 3.0 if address == settling_axis else 0.2
            if position_guard is not None and abs(row["position"] - position_guard[address]) > position_bound + 1e-9:
                motion_observed = unexpected_motion_observed = True
                raise RuntimeError(f"M{address}接管位置变化超过 {position_bound:g}°")
            axis_rpm_bound = 0.3 if settling_axis is not None and address != settling_axis else rpm_bound
            if abs(row["rpm"]) > axis_rpm_bound:
                raise RuntimeError(f"M{address}原始转速超过接管上限")
            if row["phase_current"] > (350 if currents is None else currents[address]):
                raise RuntimeError(f"M{address}原始电流超过接管上限")
            if row["temperature"] >= 45:
                raise RuntimeError(f"M{address}温度无效或达到45度")
            flags = row["flags"]
            if flags & ~0x3F or flags & 0x08 or bool(flags & 1) != (address in expected_enabled):
                raise RuntimeError(f"M{address}未确认预期使能集合/保护状态")
            emit("position_probe_fresh_axis", phase=phase, **row)
            rows.append(row)
        final_disabled = not expected_enabled
        return rows

    def settle_enabled_axis(address):
        # Enabling a low-current position loop may produce a brief rotor
        # alignment movement. This is distinct from tracking a drawn path.
        # Only the newly enabled axis gets this bounded settling envelope.
        settle_started = float(clock())
        previous = None
        stable_since = None
        stable_min = stable_max = None
        while float(clock()) - settle_started < 2.5:
            rows = read_quiet("enable_settling", enabled_axes, q0, current_bounds,
                              3.0, settling_axis=address)
            if float(clock()) - settle_started >= 2.5:
                break
            current = next(row for row in rows if row["id"] == address)
            stable = (previous is not None and
                      abs(current["position"] - previous["position"]) <= 0.1 + 1e-9 and
                      abs(current["rpm"]) <= 0.3)
            if stable:
                low = min(current["position"], previous["position"])
                high = max(current["position"], previous["position"])
                stable_min = low if stable_min is None else min(stable_min, low)
                stable_max = high if stable_max is None else max(stable_max, high)
                if stable_max - stable_min > 0.1 + 1e-9:
                    stable_since = None
                    stable_min = stable_max = current["position"]
                elif stable_since is None:
                    stable_since = float(clock())
            else:
                stable_since = None
                stable_min = stable_max = None
            if stable_since is not None and float(clock()) - stable_since >= 0.4:
                old_q0 = q0[address]
                q0[address] = current["position"]
                # Bind the following motion to the observed settled pose, not
                # an outdated disabled reference that would create a jump.
                tx = disabled_position_seed(address, q0[address], directions[address - 1])
                reply = transaction(tx, "settled_cb_seed")
                if reply != bytes((address, 0xCB, 2, 0x6B)):
                    raise RuntimeError(f"M{address}落稳后参考点ACK未确认")
                emit("position_probe_enable_settled", id=address, before_q0=old_q0,
                     settled_q0=q0[address], offset_deg=q0[address]-old_q0,
                     elapsed_s=float(clock())-settle_started, max_offset_deg=3.0,
                     cb_current_ma=20, global_current_setting_ma=0)
                return read_quiet("zero_current_axis_verified", enabled_axes, q0, current_bounds, 0.3)
            previous = current
            bounded_wait(0.05)
        raise RuntimeError(f"M{address}使能后2.5秒内未落稳")

    def guard_record(record):
        nonlocal raw_failure, motion_observed, unexpected_motion_observed
        if cleaning:
            return
        try:
            address = record.address
            if type(address) is not int or address not in IDS:
                raise RuntimeError("原始0x43反馈ID无效")
            row = {"id": address}
            for key in ("position", "rpm", "phase_current", "temperature", "sampled_at"):
                row[key] = float(getattr(record, key))
                if not math.isfinite(row[key]):
                    raise RuntimeError(f"M{address}原始0x43字段 {key} 非有限")
            row["flags"] = record.flags
            row["raw_packet_hex"] = record.raw_packet_hex
            emit("position_microprobe_raw_record", **row)
            if type(row["flags"]) is not int or row["flags"] & ~0x3F or row["flags"] & 0x08 or not row["flags"] & 1:
                raise RuntimeError(f"M{address}原始0x43失能/保护/状态无效")
            if abs(row["position"] - q0[address]) > 0.1 + 1e-9:
                motion_observed = True
            if abs(row["position"] - q0[address]) > guard_position_bound + 1e-9:
                motion_observed = True
                unexpected_motion_observed = True
                raise RuntimeError(f"M{address}原始0x43位置越界")
            if abs(row["rpm"]) > guard_rpm_bound:
                raise RuntimeError(f"M{address}原始0x43转速越界")
            if abs(row["phase_current"]) > (350 if guard_currents is None else guard_currents[address]):
                raise RuntimeError(f"M{address}原始0x43电流越界")
            if row["temperature"] >= 45:
                raise RuntimeError(f"M{address}原始0x43温度达到45度")
            if not -0.02 <= float(clock()) - row["sampled_at"] <= 0.20:
                raise RuntimeError(f"M{address}原始0x43时间陈旧/无效")
            return row
        except Exception as exc:
            if raw_failure is None:
                raw_failure = str(exc)
            raise

    def check_reader():
        check()
        if reader is None:
            return
        reader.check()
        # The async parser can add a counter while the writer checks it.
        if any(value for key, value in reader.parser_stats.copy().items() if key.startswith("invalid")):
            raise RuntimeError("原始0x43解析拒绝帧，停止且不丢弃异常继续")

    def drain_feedback():
        check_reader()
        if reader is None:
            return
        for record in reader.drain():
            # Guard every record even if a later sample of the same axis exists.
            row = guard_record(record)
            if row["sampled_at"] <= latest.get(row["id"], {}).get("sampled_at", -math.inf):
                raise RuntimeError("原始0x43时间戳不递增")
            latest[row["id"]] = row
        check_reader()

    def snapshot():
        drain_feedback()
        if set(latest) != set(IDS):
            raise RuntimeError("四轴新鲜0x43反馈不完整")
        rows = [dict(latest[address]) for address in IDS]
        stamps = [row["sampled_at"] for row in rows]
        if (float(clock()) - min(stamps) > 0.20 or max(stamps) - min(stamps) > 0.10
                or max(stamps) > float(clock()) + 0.02):
            raise RuntimeError("四轴0x43反馈陈旧/跨度超过0.1秒")
        return rows

    def raw_stream_observer():
        buffer, stats = bytearray(), {}
        def observe_raw(chunk):
            buffer.extend(chunk)
            records, _ = api._consume_timed_position_buffer(buffer, parser_stats=stats)
            for record in records:
                guard_record(record)
            if not cleaning and any(value for key, value in stats.items() if key.startswith("invalid")):
                raise RuntimeError("启动流出现原始0x43解析拒绝帧")
        return observe_raw

    def polled_exchange(packet, expected_length, phase):
        """One owner: no next TX until the previous response is complete."""
        check()
        connection.phase = phase
        started = float(clock())
        due = min(deadline, started + 0.012)
        reply = bytearray()
        error = None
        try:
            pending = int(getattr(connection, "in_waiting", 0) or 0)
            if pending:
                unsolicited = connection.read(min(pending, 4096))
                emit("position_probe_unsolicited_rx", phase=phase, raw_packet_hex=unsolicited.hex())
                raise RuntimeError("主机轮询前出现未请求回包，停止且不清空继续")
            connection.write(packet)
            last_byte = None
            while float(clock()) < due:
                check()
                waiting = int(getattr(connection, "in_waiting", 0) or 0)
                chunk = connection.read(min(waiting, 128) if waiting else 1)
                if chunk:
                    reply.extend(chunk)
                    last_byte = float(clock())
                    if len(reply) > expected_length:
                        raise RuntimeError("主机轮询收到额外/错误帧，停止且不删尾")
                elif len(reply) == expected_length and last_byte is not None and float(clock())-last_byte >= 0.0005:
                    return bytes(reply)
            raise TimeoutError("主机轮询事务超过12毫秒时限")
        except Exception as exc:
            error = str(exc)
            raise
        finally:
            emit("position_probe_polled_transaction", phase=phase, tx_hex=packet.hex(),
                 rx_hex=bytes(reply).hex(), error=error, started_at=started,
                 completed_at=float(clock()), expected_bytes=expected_length)

    def poll_one_axis():
        nonlocal poll_index
        address = IDS[poll_index % len(IDS)]
        poll_index += 1
        reply = polled_exchange(bytes((address, 0x43, 0x7A, 0x6B)), 37, "master_polled_feedback")
        decoded = decode_quiet_43(reply, address, float(clock()))
        record = SimpleNamespace(address=address, **{k:v for k,v in decoded.items() if k != "id"})
        row = guard_record(record)
        if row["sampled_at"] <= latest.get(address, {}).get("sampled_at", -math.inf):
            raise RuntimeError("轮询反馈时间戳不递增")
        latest[address] = row
        stats = stream_runs[-1]["feedback_per_axis"][str(address)]
        stats["records"] += 1
        stats["first_at"] = row["sampled_at"] if stats["first_at"] is None else stats["first_at"]
        stats["last_at"] = row["sampled_at"]
        span = stats["last_at"] - stats["first_at"]
        stats["measured_hz"] = (stats["records"]-1)/span if span > 0 else None

    def start_stream(phase):
        nonlocal latest
        check()
        connection.phase = phase
        if not api.stop_timed_feedback_stream(connection):
            raise RuntimeError("进入主机轮询前四轴自主反馈未确认关闭")
        latest = {}
        stream_runs.append({"phase": phase, "mode": "master_polled",
                            "results": [], "effective_interval_ms": 2000 / COMMAND_HZ,
                            "feedback_per_axis": {str(address): {"records": 0, "first_at": None,
                                "last_at": None, "measured_hz": None} for address in IDS}})
        emit("position_microprobe_stream_started", **stream_runs[-1])
        warm_deadline = min(deadline, float(clock()) + 0.50)
        first, signature, count = None, None, 0
        while float(clock()) < warm_deadline:
            poll_one_axis()
            if set(latest) == set(IDS):
                rows = snapshot()
                stamps = tuple(row["sampled_at"] for row in rows)
                if signature is None or all(new > old for new, old in zip(stamps, signature)):
                    signature, count = stamps, count + 1
                    first = min(stamps) if first is None else first
                    if count >= 3 and min(stamps) - first >= 0.15 - 1e-9:
                        emit("position_microprobe_stream_warm", phase=phase, fresh_batches=count,
                             observed_span_seconds=min(stamps) - first)
                        return
            bounded_wait(0.005)
        raise RuntimeError("0x43流未取得0.15秒三批四轴新鲜静止反馈")

    def stop_reader_and_stream(phase):
        nonlocal reader, stream_attempted
        connection.phase = phase
        connection.observe_raw = None
        if reader is not None:
            active_reader, reader = reader, None
            try:
                remaining_records = active_reader.stop()
                for record in remaining_records:
                    guard_record(record)
            except Exception as exc:
                if not cleaning:
                    raise
                cleanup_errors.append(f"feedback_reader_stop: {exc}")
            finally:
                if getattr(getattr(active_reader, "thread", None), "is_alive", lambda: False)():
                    raise RuntimeError("反馈线程仍在运行，禁止并发查询/恢复配置")
        if stream_attempted:
            # With the reader stopped, guard the original frames consumed by
            # stream-control helpers as well as the frames used by the probe.
            connection.observe_raw = raw_stream_observer()
            if not api.stop_timed_feedback_stream(connection):
                raise RuntimeError("四轴定时反馈流未全部停止")
            stream_attempted = False
            connection.observe_raw = None
        check()

    def fresh_stop_disable(phase):
        nonlocal final_disabled
        final_disabled = False
        errors = []
        # No stream negotiation or logging preamble delays these fresh writes.
        for command in (0xFE, 0xF3):
            for address in IDS:
                packet = (bytes((address, 0xFE, 0x98, 0, 0x6B)) if command == 0xFE
                          else bytes((address, 0xF3, 0xAB, 0, 0, 0x6B)))
                try:
                    connection.phase = phase + ("_stop" if command == 0xFE else "_disable")
                    connection.write(packet)
                except Exception as exc:
                    errors.append(f"M{address} {command:02X}: {exc}")
                # The first protective write is immediate; leave a reply slot
                # between subsequent writes on the shared half-duplex bus.
                if sleep is None:
                    time.sleep(0.005)
                else:
                    sleep(0.005)
        return errors

    def emergency_stop(error):
        nonlocal cleaning, deadline
        cleaning = True
        deadline = float(clock()) + 15.0
        connection.write_budget = 0.10
        errors = fresh_stop_disable("emergency")
        cleanup_errors.extend(errors)
        return {"attempted": True, "errors": errors, "reason": error}

    def send_positions(targets, current_ma, max_rpm):
        nonlocal last_position_command_at
        check_reader()
        stamp = float(clock())
        if last_position_command_at is not None and stamp - last_position_command_at > MAX_COMMAND_GAP_SECONDS + 1e-9:
            raise TimeoutError("位置指令实际间隔超过本轮三个周期，停止且不补发旧点")
        if max_rpm != 1.0 or any(abs(float(value) - q0[address]) > 0.5 + 1e-9 for address, value in zip(IDS, targets)):
            raise RuntimeError("位置微幅指令超出固定0.5度/1RPM范围")
        connection.phase = "microprobe_command_stream"
        connection.write_budget = MAX_COMMAND_WRITE_SECONDS
        packet = microprobe_position_frame(targets, current_ma, directions)
        axis_stamps = []
        # Core generation includes continuous phase-boundary command timing.
        # This device acknowledges addressed CB but did not acknowledge AA/CB
        # in the recorded trial. Establish the functional baseline with four
        # explicit transactions; preserve the actual inter-axis skew in logs.
        for index, address in enumerate(IDS):
            axis_started = float(clock())
            command = packet[4 + index * 14:4 + (index + 1) * 14]
            reply = polled_exchange(command, 4, "master_polled_command")
            if reply != bytes((address, 0xCB, 2, 0x6B)):
                raise RuntimeError(f"M{address}位置指令确认无效")
            axis_stamps.append(dict(id=address, started_at=axis_started, completed_at=float(clock())))
            if float(clock()) - stamp >= 1.0 / COMMAND_HZ:
                raise TimeoutError("本组四轴指令耗尽一个控制周期，停止且不追发")
        completed = float(clock())
        last_position_command_at = stamp
        command_stamps.append({"started_at": stamp, "completed_at": completed})
        emit("position_microprobe_command_cadence", sequence=len(command_stamps),
             started_at=stamp, completed_at=completed,
             interval_seconds=None if len(command_stamps) < 2 else stamp - command_stamps[-2]["started_at"],
             write_seconds=completed - stamp, target_hz=COMMAND_HZ,
             transport="addressed_cb", axes=axis_stamps,
             axis_start_skew_seconds=axis_stamps[-1]["started_at"]-axis_stamps[0]["started_at"])
        check_reader()
        poll_one_axis()
        poll_one_axis()
        if float(clock()) - stamp >= 1.0 / COMMAND_HZ:
            raise TimeoutError("本组指令与反馈耗尽一个控制周期，停止且不追发")

    def torque_admission():
        """Observe a brief, hardware-limited torque command then recover q0."""
        nonlocal last_position_command_at
        begin = float(clock())
        pull = tuple(api.pull_directions())
        if len(pull) != 4 or any(value not in (0, 1) for value in pull):
            raise RuntimeError("收绳方向无效，未发送力矩命令")
        axis_results = []

        def observe(phase):
            poll_one_axis()
            rows = [latest[address] for address in IDS]
            if any(abs(row['position'] - q0[row['id']]) > 3.0 for row in rows):
                raise RuntimeError("低力矩接管位移超过3度，停止")
            if any(abs(row['rpm']) > 1.2 for row in rows):
                raise RuntimeError("低力矩接管速度超过1.2RPM，停止")
            emit("torque_admission_feedback", phase=phase, rows=rows)
            return rows

        # Four addressed commands avoid relying on unobserved AA ACKs.
        for address, direction in zip(IDS, pull):
            pulse_started = float(clock())
            packet = bytes((address, 0xC5, direction, 0, 200, 0, 20, 0, 0, 10, 0x6B))
            reply = polled_exchange(packet, 4, "torque_admission_command")
            if reply != bytes((address, 0xC5, 2, 0x6B)):
                raise RuntimeError(f"M{address}低力矩C5接管未确认")
            observe("entry")
            while float(clock()) - pulse_started < 0.12:
                observe("low_torque_hold")
            emit("torque_admission_low_force_observed", id=address,
                 elapsed_seconds=float(clock())-pulse_started, calibration_published=False)
            # The low-force gate never promotes the hardware cap above20mA.
            # A weak recovery must fail closed, not request additional torque.
            stable_start = None
            recovery_start = float(clock())
            # C5 observation is a distinct mode; keep its intentional pause
            # in the archive while starting a new position-recovery clock.
            last_position_command_at = None
            recovered = False
            while float(clock()) - recovery_start < 1.5:
                tick = float(clock())
                send_positions([q0[axis] for axis in IDS], 20, 1.0)
                rows = observe("q0_recovery")
                stable = all(abs(row['position']-q0[row['id']]) <= .25 and abs(row['rpm']) <= .3 for row in rows)
                stable_start = (stable_start if stable_start is not None else float(clock())) if stable else None
                if stable_start is not None and float(clock()) - stable_start >= .4:
                    recovered = True
                    break
                bounded_wait(max(0.0, tick + 1.0/COMMAND_HZ-float(clock())))
            if not recovered:
                raise RuntimeError(f"M{address}低力矩接管后未恢复q0连续停稳窗")
            axis_results.append(dict(id=address, tx_hex=packet.hex(), rx_hex=reply.hex(), recovered=True))
        return dict(success=True, final_hold_verified=True, axis_results=axis_results,
                    final_rows=rows, calibration_published=False,
                    elapsed_seconds=float(clock())-begin)

    with api.SERIAL_LOCK:
        try:
            with api.direct.open_serial(api.current_port(), api.current_baud()) as raw:
                # direct.transact restores its entry timeout after every query.
                # Fix the native bound once so its deadline clamp and this
                # wrapper do not repeatedly reconfigure the Windows COM port.
                entry_write_timeout = raw.write_timeout
                native_write_timeout = min(entry_write_timeout, MAX_COMMAND_WRITE_SECONDS) if (
                    isinstance(entry_write_timeout, (int, float)) and math.isfinite(entry_write_timeout)
                    and entry_write_timeout >= 0) else MAX_COMMAND_WRITE_SECONDS
                if entry_write_timeout != native_write_timeout:
                    raw.write_timeout = native_write_timeout
                connection = _RecordedConnection(raw, emit, clock, check)
                try:
                    directions = api.calibration_data().get("encoder_positive_direction")
                    if not valid_encoder_directions(directions):
                        raise RuntimeError("保存的四台编码器方向缺失或无效；请先完成方向校验")
                    emit("position_probe_saved_directions", directions=list(directions),
                         source="saved_calibration", fallback_used=False)
                    if getattr(raw, "_y42_direction_mode", "auto") != "auto":
                        raise RuntimeError("微幅轮询路径要求已验证的自动RS485方向")
                    rows = read_quiet("initial_disabled")
                    initial_positions = {row["id"]: row["position"] for row in rows}
                    current_bounds = {row["id"]: min(80, max(row["phase_current"] + 30, 50)) for row in rows}
                    guard_currents = current_bounds
                    for address in IDS:
                        cfg = read_config(address, "original_config")
                        original[address] = verified[address] = cfg
                        emit("position_probe_original_configuration", id=address, raw_hex=cfg.hex())
                        if (cfg[5] != 1 or cfg[25] not in (1, 3) or cfg[26] != 0
                                or int.from_bytes(cfg[16:18], "big") != 3000
                                or int.from_bytes(cfg[18:20], "big") != 3000):
                            raise RuntimeError(f"M{address}模式/ACK/角度量纲/原3000mA与3000RPM限值不符")
                    read_quiet("before_envelope")
                    owned = True
                    for address in IDS:
                        read_quiet("zero_envelope_disabled_gate")
                        cfg = bytearray(original[address]); cfg[16:20] = b"\x00\x00\x00\x01"
                        write_config(address, bytes(cfg), "zero_envelope")
                    # A fresh read of ALL four full configurations precedes ANY enable.
                    for address in IDS:
                        if read_config(address, "all_zero_exact_readback") != verified[address] or verified[address][16:20] != b"\x00\x00\x00\x01":
                            raise RuntimeError("四轴零电流/1RPM完整配置未全部确认")
                    rows = read_quiet("before_zero_current_enable", currents=current_bounds, rpm_bound=0.3)
                    if any(abs(row["position"] - initial_positions[row["id"]]) > 0.3 for row in rows):
                        raise RuntimeError("预置前编码器已移动，拒绝过期q0")
                    q0 = {row["id"]: row["position"] for row in rows}
                    for address in IDS:
                        q0[address] = next(row["position"] for row in rows if row["id"] == address)
                        reply = transaction(bytes((address, 0xF3, 0xAB, 1, 0, 0x6B)), "zero_current_enable")
                        if reply != bytes((address, 0xF3, 2, 0x6B)):
                            raise RuntimeError(f"M{address}零电流使能ACK未确认")
                        enabled_axes.add(address)
                        tx = disabled_position_seed(address, q0[address], directions[address - 1])
                        reply = transaction(tx, "zero_current_cb_seed")
                        accepted = reply == bytes((address, 0xCB, 2, 0x6B))
                        seeds.append({"id": address, "q0": q0[address], "tx_hex": tx.hex(), "rx_hex": reply.hex(), "accepted": accepted})
                        emit("position_probe_zero_current_seed", **seeds[-1])
                        if not accepted:
                            raise RuntimeError(f"M{address}零电流CB预置被拒绝/未确认")
                        if corner_phase is not None and address == 1:
                            # Isolate unsolicited errors from errors triggered
                            # by the following query. All global current caps
                            # are still zero; do not transmit during this window.
                            idle_started = float(clock())
                            connection.phase = "zero_current_seed_idle_observation"
                            while float(clock()) - idle_started < 0.35:
                                check()
                                pending = int(connection.in_waiting)
                                if pending:
                                    unsolicited = connection.read(min(pending, 4096))
                                    emit("corner_preparation_idle_observation",
                                         elapsed_seconds=float(clock())-idle_started,
                                         tx_count=0, raw_packet_hex=unsolicited.hex(),
                                         success=False)
                                    raise RuntimeError("零电流使能后无发送窗口出现非请求回包 " + unsolicited.hex())
                                bounded_wait(0.005)
                            emit("corner_preparation_idle_observation",
                                 elapsed_seconds=float(clock())-idle_started,
                                 tx_count=0, raw_packet_hex="", success=True)
                        rows = settle_enabled_axis(address)
                    read_quiet("zero_current_all_verified", IDS, q0, current_bounds, 0.3)
                    takeover_passed = True
                    # Prepare caps using fresh synchronous checks while every
                    # accepted q0 CB remains capped at20mA. Start the async
                    # stream only once after preparation, and require its full
                    # stable window BEFORE the actual20->150mA motion ramp.
                    # All four CB targets/caps are already q0/20 mA while global
                    # limits are zero. No old target can become live on promotion.
                    promotion_order = (2, 3, 4, 1)
                    emit("position_probe_promotion_order", order=list(promotion_order),
                         experiment="m1_last_same_caps_and_guards")
                    for cap in ((20, 60) if corner_phase is not None else ((20,) if torque_mode else (20, 150))):
                        for address in promotion_order:
                            read_quiet(f"before_global_{cap}", IDS, q0, current_bounds, 0.3)
                            cfg = bytearray(verified[address]); cfg[16:20] = cap.to_bytes(2, "big") + b"\x00\x01"
                            write_config(address, bytes(cfg), f"global_{cap}")
                            read_quiet(f"after_global_{cap}", IDS, q0, current_bounds, 0.3)
                        for address in IDS:
                            if read_config(address, f"all_global_{cap}_exact") != verified[address]:
                                raise RuntimeError("升流后四轴完整配置复核失败")
                    if callable(getattr(api, "set_high_resolution_timer", None)):
                        timer_enabled = bool(api.set_high_resolution_timer(True))
                    start_stream("q0_20ma_master_polled_warmup")
                    guard_position_bound, guard_rpm_bound, guard_currents = (
                        (3.0, 1.2, current_bounds) if torque_mode else (4.0, 3.0, None))
                    if corner_phase is not None:
                        guard_position_bound, guard_rpm_bound, guard_currents = (
                            3.0 if corner_position_mode else 30.0), 1.2, {address:90 for address in IDS}
                    connection.write_budget = MAX_COMMAND_WRITE_SECONDS
                    connection.prepare_write_timeout()
                    deadline = float(clock()) + (10.0 if corner_phase is not None else 8.0)
                    if corner_phase is not None:
                        corner_pull=tuple(api.pull_directions())
                        if len(corner_pull)!=4 or any(value not in (0,1) for value in corner_pull):
                            raise ValueError('Corner pull directions are invalid')
                        def corner_feedback():
                            for _ in IDS:
                                poll_one_axis()
                            return snapshot()
                        def corner_send(packet):
                            packet=bytes(packet)
                            if (len(packet)!=11 or packet[0] not in IDS or packet[1]!=0xC5
                                    or packet[2]!=corner_pull[packet[0]-1]
                                    or packet[3:5]!=b'\x00\xc8'
                                    or int.from_bytes(packet[5:7], 'big') not in (20,40,60)
                                    or packet[7:]!=b'\x00\x00\x0a\x6b'):
                                raise ValueError('Corner phase attempted a command outside its fixed limits')
                            if corner_position_mode and (packet[0]==1 or int.from_bytes(packet[5:7],'big')!=20):
                                raise ValueError('Position corner mode allows only passive-axis 20mA C5')
                            reply=polled_exchange(packet,4,'corner_stop_probe_command')
                            if reply!=bytes((packet[0],0xC5,2,0x6B)):
                                raise RuntimeError('Corner C5 acknowledgement missing')
                            return {'accepted':True,'tx_hex':packet.hex(),'rx_hex':reply.hex()}
                        corner_cb_previous = [None, None]
                        corner_cb_start = float(latest[1]['position'])
                        def corner_cb(target_position_deg, current_ma):
                            check()
                            if not corner_position_mode:
                                raise ValueError('CB is only available in position corner mode')
                            target=float(target_position_deg)
                            if (not math.isfinite(target) or type(current_ma) is not int
                                    or current_ma not in (20,40,60)):
                                raise ValueError('Invalid bounded corner CB target/current')
                            now=float(clock())
                            actual=latest[1]
                            if now-float(actual['sampled_at'])>0.20:
                                raise RuntimeError('Corner CB requires fresh M1 feedback')
                            # Validate the encoded target as well as the input.
                            tx=bytearray(disabled_position_seed(1,target,directions[0]))
                            tx[11:13]=current_ma.to_bytes(2,'big')
                            encoded=int.from_bytes(tx[5:9],'big')/10 * (-1 if tx[2]^directions[0] else 1)
                            if (abs(encoded-float(actual['position']))>0.2+1e-9
                                    or abs(encoded-corner_cb_start)>2.0+1e-9):
                                raise ValueError('Corner CB target exceeds feedback lead or total travel')
                            previous, sent_at=corner_cb_previous
                            if previous is not None and (abs(encoded-previous)>0.1+1e-9 or now-sent_at<0.045):
                                raise ValueError('Corner CB step/cadence exceeds fixed limits')
                            reply=polled_exchange(bytes(tx),4,'corner_position_step_command')
                            if reply!=bytes((1,0xCB,2,0x6B)):
                                raise RuntimeError('Corner CB acknowledgement missing')
                            corner_cb_previous[:]=[encoded,float(clock())]
                            return {'accepted':True,'tx_hex':bytes(tx).hex(),'rx_hex':reply.hex(),
                                    'target_position_deg':encoded}
                        session=SimpleNamespace(send_c5=corner_send, read_feedback_43=corner_feedback,
                            send_cb=corner_cb, encoder_positive_directions=tuple(directions),
                            STOP_EVENT=api.STOP_EVENT, manual_test_emit=api.manual_test_emit,
                            corner_probe_axis_id=1, pull_directions=corner_pull,
                            corner_probe_encoder_direction_sign=1 if corner_pull[0]==directions[0] else -1)
                        probe_result=corner_phase(session,clock=clock,sleep=bounded_wait)
                        reason=str(probe_result.get('reason','角点短行程结束；不发布校准'))
                    elif torque_mode:
                        probe_result = torque_admission()
                        reason = "四轴低力矩C5接管与q0恢复已验证；尚未进行机械四角标定"
                    else:
                        probe_result = run_position_probe([q0[address] for address in IDS], snapshot=snapshot,
                            send_positions=send_positions, clock=clock, sleep=bounded_wait, emergency_stop=emergency_stop,
                            emit=lambda event: emit(event["event"], **{key: value for key, value in event.items() if key != "event"}),
                            config=ProbeConfig(tick_seconds=1.0 / COMMAND_HZ))
                        reason = "四轴0.5度编码器微幅响应及回零已验证"
                except ProbeSafetyError as exc:
                    probe_result, reason = exc.result, str(exc)
                except Exception as exc:
                    reason = str(exc)
                    emit("position_microprobe_failed", error=reason)
                finally:
                    if timer_enabled:
                        try:
                            api.set_high_resolution_timer(False)
                        except Exception as exc:
                            cleanup_errors.append(f"timer_cleanup: {exc}")
                    if owned:
                        if not cleaning:
                            deadline = float(clock()) + 15.0
                        cleaning = True
                        connection.write_budget = 0.10
                        cleanup_errors.extend(fresh_stop_disable("cleanup"))
                        not_attempted = sorted(touched)
                        try:
                            stop_reader_and_stream("cleanup_stream_stop")
                            # Fresh STOP/DISABLE after stream shutdown and fresh
                            # full disabled feedback before every full restore.
                            disable_acks = set()
                            for action, command in (("stop", 0xFE), ("disable", 0xF3)):
                                for address in IDS:
                                    packet = bytes((address, 0xFE, 0x98, 0, 0x6B)) if command == 0xFE else bytes((address, 0xF3, 0xAB, 0, 0, 0x6B))
                                    try:
                                        reply = transaction(packet, "cleanup_fresh_" + action, True)
                                        if command == 0xFE and reply == bytes((address, 0xFE, 0xE2, 0x6B)):
                                            cleanup_stop_e2_axes.append(address)
                                            emit("position_microprobe_cleanup_stop_e2", id=address,
                                                 rx_hex=reply.hex(), pending_fresh_disabled_confirmation=True)
                                        elif reply != bytes((address, command, 2, 0x6B)):
                                            raise RuntimeError(f"M{address}收尾{action}ACK未确认")
                                        elif command == 0xF3:
                                            disable_acks.add(address)
                                    except Exception as ack_error:
                                        # One bad STOP reply must never prevent
                                        # attempts to DISABLE every other axis.
                                        cleanup_errors.append(str(ack_error))
                            if disable_acks != set(IDS):
                                raise RuntimeError("四轴DISABLE ACK未全部确认，禁止恢复配置")
                            # DISABLE acknowledgement precedes mechanical
                            # settling. Wait for a fresh zero-speed sample
                            # instead of failing the entire restore on that
                            # first residual-speed sample. All other guards
                            # remain immediate and the wait is bounded.
                            settle_deadline = min(deadline, float(clock()) + 1.0)
                            while True:
                                try:
                                    read_quiet("cleanup_disabled", rpm_bound=0.0)
                                    break
                                except RuntimeError as settling_error:
                                    if ("原始转速超过接管上限" not in str(settling_error)
                                            or float(clock()) >= settle_deadline):
                                        raise
                                    emit("cleanup_wait_for_stationary", reason=str(settling_error))
                                    bounded_wait(0.04)
                            cleanup_stop_e2_disabled_confirmed = bool(cleanup_stop_e2_axes)
                            for address in sorted(touched):
                                read_quiet("restore_disabled_gate", rpm_bound=0.0 if cleanup_stop_e2_axes else 0.6)
                                current = read_config(address, "restore_current")
                                if current[:16] + current[20:] != original[address][:16] + original[address][20:]:
                                    raise RuntimeError(f"M{address}其他配置字段改变，禁止覆盖恢复")
                                verified[address] = current
                                write_config(address, original[address], "restore_original")
                                restored.append(address)
                            read_quiet("final_disabled", rpm_bound=0.0 if cleanup_stop_e2_axes else 0.6)
                            restore_complete = len(restored) == len(touched)
                        except Exception as exc:
                            cleanup_errors.append(str(exc))
                            final_disabled = False
                            try:
                                if reader is None and not stream_attempted:
                                    read_quiet("cleanup_failure_disabled", rpm_bound=0.0 if cleanup_stop_e2_axes else 0.6)
                            except Exception as final_exc:
                                cleanup_errors.append(str(final_exc))
        except Exception as exc:
            reason = str(exc)
            final_disabled = False

    intervals = [right["started_at"] - left["started_at"] for left, right in zip(command_stamps, command_stamps[1:])]
    measured_hz = (len(intervals) / sum(intervals)) if intervals and sum(intervals) > 0 else None
    skipped_ticks = None if probe_result is None else probe_result.get("skipped_ticks")
    response_passed = bool(probe_result and probe_result.get("success"))
    cadence_passed = bool(measured_hz is not None and CADENCE_MIN_HZ <= measured_hz <= CADENCE_MAX_HZ
        and skipped_ticks == 0 and intervals and max(intervals) <= CADENCE_MAX_INTERVAL_SECONDS + 1e-9)
    success = bool(response_passed and (torque_mode or cadence_passed) and final_disabled and restore_complete and not cleanup_errors)
    if response_passed and not cadence_passed and not torque_mode:
        reason = f"编码器微幅响应通过，但本轮{COMMAND_HZ:g}Hz指令节拍未通过"
    result = dict(success=success, stage="corner_stop_probe" if corner_phase is not None else ("torque_admission" if torque_mode else "position_microprobe"), reason=reason,
        response_passed=response_passed, cadence_passed=cadence_passed,
        takeover_passed=takeover_passed, q0=q0, initial_positions=initial_positions,
        current_bounds=current_bounds, zero_current_seed_results=seeds, motion_observed=motion_observed,
        preparation_read_recoveries=preparation_read_recoveries,
        unexpected_motion_observed=unexpected_motion_observed,
        final_disabled_confirmed=final_disabled, configuration_restored=restore_complete,
        configuration_restore={"restored": restored, "unknown": sorted(unknown), "not_attempted": not_attempted},
        original_configurations_hex={str(key): value.hex() for key, value in original.items()},
        cleanup_errors=cleanup_errors, requires_hardware_stop=owned and not final_disabled,
        cleanup_stop_e2_axes=cleanup_stop_e2_axes,
        cleanup_stop_e2_disabled_confirmed=cleanup_stop_e2_disabled_confirmed,
        elapsed_seconds=float(clock()) - started, stream_runs=stream_runs, probe_result=probe_result,
        axis_results=[] if probe_result is None else probe_result.get("axis_results", []),
        final_hold_verified=bool(probe_result and probe_result.get("final_hold_verified")),
        cadence={"required_for_this_result": not torque_mode, "target_hz": COMMAND_HZ, "measured_command_hz": measured_hz,
                 "passed": cadence_passed, "minimum_accepted_hz": CADENCE_MIN_HZ,
                 "maximum_accepted_hz": CADENCE_MAX_HZ, "skipped_ticks": skipped_ticks,
                 "late_intervals": sum(interval > CADENCE_MAX_INTERVAL_SECONDS + 1e-9 for interval in intervals),
                 "maximum_accepted_interval_seconds": CADENCE_MAX_INTERVAL_SECONDS,
                 "completed_commands": len(command_stamps), "max_interval_seconds": max(intervals, default=None),
                 "max_allowed_interval_seconds": MAX_COMMAND_GAP_SECONDS,
                 "commands": command_stamps}, xyz_precision_measured=False, tension_verified=False)
    if cleanup_errors:
        result["reason"] += "；收尾未完整确认：" + "; ".join(cleanup_errors)
    emit("position_microprobe_adapter_result", result=result)
    return result
