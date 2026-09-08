"""Bounded single-axis C5 corner-stop candidate probe.

This module intentionally contains no serial-port ownership, configuration
mutation, calibration publication, or controller import.  The caller supplies
one already-owned hardware session.  That keeps the probe on the same
transport/feedback owner as the existing microprobe worker instead of adding a
second serial reader.

An independent session object used by ``run_hardware_corner_stop_probe`` must
provide these small callbacks::

    capture_configuration() -> opaque snapshot
    send_c5(frame: bytes) -> {"accepted": True, ...}
    read_feedback_43() -> four complete decoded 0x43 records
    stop_disable() -> {"ok": True, ...}
    restore_configuration(snapshot) -> {"ok": True, ...}

The existing microprobe session can instead call
``run_corner_probe_phase``.  That phase only requires ``send_c5(frame)`` and
``read_feedback_43()`` (plus optional stop/event logging hooks); the caller
retains ownership of configuration, STOP/DISABLE and exact restoration.

Each feedback record is a mapping or object with ``id``, ``position``,
``rpm``, ``phase_current``, ``temperature``, ``flags``, ``sampled_at`` and
``raw_packet_hex``.  ``read_feedback_43`` must return one fresh complete batch;
the adapter does not reuse cached rows or silently accept a partial batch.

The implementation is deliberately a *candidate sampler*.  A run that moves
and then reaches a bounded low-RPM position plateau at 60 mA returns
``candidate_stop=True``.  That is evidence for a later mechanical validation,
not a confirmed stop and never a calibration result.  A run with no observed
motion always returns ``candidate_stop=False``.
"""
from __future__ import annotations

import math
import time
from collections.abc import Mapping, Sequence
from typing import Any


IDS = (1, 2, 3, 4)
HOLD_CURRENT_MA = 20
CURRENT_LEVELS_MA = (20, 40, 60)
MAX_RPM = 1.0
RPM_SAFETY_LIMIT = 1.2
CURRENT_MARGIN_MA = 30
ROUND_MAX_SECONDS = 8.0
MAX_CUMULATIVE_TRAVEL_DEG = 30.0
MAX_ABSOLUTE_TRAVEL_DEG = 30.0
MAX_FEEDBACK_AGE_SECONDS = 0.20
MAX_BATCH_SPAN_SECONDS = 0.10
SAMPLE_PERIOD_SECONDS = 0.05
LEVEL_DWELL_SECONDS = 1.0
PLATEAU_SECONDS = 0.40
PLATEAU_RPM_LIMIT = 0.30
PLATEAU_POSITION_SPAN_DEG = 0.20
MIN_MOTION_DEG = 0.20
MAX_TEMPERATURE_C = 45.0
FEEDBACK_STATUS_MASK = 0x3F
PROTECTION_FLAG = 0x08

# The C5 phase above is retained as a quarantined diagnostic.  Position-mode
# probing has a separate envelope because the real controller was observed to
# ignore the C5 speed field under load.  The position callback supplied by the
# existing serial owner enforces the same values again before transmission.
CB_STEP_CURRENT_LEVELS_MA = (40, 60)
CB_STEP_DEG = 0.10
CB_MAX_LEAD_DEG = 0.20
CB_MAX_TARGET_TRAVEL_DEG = 2.0
CB_MAX_ACTUAL_TRAVEL_DEG = 2.0
CB_MIN_COMMAND_INTERVAL_SECONDS = 0.045
CB_SAMPLE_PERIOD_SECONDS = 0.050
CB_LEVEL_DWELL_SECONDS = 1.0
# One encoder count is 0.1 degree on this device.  Require two counts before
# calling it observed motion; a single count can be quantisation/noise.
CB_MIN_MOTION_DEG = 0.20


class CornerStopProbeError(RuntimeError):
    """A bounded probe stopped before it could produce a safe result."""


_MISSING = object()


def _field(record: object, name: str, default: object = _MISSING) -> object:
    if isinstance(record, Mapping):
        value = record.get(name, default)
    else:
        value = getattr(record, name, default)
    if value is _MISSING:
        raise CornerStopProbeError(f"0x43反馈缺少字段 {name}")
    return value


def _number(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise CornerStopProbeError(f"0x43字段 {name} 类型无效")
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise CornerStopProbeError(f"0x43字段 {name} 不是数值") from None
    if not math.isfinite(result):
        raise CornerStopProbeError(f"0x43字段 {name} 非有限")
    return result


def _json_value(value: object) -> object:
    """Keep command/cleanup archives serialisable without exposing an object."""
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, (bytes, bytearray)):
        return bytes(value).hex()
    if isinstance(value, Mapping):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        return [_json_value(item) for item in value]
    return repr(value)


def c5_frame(address: int, direction: int, current_ma: int, *, max_rpm: float = MAX_RPM) -> bytes:
    """Build one fixed, bounded X-firmware 0xC5 command.

    The adapter only permits the three currents used by the candidate probe,
    a 200 mA/s slope, and 1 RPM.  This function is pure and is useful to the
    parent worker's existing serial owner; it does not transmit anything.
    """
    if type(address) is not int or address not in IDS:
        raise ValueError("C5电机ID必须是1..4")
    if direction not in (0, 1):
        raise ValueError("C5方向必须是0或1")
    if type(current_ma) is not int or current_ma not in CURRENT_LEVELS_MA:
        raise ValueError("候选止挡C5电流只能是20/40/60mA")
    if float(max_rpm) != MAX_RPM:
        raise ValueError("候选止挡C5速度固定为1RPM")
    return bytes(
        (
            address,
            0xC5,
            direction,
            0x00,
            0xC8,  # fixed 200 mA/s slope
            (current_ma >> 8) & 0xFF,
            current_ma & 0xFF,
            0x00,
            0x00,
            0x0A,  # 1.0 RPM in 0.1 RPM units
            0x6B,
        )
    )


def _resolve_config(api: object, axis_id: int | None, pull_directions: Sequence[int] | None,
                    encoder_direction_sign: int | None) -> tuple[int, tuple[int, ...], int]:
    """Resolve one-argument loader calls without guessing a mechanical axis."""
    if axis_id is None:
        candidate = getattr(api, "corner_probe_axis_id", None)
        axis_id = candidate() if callable(candidate) else candidate
    if pull_directions is None:
        candidate = getattr(api, "pull_directions", None)
        pull_directions = candidate() if callable(candidate) else candidate
    if encoder_direction_sign is None:
        candidate = getattr(api, "corner_probe_encoder_direction_sign", None)
        if candidate is None:
            candidate = getattr(api, "encoder_direction_sign", None)
        encoder_direction_sign = candidate() if callable(candidate) else candidate
    if type(axis_id) is not int or axis_id not in IDS:
        raise ValueError("必须显式提供已验证的corner_probe_axis_id")
    if pull_directions is None or len(pull_directions) != len(IDS):
        raise ValueError("必须提供四轴已验证的C5 pull_directions")
    directions = tuple(int(value) for value in pull_directions)
    if any(value not in (0, 1) for value in directions):
        raise ValueError("C5 pull_directions必须全部是0或1")
    if encoder_direction_sign not in (-1, 1):
        raise ValueError("必须提供目标轴已验证的编码器方向符号(+1/-1)")
    return axis_id, directions, int(encoder_direction_sign)


def _stop_requested(api: object) -> bool:
    callback = getattr(api, "stop_requested", None)
    if callable(callback):
        return bool(callback())
    event = getattr(api, "STOP_EVENT", None)
    return bool(event is not None and callable(getattr(event, "is_set", None)) and event.is_set())


def _emit(api: object, event: str, **fields: object) -> None:
    callback = getattr(api, "emit", None)
    if callback is None:
        callback = getattr(api, "manual_test_emit", None)
    if not callable(callback):
        return
    try:
        callback(event, **fields)
    except Exception:
        # Telemetry must not prevent the caller's emergency cleanup.
        return


def _accepted(reply: object, label: str) -> bool:
    if isinstance(reply, bool):
        return reply
    if isinstance(reply, Mapping):
        for key in ("accepted", "ok", "success"):
            if key in reply:
                return bool(reply[key])
    raise CornerStopProbeError(f"{label}未返回可验证的accepted/ok结果")


def _cleanup_ok(reply: object, label: str) -> bool:
    if isinstance(reply, bool):
        return reply
    if isinstance(reply, Mapping):
        if "ok" in reply:
            return bool(reply["ok"])
        if "success" in reply:
            return bool(reply["success"])
        if label == "stop_disable" and {"stopped", "disabled"}.issubset(reply):
            return bool(reply["stopped"] and reply["disabled"])
        if label == "restore_configuration" and "restored" in reply:
            return bool(reply["restored"])
    raise CornerStopProbeError(f"{label}未返回可验证的收尾结果")


def _record_copy(record: object) -> dict[str, object]:
    """Copy all known feedback fields before validation, including bad frames."""
    fields = {}
    for name in ("id", "position", "rpm", "phase_current", "temperature", "flags", "sampled_at",
                 "raw_packet_hex", "raw_packet"):
        value = _field(record, name, None)
        if value is not None:
            fields[name] = _json_value(value)
    if not fields:
        fields["record_repr"] = repr(record)
    return fields


def _validate_batch(
    records: object,
    *,
    now: float,
    command_currents: Sequence[int],
    raw_archive: list[dict[str, object]],
    phase: str,
) -> list[dict[str, float | int | str]]:
    if isinstance(records, Mapping):
        if "id" in records:
            sequence = [records]
        else:
            sequence = list(records.values())
    else:
        try:
            sequence = list(records)  # type: ignore[arg-type]
        except TypeError:
            raise CornerStopProbeError("read_feedback_43未返回四轴反馈批次") from None
    if len(sequence) != len(IDS):
        raise CornerStopProbeError(f"0x43反馈批次不完整: {len(sequence)}/4")
    # Archive the complete batch before validating any individual row.  If the
    # first row is the bad one, the remaining raw packets must still be
    # available for diagnosis rather than disappearing with the exception.
    archived_rows = []
    for record in sequence:
        copied = _record_copy(record)
        copied["phase"] = phase
        archived_rows.append(copied)
        raw_archive.append(copied)
    rows: list[dict[str, float | int | str]] = []
    seen: set[int] = set()
    stamps: list[float] = []
    for record, copied in zip(sequence, archived_rows):
        try:
            address = _field(record, "id")
            if type(address) is not int or address not in IDS or address in seen:
                raise CornerStopProbeError("0x43反馈ID重复或无效")
            seen.add(address)
            position = _number(_field(record, "position"), "position")
            rpm = _number(_field(record, "rpm"), "rpm")
            current = _number(_field(record, "phase_current"), "phase_current")
            temperature = _number(_field(record, "temperature"), "temperature")
            sampled_at = _number(_field(record, "sampled_at"), "sampled_at")
            flags = _field(record, "flags")
            if type(flags) is not int or flags & ~FEEDBACK_STATUS_MASK or flags & PROTECTION_FLAG or not flags & 1:
                raise CornerStopProbeError(f"M{address} 0x43使能/保护状态无效")
            raw_hex = _field(record, "raw_packet_hex", None)
            if raw_hex is None:
                raw_packet = _field(record, "raw_packet", None)
                raw_hex = bytes(raw_packet).hex() if raw_packet is not None else None
            if not isinstance(raw_hex, str):
                raise CornerStopProbeError(f"M{address}缺少原始0x43帧")
            packet = bytes.fromhex(raw_hex)
            if len(packet) != 37 or packet[:4] != bytes((address, 0x43, 0x25, 0x0C)) or packet[-1] != 0x6B:
                raise CornerStopProbeError(f"M{address}原始0x43帧格式无效")
            age = now - sampled_at
            if age < -0.02 or age > MAX_FEEDBACK_AGE_SECONDS:
                raise CornerStopProbeError(f"M{address}0x43反馈陈旧")
            if abs(rpm) > RPM_SAFETY_LIMIT:
                raise CornerStopProbeError(f"M{address}转速超过{RPM_SAFETY_LIMIT:g}RPM")
            if current < 0 or current > command_currents[address - 1] + CURRENT_MARGIN_MA:
                raise CornerStopProbeError(
                    f"M{address}相电流超过当前指令+{CURRENT_MARGIN_MA}mA"
                )
            if temperature >= MAX_TEMPERATURE_C:
                raise CornerStopProbeError(f"M{address}温度达到{MAX_TEMPERATURE_C:g}度")
            stamps.append(sampled_at)
            rows.append({"id": address, "position": position, "rpm": rpm, "phase_current": current,
                         "temperature": temperature, "flags": flags, "sampled_at": sampled_at,
                         "raw_packet_hex": raw_hex})
        except ValueError as exc:
            raise CornerStopProbeError(f"M反馈原始帧解析失败: {exc}") from None
    if seen != set(IDS):
        raise CornerStopProbeError("0x43反馈未覆盖四轴")
    if max(stamps) - min(stamps) > MAX_BATCH_SPAN_SECONDS:
        raise CornerStopProbeError("四轴0x43反馈时间跨度超过0.1秒")
    rows.sort(key=lambda row: int(row["id"]))
    return rows


def run_hardware_corner_stop_probe(
    api: object,
    *,
    axis_id: int | None = None,
    pull_directions: Sequence[int] | None = None,
    encoder_direction_sign: int | None = None,
    clock=time.perf_counter,
    sleep=None,
    manage_cleanup: bool = True,
) -> dict[str, Any]:
    """Run one bounded candidate round on one axis.

    The optional configuration keywords make the function convenient for
    offline tests.  The production loader may call ``run_hardware_corner_stop_probe(api)``
    when the API exposes ``corner_probe_axis_id``, ``pull_directions`` and
    ``corner_probe_encoder_direction_sign``.  Missing configuration fails
    before ``capture_configuration`` and therefore before any hardware write.
    Set ``manage_cleanup=False`` only through
    ``run_corner_probe_phase``; in that mode this function never claims that
    the caller's session was disabled or restored.
    """
    # The production loader passes the controller module, whose existing
    # microprobe owner already performs preflight, polling, STOP/DISABLE and
    # exact configuration restoration.  Delegate to that owner instead of
    # inventing a second serial stack.  Offline/session tests that provide the
    # explicit capture/send/read/cleanup seam continue through the small
    # independent implementation below.
    if (
        manage_cleanup
        and axis_id is None
        and pull_directions is None
        and encoder_direction_sign is None
        and not callable(getattr(api, "capture_configuration", None))
        and callable(getattr(api, "current_port", None))
        and hasattr(api, "direct")
    ):
        try:
            from y42_adapter_versions import load_microprobe_adapter
            owner, owner_digest = load_microprobe_adapter()
            _emit(api, "corner_stop_probe_transport_version", source_sha256=owner_digest)
            return owner.run_hardware_corner_probe(
                api,
                run_corner_probe_phase,
                clock=clock,
                sleep=sleep,
            )
        except Exception:
            raise
    axis_id, directions, encoder_sign = _resolve_config(
        api, axis_id, pull_directions, encoder_direction_sign
    )
    required = ("send_c5", "read_feedback_43")
    if manage_cleanup:
        required += ("capture_configuration", "stop_disable", "restore_configuration")
    missing = [name for name in required if not callable(getattr(api, name, None))]
    if missing:
        raise TypeError("corner-stop-probe API缺少: " + ", ".join(missing))

    started = float(clock())
    deadline = started + ROUND_MAX_SECONDS
    raw_feedback: list[dict[str, object]] = []
    commands: list[dict[str, object]] = []
    levels: list[dict[str, object]] = []
    cleanup_errors: list[str] = []
    snapshot: object = None
    owned = False
    stop_result: object = None
    restore_result: object = None
    failure_reason: str | None = None
    previous_position = None
    start_position = None
    cumulative_travel = 0.0
    max_absolute_travel = 0.0
    max_directional_progress = 0.0
    movement_observed = False
    candidate_stop = False
    plateau_since = None
    plateau_min = None
    plateau_max = None
    last_rows: list[dict[str, object]] = []

    def check() -> None:
        if _stop_requested(api):
            raise CornerStopProbeError("用户停止候选止挡探测")
        if float(clock()) >= deadline:
            raise CornerStopProbeError("单角候选探测超过8秒预算")

    def bounded_sleep(seconds: float) -> None:
        seconds = max(0.0, min(float(seconds), max(0.0, deadline - float(clock()))))
        if seconds <= 0:
            return
        if sleep is None:
            time.sleep(seconds)
        else:
            sleep(seconds)

    def send_command(command_axis: int, level_ma: int, phase: str) -> None:
        check()
        currents = tuple(level_ma if address == command_axis else HOLD_CURRENT_MA for address in IDS)
        # Use one addressed 11-byte C5 command.  The four-axis low hold is
        # established explicitly before the active axis is escalated; no
        # unverified aggregate/0xAA path is involved.
        frame = c5_frame(command_axis, directions[command_axis - 1], level_ma, max_rpm=MAX_RPM)
        entry: dict[str, object] = {
            "axis_id": command_axis,
            "level_ma": level_ma,
            "currents_ma": list(currents),
            "phase": phase,
            "max_rpm": MAX_RPM,
            "frame_hex": frame.hex(),
            "started_at": float(clock()),
        }
        try:
            reply = api.send_c5(frame)
            entry["reply"] = _json_value(reply)
            entry["accepted"] = _accepted(reply, "C5命令")
            if not entry["accepted"]:
                raise CornerStopProbeError("C5命令未确认accepted")
        except Exception as exc:
            entry["error"] = str(exc)
            raise
        finally:
            entry["completed_at"] = float(clock())
            commands.append(entry)
            _emit(api, "corner_stop_probe_command", **entry)

    def sample_feedback(level_ma: int, phase: str, *, allow_plateau: bool) -> None:
        """Read/guard one complete batch and update the motion evidence."""
        nonlocal previous_position, start_position, cumulative_travel
        nonlocal max_absolute_travel, max_directional_progress, movement_observed
        nonlocal plateau_since, plateau_min, plateau_max, candidate_stop, last_rows
        try:
            records = api.read_feedback_43()
            rows = _validate_batch(
                records,
                now=float(clock()),
                command_currents=tuple(level_ma if address == axis_id else HOLD_CURRENT_MA
                                       for address in IDS),
                raw_archive=raw_feedback,
                phase=phase,
            )
        except Exception as exc:
            if not isinstance(exc, CornerStopProbeError):
                exc = CornerStopProbeError(str(exc))
            _emit(api, "corner_stop_probe_feedback_failure", phase=phase, error=str(exc))
            raise exc
        last_rows = [dict(row) for row in rows]
        active = rows[axis_id - 1]
        position = float(active["position"])
        rpm = abs(float(active["rpm"]))
        if start_position is None:
            start_position = position
        if previous_position is not None:
            cumulative_travel += abs(position - previous_position)
        previous_delta = 0.0 if previous_position is None else abs(position - previous_position)
        previous_position = position
        absolute_travel = abs(position - start_position)
        directional_progress = (position - start_position) * encoder_sign
        max_absolute_travel = max(max_absolute_travel, absolute_travel)
        max_directional_progress = max(max_directional_progress, directional_progress)
        if cumulative_travel > MAX_CUMULATIVE_TRAVEL_DEG + 1e-9:
            raise CornerStopProbeError("累计角位移超过30度")
        if max_absolute_travel > MAX_ABSOLUTE_TRAVEL_DEG + 1e-9:
            raise CornerStopProbeError("相对起点角位移超过30度")
        if directional_progress < -MIN_MOTION_DEG:
            raise CornerStopProbeError("主动轴反馈方向与C5拉绳方向相反")
        if directional_progress >= MIN_MOTION_DEG:
            movement_observed = True

        if allow_plateau:
            stable_sample = (level_ma == CURRENT_LEVELS_MA[-1] and movement_observed
                             and rpm <= PLATEAU_RPM_LIMIT and previous_delta <= 0.10)
            if stable_sample:
                plateau_min = position if plateau_min is None else min(plateau_min, position)
                plateau_max = position if plateau_max is None else max(plateau_max, position)
                if plateau_max - plateau_min > PLATEAU_POSITION_SPAN_DEG:
                    plateau_since = None
                    plateau_min = plateau_max = position
                elif plateau_since is None:
                    plateau_since = float(clock())
                elif float(clock()) - plateau_since >= PLATEAU_SECONDS:
                    candidate_stop = True
                    _emit(api, "corner_stop_probe_candidate", axis_id=axis_id,
                          current_level_ma=level_ma, position=position,
                          directional_progress=directional_progress,
                          plateau_seconds=float(clock()) - plateau_since,
                          evidence="motion_then_final_level_low_rpm_position_plateau")
            else:
                plateau_since = None
                plateau_min = plateau_max = None

    try:
        if manage_cleanup:
            snapshot = api.capture_configuration()
            owned = True
        _emit(api, "corner_stop_probe_started", axis_id=axis_id,
              pull_directions=list(directions), encoder_direction_sign=encoder_sign,
              current_levels_ma=list(CURRENT_LEVELS_MA), max_rpm=MAX_RPM,
              max_round_seconds=ROUND_MAX_SECONDS, max_travel_deg=MAX_CUMULATIVE_TRAVEL_DEG)
        # Establish a real low-force C5 hold on every axis.  A CB q0 target is
        # a position-loop ceiling, not a 20mA force hold, and must not be used
        # to infer that the passive ropes stayed taut.  Every command is
        # immediately followed by a fresh four-axis 0x43 batch.
        for hold_axis in IDS:
            hold_phase = f"initial_hold_m{hold_axis}"
            send_command(hold_axis, HOLD_CURRENT_MA, hold_phase)
            sample_feedback(HOLD_CURRENT_MA, hold_phase, allow_plateau=False)

        for level_index, level_ma in enumerate(CURRENT_LEVELS_MA):
            phase = f"level_{level_ma}"
            send_command(axis_id, level_ma, phase)
            level_started = float(clock())
            level_limit = min(deadline, level_started + LEVEL_DWELL_SECONDS) if level_index < 2 else deadline
            plateau_since = None
            plateau_min = plateau_max = None
            sample_count = 0
            while float(clock()) < level_limit:
                check()
                sample_feedback(level_ma, phase, allow_plateau=True)
                if candidate_stop:
                    break
                sample_count += 1
                bounded_sleep(min(SAMPLE_PERIOD_SECONDS,
                                  max(0.0, level_limit - float(clock()))))
            levels.append({"current_level_ma": level_ma, "started_at": level_started,
                           "completed_at": float(clock()), "feedback_samples": sample_count,
                           "candidate_stop": candidate_stop})
            if candidate_stop:
                break
        if candidate_stop:
            stop_reason = "motion_then_final_level_plateau_candidate"
        elif not movement_observed:
            stop_reason = "insufficient_motion"
        else:
            stop_reason = "motion_observed_but_no_bounded_plateau"
    except Exception as exc:
        failure_reason = str(exc)
        stop_reason = "safety_abort"
        _emit(api, "corner_stop_probe_failed", error=failure_reason)
    finally:
        if owned:
            try:
                stop_result = api.stop_disable()
                if not _cleanup_ok(stop_result, "stop_disable"):
                    raise CornerStopProbeError("失能收尾未确认")
            except Exception as exc:
                cleanup_errors.append(f"stop_disable: {exc}")
            try:
                restore_result = api.restore_configuration(snapshot)
                if not _cleanup_ok(restore_result, "restore_configuration"):
                    raise CornerStopProbeError("配置恢复未确认")
            except Exception as exc:
                cleanup_errors.append(f"restore_configuration: {exc}")

    cleanup_ok = not cleanup_errors
    phase_success = bool(failure_reason is None)
    success = bool(phase_success and (cleanup_ok if manage_cleanup else True))
    result: dict[str, Any] = {
        "success": success,
        "axis_id": axis_id,
        "pull_directions": list(directions),
        "encoder_direction_sign": encoder_sign,
        "movement_observed": movement_observed,
        "candidate_stop": bool(candidate_stop and movement_observed),
        "evidence": ("motion_then_final_level_low_rpm_position_plateau"
                      if candidate_stop and movement_observed else None),
        "reason": stop_reason,
        "stop_reason": stop_reason,
        "failure_reason": failure_reason,
        "start_position_deg": start_position,
        "end_position_deg": (last_rows[axis_id - 1]["position"] if last_rows else None),
        "max_directional_progress_deg": max_directional_progress,
        "max_absolute_travel_deg": max_absolute_travel,
        "cumulative_travel_deg": cumulative_travel,
        "commands": commands,
        "levels": levels,
        "raw_feedback": raw_feedback,
        "stop_disable": _json_value(stop_result) if manage_cleanup else None,
        "configuration_restore": _json_value(restore_result) if manage_cleanup else None,
        "cleanup_errors": cleanup_errors,
        "cleanup_owned_by_caller": not manage_cleanup,
        "final_disabled_confirmed": (bool(cleanup_ok and stop_result is not None)
                                      if manage_cleanup else None),
        "configuration_restored": (bool(cleanup_ok and restore_result is not None)
                                    if manage_cleanup else None),
        "calibration_published": False,
        "requires_external_mechanical_validation": True,
        "elapsed_seconds": float(clock()) - started,
    }
    # A safe no-motion run is useful evidence, but must never look like a
    # successful corner discovery.
    if not movement_observed and failure_reason is None:
        result["success"] = bool(cleanup_ok)
        result["candidate_stop"] = False
        result["evidence"] = None
    _emit(api, "corner_stop_probe_result", result=result)
    return result


def run_corner_probe_phase(
    api: object,
    *,
    axis_id: int | None = None,
    pull_directions: Sequence[int] | None = None,
    encoder_direction_sign: int | None = None,
    clock=time.perf_counter,
    sleep=None,
) -> dict[str, Any]:
    """Run only the bounded motion phase inside an already-owned session.

    The caller must perform the existing session's preflight, STOP/DISABLE and
    exact configuration restore.  This function never opens a port, changes a
    global envelope, disables a motor, or reports cleanup as successful.  It is
    the intended seam for the production worker's existing
    ``_run_hardware_probe`` path.
    """
    return run_hardware_corner_stop_probe(
        api,
        axis_id=axis_id,
        pull_directions=pull_directions,
        encoder_direction_sign=encoder_direction_sign,
        clock=clock,
        sleep=sleep,
        manage_cleanup=False,
    )


def _resolve_cb_step_config(
    api: object,
    axis_id: int | None,
    pull_directions: Sequence[int] | None,
    encoder_positive_directions: Sequence[int] | None,
) -> tuple[int, tuple[int, ...], tuple[int, ...], int]:
    """Resolve runtime direction data for the bounded CB step phase.

    ``pull_directions`` is the live C5 direction source and
    ``encoder_positive_directions`` is the live absolute-position direction
    source.  There is deliberately no fallback to the legacy module
    constant: using a stale direction would make an otherwise bounded CB
    command pull the wrong cable.
    """
    if axis_id is None:
        candidate = getattr(api, "corner_probe_axis_id", None)
        axis_id = candidate() if callable(candidate) else candidate
    if type(axis_id) is not int or axis_id != 1:
        raise ValueError("CB短行程阶段只允许已核验的M1")

    def resolve(value: object, name: str) -> tuple[int, ...]:
        try:
            values = tuple(value)  # type: ignore[arg-type]
        except TypeError:
            raise ValueError(f"必须提供运行时{name}") from None
        if len(values) != len(IDS):
            raise ValueError(f"{name}必须覆盖四轴")
        result = []
        for item in values:
            if isinstance(item, bool):
                raise ValueError(f"{name}只能是0或1")
            try:
                converted = int(item)
            except (TypeError, ValueError):
                raise ValueError(f"{name}只能是0或1") from None
            if converted not in (0, 1) or converted != item:
                raise ValueError(f"{name}只能是0或1")
            result.append(converted)
        return tuple(result)

    if pull_directions is None:
        candidate = getattr(api, "pull_directions", None)
        pull_directions = candidate() if callable(candidate) else candidate
    if encoder_positive_directions is None:
        candidate = getattr(api, "encoder_positive_directions", None)
        encoder_positive_directions = candidate() if callable(candidate) else candidate
    pulls = resolve(pull_directions, "pull_directions")
    positives = resolve(encoder_positive_directions, "encoder_positive_directions")
    # A CB target that increases the encoder position pulls in the C5
    # direction exactly when the two driver direction bits match.
    step_sign = 1 if pulls[0] == positives[0] else -1
    return axis_id, pulls, positives, step_sign


def run_corner_cb_step_phase(
    api: object,
    *,
    axis_id: int | None = None,
    pull_directions: Sequence[int] | None = None,
    encoder_positive_directions: Sequence[int] | None = None,
    clock=time.perf_counter,
    sleep=None,
) -> dict[str, Any]:
    """Run a cleanup-free, position-mode M1 short-step response check.

    This is a separate phase from :func:`run_corner_probe_phase`.  The
    observed C5 speed field is not trusted for active motion, so M1 never
    receives a C5 command here.  M2--M4 receive only a 20 mA C5 hold; M1 is
    advanced by the existing owner's ``send_cb(target, current)`` callback in
    0.1 degree steps.  The callback performs the final frame construction and
    transport checks, while this phase enforces the sequencing and records
    every command and fresh four-axis feedback batch.

    The phase owns no serial port and never disables or restores anything.  A
    successful return means only that the bounded position command/feedback
    path stayed inside its guard envelope.  It is not a mechanical stop,
    corner, calibration, or trajectory result.
    """
    axis_id, pulls, positives, step_sign = _resolve_cb_step_config(
        api, axis_id, pull_directions, encoder_positive_directions
    )
    required = ("send_c5", "send_cb", "read_feedback_43")
    missing = [name for name in required if not callable(getattr(api, name, None))]
    if missing:
        raise TypeError("CB短行程phase API缺少: " + ", ".join(missing))

    started = float(clock())
    deadline = started + ROUND_MAX_SECONDS
    raw_feedback: list[dict[str, object]] = []
    commands: list[dict[str, object]] = []
    levels: list[dict[str, object]] = []
    failure_reason: str | None = None
    stop_reason = "尚未开始"
    start_position: float | None = None
    previous_position: float | None = None
    last_rows: list[dict[str, object]] = []
    last_target: float | None = None
    last_cb_at: float | None = None
    cumulative_travel = 0.0
    max_absolute_travel = 0.0
    max_target_travel = 0.0
    max_lead = 0.0
    movement_observed = False
    target_limit_reached = False
    active_commanded = False

    def check() -> None:
        if _stop_requested(api):
            raise CornerStopProbeError("用户停止CB短行程验证")
        if float(clock()) >= deadline:
            raise CornerStopProbeError("CB短行程阶段超过8秒预算")

    def bounded_sleep(seconds: float) -> None:
        seconds = max(0.0, min(float(seconds), max(0.0, deadline - float(clock()))))
        if seconds <= 0.0:
            return
        if sleep is None:
            time.sleep(seconds)
        else:
            sleep(seconds)

    def sample_feedback(current_ma: int, phase: str, *, track_motion: bool) -> None:
        nonlocal start_position, previous_position, cumulative_travel
        nonlocal max_absolute_travel, movement_observed, last_rows
        try:
            records = api.read_feedback_43()
            rows = _validate_batch(
                records,
                now=float(clock()),
                command_currents=tuple(
                    current_ma if address == axis_id else HOLD_CURRENT_MA for address in IDS
                ),
                raw_archive=raw_feedback,
                phase=phase,
            )
        except Exception as exc:
            if not isinstance(exc, CornerStopProbeError):
                exc = CornerStopProbeError(str(exc))
            _emit(api, "corner_cb_step_feedback_failure", phase=phase, error=str(exc))
            raise exc
        last_rows = [dict(row) for row in rows]
        position = float(rows[axis_id - 1]["position"])
        if start_position is None:
            start_position = position
        if previous_position is not None:
            cumulative_travel += abs(position - previous_position)
        previous_position = position
        absolute_travel = abs(position - start_position)
        max_absolute_travel = max(max_absolute_travel, absolute_travel)
        if cumulative_travel > MAX_CUMULATIVE_TRAVEL_DEG + 1e-9:
            raise CornerStopProbeError("CB累计角位移超过30度")
        if track_motion:
            directional_progress = (position - start_position) * step_sign
            if directional_progress < -CB_MIN_MOTION_DEG - 1e-9:
                raise CornerStopProbeError("CB主动轴反馈方向与运行时拉绳方向相反")
            if directional_progress >= CB_MIN_MOTION_DEG:
                movement_observed = True
        if absolute_travel > CB_MAX_ACTUAL_TRAVEL_DEG + 1e-9:
            raise CornerStopProbeError("CB反馈相对phase起点超过2度")

    def send_passive_c5(command_axis: int, phase: str) -> None:
        check()
        frame = c5_frame(command_axis, pulls[command_axis - 1], HOLD_CURRENT_MA)
        entry: dict[str, object] = {
            "kind": "C5",
            "axis_id": command_axis,
            "current_ma": HOLD_CURRENT_MA,
            "currents_ma": [HOLD_CURRENT_MA] * len(IDS),
            "phase": phase,
            "max_rpm": MAX_RPM,
            "frame_hex": frame.hex(),
            "started_at": float(clock()),
        }
        try:
            reply = api.send_c5(frame)
            entry["reply"] = _json_value(reply)
            entry["accepted"] = _accepted(reply, "被动轴C5命令")
            if not entry["accepted"]:
                raise CornerStopProbeError("被动轴C5命令未确认accepted")
        except Exception as exc:
            entry["error"] = str(exc)
            raise
        finally:
            entry["completed_at"] = float(clock())
            commands.append(entry)
            _emit(api, "corner_cb_step_c5", **entry)

    def send_position_step(target: float, current_ma: int, phase: str) -> None:
        nonlocal last_target, last_cb_at, max_target_travel, max_lead
        check()
        if start_position is None or previous_position is None:
            raise CornerStopProbeError("CB目标发送前没有fresh M1反馈")
        if type(current_ma) is not int or current_ma not in CB_STEP_CURRENT_LEVELS_MA:
            raise CornerStopProbeError("CB当前阶段电流只能是40/60mA")
        if not math.isfinite(float(target)):
            raise CornerStopProbeError("CB目标角度非有限")
        target = float(target)
        # A fresh reverse-direction sample is a transport/mechanical fault,
        # not an ordinary position lag.  Check it before the lead envelope so
        # the archive preserves the useful direction diagnosis instead of
        # reporting a secondary "lead > 0.2°" error.
        if previous_position is not None and (previous_position - start_position) * step_sign <= -CB_MIN_MOTION_DEG + 1e-9:
            raise CornerStopProbeError("CB主动轴反馈方向与运行时拉绳方向相反")
        lead = abs(target - previous_position)
        target_progress = (target - start_position) * step_sign
        if lead > CB_MAX_LEAD_DEG + 1e-9:
            raise CornerStopProbeError("CB目标相对最新反馈超过0.2度")
        if target_progress < -1e-9 or target_progress > CB_MAX_TARGET_TRAVEL_DEG + 1e-9:
            raise CornerStopProbeError("CB目标相对phase起点超过2度")
        if last_target is not None and abs(target - last_target) > CB_STEP_DEG + 1e-9:
            raise CornerStopProbeError("CB相邻目标步长超过0.1度")
        now = float(clock())
        if last_cb_at is not None and now - last_cb_at < CB_MIN_COMMAND_INTERVAL_SECONDS - 1e-9:
            raise CornerStopProbeError("CB相邻指令间隔不足45毫秒")
        # Feedback can quantise one count ahead of the target.  A target that
        # trails the latest sample by <=0.2° is safe and is held/rebased on the
        # next tick; rejecting that normal closed-loop lag caused false stops.
        entry: dict[str, object] = {
            "kind": "CB",
            "axis_id": axis_id,
            "target_position_deg": target,
            "current_ma": current_ma,
            "lead_deg": lead,
            "target_progress_deg": target_progress,
            "phase": phase,
            "started_at": now,
        }
        try:
            reply = api.send_cb(target, current_ma)
            entry["reply"] = _json_value(reply)
            entry["accepted"] = _accepted(reply, "CB位置命令")
            if not entry["accepted"]:
                raise CornerStopProbeError("CB位置命令未确认accepted")
        except Exception as exc:
            entry["error"] = str(exc)
            raise
        finally:
            entry["completed_at"] = float(clock())
            commands.append(entry)
            _emit(api, "corner_cb_step_command", **entry)
        last_target = target
        last_cb_at = float(clock())
        max_target_travel = max(max_target_travel, abs(target - start_position))
        max_lead = max(max_lead, lead)

    try:
        _emit(
            api,
            "corner_cb_step_started",
            axis_id=axis_id,
            pull_directions=list(pulls),
            encoder_positive_directions=list(positives),
            encoder_step_sign=step_sign,
            current_levels_ma=list(CB_STEP_CURRENT_LEVELS_MA),
            step_deg=CB_STEP_DEG,
            max_lead_deg=CB_MAX_LEAD_DEG,
            max_target_travel_deg=CB_MAX_TARGET_TRAVEL_DEG,
            max_round_seconds=ROUND_MAX_SECONDS,
        )
        # This is the only starting reference.  No q0/calibration value is
        # accepted as a substitute for the fresh raw 0x43 batch.
        sample_feedback(HOLD_CURRENT_MA, "fresh_start", track_motion=False)
        # M1 stays on its existing CB q0/20mA seed.  Sending an active C5 here
        # would reintroduce the observed uncontrolled-speed failure mode.
        for passive_axis in (2, 3, 4):
            phase = f"passive_hold_m{passive_axis}"
            send_passive_c5(passive_axis, phase)
            sample_feedback(HOLD_CURRENT_MA, phase, track_motion=False)

        for current_ma in CB_STEP_CURRENT_LEVELS_MA:
            phase = f"cb_level_{current_ma}"
            level_started = float(clock())
            level_limit = min(deadline, level_started + CB_LEVEL_DWELL_SECONDS) \
                if current_ma == CB_STEP_CURRENT_LEVELS_MA[0] else deadline
            sample_count = 0
            while float(clock()) < level_limit:
                check()
                # Reserve a full feedback interval.  This also guarantees the
                # bridge's >=45ms inter-command guard at a current-level edge.
                if float(clock()) + CB_SAMPLE_PERIOD_SECONDS > level_limit + 1e-9:
                    break
                assert previous_position is not None
                assert start_position is not None
                if last_target is None:
                    target = previous_position + step_sign * CB_STEP_DEG
                else:
                    # Advance from the last accepted target, rather than from
                    # a possibly quantised/jumpy feedback sample.  This keeps
                    # adjacent commands at one count.  If feedback lags far
                    # enough that the next target would lead it by >0.2
                    # degrees, repeat the last target and wait for it.
                    proposed = last_target + step_sign * CB_STEP_DEG
                    signed_gap = (proposed - previous_position) * step_sign
                    if signed_gap < -1e-9:
                        # A single quantised count can put feedback exactly
                        # one step ahead of the target.  Synchronise to that
                        # current position only when the adjacent target
                        # change remains one count; a larger jump cannot be
                        # followed safely under the bridge contract.
                        if abs(previous_position - last_target) <= CB_STEP_DEG + 1e-9:
                            target = previous_position
                        else:
                            raise CornerStopProbeError(
                                "CB反馈超前超过单步，不能在0.1度相邻限制内追随"
                            )
                    elif abs(proposed - previous_position) > CB_MAX_LEAD_DEG + 1e-9:
                        target = last_target
                    else:
                        target = proposed
                target_progress = (target - start_position) * step_sign
                if target_progress > CB_MAX_TARGET_TRAVEL_DEG + 1e-9:
                    target_limit_reached = True
                    stop_reason = "position_target_limit_reached"
                    break
                send_position_step(target, current_ma, phase)
                active_commanded = True
                bounded_sleep(CB_SAMPLE_PERIOD_SECONDS)
                sample_feedback(current_ma, phase, track_motion=True)
                sample_count += 1
                if max_absolute_travel >= CB_MAX_ACTUAL_TRAVEL_DEG - 1e-9:
                    target_limit_reached = True
                    stop_reason = "position_actual_limit_reached"
                    break
            levels.append(
                {
                    "current_level_ma": current_ma,
                    "started_at": level_started,
                    "completed_at": float(clock()),
                    "feedback_samples": sample_count,
                    "target_limit_reached": target_limit_reached,
                }
            )
            if target_limit_reached:
                break
        if failure_reason is None:
            if target_limit_reached:
                pass
            elif not movement_observed:
                stop_reason = "insufficient_motion"
            else:
                stop_reason = "bounded_position_phase_complete"
    except Exception as exc:
        failure_reason = str(exc)
        stop_reason = "safety_abort"
        _emit(api, "corner_cb_step_failed", error=failure_reason)

    final_position = previous_position
    result: dict[str, Any] = {
        "success": bool(failure_reason is None),
        "phase_passed": bool(failure_reason is None and movement_observed),
        "axis_id": axis_id,
        "pull_directions": list(pulls),
        "encoder_positive_directions": list(positives),
        "encoder_step_sign": step_sign,
        "movement_observed": movement_observed,
        "position_response": (
            "bounded_motion_observed" if movement_observed else "insufficient_motion"
        ),
        "candidate_stop": False,
        "evidence": None,
        "reason": stop_reason,
        "stop_reason": stop_reason,
        "failure_reason": failure_reason,
        "start_position_deg": start_position,
        "end_position_deg": final_position,
        "max_target_travel_deg": max_target_travel,
        "max_lead_deg": max_lead,
        "max_absolute_travel_deg": max_absolute_travel,
        "cumulative_travel_deg": cumulative_travel,
        "commands": commands,
        "levels": levels,
        "raw_feedback": raw_feedback,
        "target_limit_reached": target_limit_reached,
        "active_commanded": active_commanded,
        "cleanup_owned_by_caller": True,
        "final_disabled_confirmed": None,
        "configuration_restored": None,
        "calibration_published": False,
        "requires_external_mechanical_validation": True,
        "requires_full_four_corner_calibration": True,
        "elapsed_seconds": float(clock()) - started,
    }
    _emit(api, "corner_cb_step_result", result=result)
    return result


def run_hardware_corner_cb_step_probe(
    api: object,
    *,
    axis_id: int | None = None,
    pull_directions: Sequence[int] | None = None,
    encoder_positive_directions: Sequence[int] | None = None,
    clock=time.perf_counter,
    sleep=None,
) -> dict[str, Any]:
    """Named production seam; the existing owner supplies the session.

    This wrapper intentionally does not call the cleanup-owning C5 adapter or
    open a second transport.  The root worker should pass the returned phase
    into its existing ``run_hardware_corner_probe(..., position_mode=True)``
    bridge.
    """
    return run_corner_cb_step_phase(
        api,
        axis_id=axis_id,
        pull_directions=pull_directions,
        encoder_positive_directions=encoder_positive_directions,
        clock=clock,
        sleep=sleep,
    )


def run_hardware_corner_probe(api: object, phase=None, *, clock=time.perf_counter, sleep=None):
    """Compatibility seam for the existing microprobe session owner.

    ``phase`` defaults to this module's cleanup-free motion phase.  The
    function is intentionally a thin delegation; the real owner remains
    ``y42_position_microprobe_io.run_hardware_corner_probe``.
    """
    if phase is None:
        phase = run_corner_probe_phase
    from y42_adapter_versions import load_microprobe_adapter
    owner, owner_digest = load_microprobe_adapter()
    _emit(api, "corner_stop_probe_transport_version", source_sha256=owner_digest)
    return owner.run_hardware_corner_probe(api, phase, clock=clock, sleep=sleep)


__all__ = [
    "CornerStopProbeError",
    "CURRENT_LEVELS_MA",
    "ROUND_MAX_SECONDS",
    "MAX_CUMULATIVE_TRAVEL_DEG",
    "MAX_FEEDBACK_AGE_SECONDS",
    "CB_STEP_CURRENT_LEVELS_MA",
    "CB_STEP_DEG",
    "CB_MAX_LEAD_DEG",
    "CB_MAX_TARGET_TRAVEL_DEG",
    "c5_frame",
    "run_corner_probe_phase",
    "run_corner_cb_step_phase",
    "run_hardware_corner_cb_step_probe",
    "run_hardware_corner_probe",
    "run_hardware_corner_stop_probe",
]
