"""Bounded motor-angle probe with injected IO only; never a Cartesian test.

The adapter owns enable/configuration, raw packet validation, independent stop,
and bounded IO. All sampled_at values must use the injected clock's time base.
Callbacks must return within their own deadlines: Python cannot interrupt a
blocked synchronous callback. This module never writes calibration or files.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ProbeConfig:
    amplitude_deg: float = 0.5
    axis_signs: tuple = (1, 1, 1, 1)
    move_seconds: float = 0.35
    settle_seconds: float = 0.15
    maximum_settle_seconds: float = 0.65
    current_ramp_seconds: float = 0.40
    entry_current_ma: int = 20
    probe_current_ma: int = 150
    final_current_ma: int = 20
    max_rpm: float = 1.0
    total_seconds: float = 8.0
    tick_seconds: float = 0.01
    feedback_max_age_seconds: float = 0.20
    feedback_max_span_seconds: float = 0.10
    stable_rpm: float = 0.30
    settle_error_deg: float = 0.25


class ProbeSafetyError(RuntimeError):
    def __init__(self, message, result):
        super().__init__(message)
        self.result = result


def run_position_probe(q0, *, snapshot, send_positions, clock, sleep,
                       emergency_stop, emit=lambda event: None, config=None):
    """Return verified angle response data, or emergency-stop and raise.

    One axis at a time moves q0 -> q0 +/- amplitude -> q0. There is no automatic
    retry or escalation. Normal completion is q0 held at final_current_ma with
    >= 0.15 s / >= 3 new complete feedback batches; the adapter then owns the
    requested stop/disable/config restoration. No result proves rope tension.
    """
    config = ProbeConfig() if config is None else config
    result = {"success": False, "kind": "encoder_angle_only", "xyz_precision_measured": False,
              "tension_verified": False, "axis_results": [], "commands": [],
              "feedback": [], "static_windows": [], "skipped_ticks": 0,
              "final_hold_verified": False, "emergency_requested": False}
    started = float(clock())
    deadline = started + min(8.0, float(config.total_seconds))
    last_targets = []
    last_rows = None
    error_since = [None] * 4

    def record(kind, **values):
        emit({"event": kind, "host_time": float(clock()), **values})

    def check_time():
        if float(clock()) >= deadline:
            raise RuntimeError("位置微幅验证总时限已到，停止且不重试")

    def read_feedback(stage):
        nonlocal last_rows
        check_time()
        rows = snapshot()
        check_time()
        now = float(clock())
        if not isinstance(rows, (list, tuple)) or len(rows) != 4:
            raise RuntimeError("四轴反馈不完整")
        indexed = {}
        for raw in rows:
            if not isinstance(raw, dict):
                raise RuntimeError("反馈行格式无效")
            address = raw.get("id")
            if isinstance(address, bool) or not isinstance(address, int) or address not in (1, 2, 3, 4) or address in indexed:
                raise RuntimeError("反馈ID重复或无效")
            row = {"id": address}
            for key in ("position", "rpm", "phase_current", "temperature", "sampled_at"):
                value = float(raw[key])
                if not math.isfinite(value):
                    raise RuntimeError(f"M{address} {key} 非有限反馈")
                row[key] = value
            flags = raw.get("flags")
            if isinstance(flags, bool) or not isinstance(flags, int) or flags < 0 or flags & ~0x3F:
                raise RuntimeError(f"M{address} 状态位无效")
            row["flags"] = flags
            if not flags & 0x01 or flags & 0x08:
                raise RuntimeError(f"M{address} 已失能或保护故障")
            if not -0.02 <= now - row["sampled_at"] <= config.feedback_max_age_seconds:
                raise RuntimeError(f"M{address} 反馈陈旧或时间基准无效")
            if abs(row["position"] - origin[address - 1]) > 4.0:
                raise RuntimeError(f"M{address} 偏离原位超过4度")
            if abs(row["rpm"]) > 3.0:
                raise RuntimeError(f"M{address} 原始转速超过3RPM")
            if abs(row["phase_current"]) > 350.0:
                raise RuntimeError(f"M{address} 电流超过350mA")
            if row["temperature"] >= 45.0:
                raise RuntimeError(f"M{address} 温度达到45度")
            indexed[address] = row
        ordered = [indexed[address] for address in (1, 2, 3, 4)]
        timestamps = [row["sampled_at"] for row in ordered]
        if max(timestamps) - min(timestamps) > config.feedback_max_span_seconds:
            raise RuntimeError("四轴反馈时间跨度过大")
        entry = {"host_time": now, "stage": stage, "rows": ordered,
                 "targets_deg": list(last_targets),
                 "errors_deg": [row["position"] - target for row, target in zip(ordered, last_targets)]}
        result["feedback"].append(entry)
        last_rows = ordered
        record("position_probe_feedback", **entry)
        for index, error in enumerate(entry["errors_deg"]):
            if abs(error) > 1.0:
                error_since[index] = now if error_since[index] is None else error_since[index]
                if now - error_since[index] >= 0.30 - 1e-9:
                    raise RuntimeError(f"M{index + 1} 跟随误差超过1度持续0.3秒")
            else:
                error_since[index] = None
        return ordered

    def send(targets, current, stage):
        nonlocal last_targets
        check_time()
        values = [float(value) for value in targets]
        if len(values) != 4 or any(not math.isfinite(value) or abs(value - zero) > 2.0 + 1e-9
                                   for value, zero in zip(values, origin)):
            raise RuntimeError("微幅目标超出2度硬界")
        current = int(round(current))
        if not 20 <= current <= 150:
            raise RuntimeError("微幅指令电流超出20..150mA")
        stamp = float(clock())
        send_positions(values, current, config.max_rpm)
        completed = float(clock())
        last_targets = values
        command = {"stage": stage, "scheduled_send_time": stamp, "completed_send_time": completed,
                   "targets_deg": values, "current_ma": current, "max_rpm": config.max_rpm}
        result["commands"].append(command)
        record("position_probe_command", **command)
        check_time()

    def next_tick(previous):
        next_time = previous + config.tick_seconds
        now = float(clock())
        if next_time < now - 1e-9:
            skipped = int(math.ceil((now - next_time) / config.tick_seconds))
            result["skipped_ticks"] += skipped
            next_time += skipped * config.tick_seconds
        return next_time

    def wait_tick(target):
        check_time()
        remaining = target - float(clock())
        if remaining > 0.0:
            sleep(min(remaining, max(0.0, deadline - float(clock()))))
        check_time()

    def eased_phase(from_targets, to_targets, from_current, to_current, duration, stage):
        phase_start = float(clock())
        due = max(phase_start, result["commands"][-1]["scheduled_send_time"] + config.tick_seconds) if result["commands"] else phase_start
        while True:
            wait_tick(due)
            fraction = min(1.0, max(0.0, (float(clock()) - phase_start) / duration))
            ease = fraction ** 3 * (10.0 + fraction * (-15.0 + 6.0 * fraction))
            targets = [left + (right - left) * ease for left, right in zip(from_targets, to_targets)]
            send(targets, from_current + (to_current - from_current) * ease, stage)
            read_feedback(stage)
            if fraction >= 1.0 - 1e-9:
                return
            due = next_tick(due)

    def settle(targets, current, stage, response_axis=None, before_position=None, sign=1):
        begin = float(clock())
        # Keep the command clock across phase boundaries. Starting a new
        # period after the preceding IO completes adds that IO latency twice.
        due = max(begin, result["commands"][-1]["scheduled_send_time"] + config.tick_seconds) if result["commands"] else begin
        window = []
        previous_signature = None
        last_response = None
        while True:
            wait_tick(due)
            send(targets, current, stage)
            rows = read_feedback(stage)
            now = float(clock())
            signature = tuple(row["sampled_at"] for row in rows)
            if response_axis is not None:
                last_response = sign * (rows[response_axis]["position"] - before_position)
            stable = all(abs(row["position"] - target) <= config.settle_error_deg
                         and abs(row["rpm"]) <= config.stable_rpm for row, target in zip(rows, targets))
            if not stable:
                window = []
            elif previous_signature is None or all(stamp > old for stamp, old in zip(signature, previous_signature)):
                window.append({"host_time": now, "rows": rows,
                               "errors_deg": [row["position"] - target for row, target in zip(rows, targets)]})
                previous_signature = signature
                if len(window) >= 3 and min(signature) - min(row["sampled_at"] for row in window[0]["rows"]) >= config.settle_seconds - 1e-9:
                    if last_response is not None and last_response <= 0.1000001:
                        raise RuntimeError(f"M{response_axis + 1} 响应不超过0.1度或方向相反，未证实跟随")
                    result["static_windows"].append({"stage": stage, "samples": window})
                    record("position_probe_static_window", stage=stage, samples=window)
                    return rows
            if now - begin >= config.maximum_settle_seconds:
                if last_response is not None and last_response <= 0.1000001:
                    raise RuntimeError(f"M{response_axis + 1} 响应不超过0.1度或方向相反，未证实跟随")
                raise RuntimeError(f"{stage} 未取得连续静止到位窗，停止且不加力")
            due = next_tick(due)

    try:
        origin = [float(value) for value in q0]
        if len(origin) != 4 or not all(math.isfinite(value) for value in origin):
            raise ValueError("q0必须是四个有限编码器角度")
        numeric = (config.amplitude_deg, config.move_seconds, config.settle_seconds,
                   config.maximum_settle_seconds, config.current_ramp_seconds, config.max_rpm,
                   config.total_seconds, config.tick_seconds, config.feedback_max_age_seconds,
                   config.feedback_max_span_seconds, config.stable_rpm, config.settle_error_deg)
        if not all(math.isfinite(float(value)) for value in numeric):
            raise ValueError("微幅配置必须有限")
        if not 0.0 < config.amplitude_deg <= 2.0 or len(config.axis_signs) != 4 or any(sign not in (-1, 1) for sign in config.axis_signs):
            raise ValueError("微幅幅度必须>0且<=2度，方向必须四个+1/-1")
        if (not 0.0 < config.max_rpm <= 1.0 or config.move_seconds < 0.35
                or config.current_ramp_seconds < 0.4 or config.settle_seconds < 0.15
                or not config.settle_seconds <= config.maximum_settle_seconds <= 0.65
                or not 0.0 < config.total_seconds <= 8.0 or config.tick_seconds not in (0.01, 0.02, 0.04)
                or not 0.0 < config.feedback_max_age_seconds <= 0.20
                or not 0.0 < config.feedback_max_span_seconds <= 0.10
                or not 0.0 <= config.stable_rpm <= 0.30 or not 0.0 < config.settle_error_deg <= 0.25
                or config.entry_current_ma not in (20, 150) or config.probe_current_ma != 150 or config.final_current_ma != 20):
            raise ValueError("微幅配置不能放宽安全边界")
        last_targets = list(origin)
        result["q0_deg"] = list(origin)
        result["started_at"] = started
        initial_rows = read_feedback("entry")
        if any(abs(row["position"] - zero) > config.settle_error_deg or abs(row["rpm"]) > config.stable_rpm
               for row, zero in zip(initial_rows, origin)):
            raise RuntimeError("入口尚未在q0静止，拒绝接管")
        eased_phase(origin, origin, config.entry_current_ma, 150, config.current_ramp_seconds, "soft_current_start")
        move_duration = max(config.move_seconds, 1.875 * config.amplitude_deg / (6.0 * config.max_rpm))
        for axis, sign in enumerate(config.axis_signs):
            before = last_rows[axis]["position"]
            target = list(origin)
            target[axis] += sign * config.amplitude_deg
            eased_phase(origin, target, 150, 150, move_duration, f"M{axis + 1}_out")
            peak_rows = settle(target, 150, f"M{axis + 1}_peak", axis, before, sign)
            response = sign * (peak_rows[axis]["position"] - before)
            eased_phase(target, origin, 150, 150, move_duration, f"M{axis + 1}_return")
            return_rows = settle(origin, 150, f"M{axis + 1}_returned")
            result["axis_results"].append({"id": axis + 1, "sign": sign,
                "commanded_excursion_deg": sign * config.amplitude_deg,
                "observed_signed_response_deg": response,
                "peak_error_deg": peak_rows[axis]["position"] - target[axis],
                "return_error_deg": return_rows[axis]["position"] - origin[axis],
                "peak_rows": peak_rows, "return_rows": return_rows})
        eased_phase(origin, origin, 150, 20, config.current_ramp_seconds, "soft_current_finish")
        final_rows = settle(origin, 20, "final_20ma_hold")
        check_time()
        result.update(success=True, final_hold_verified=True, final_rows=final_rows,
                      final_current_ma=20, elapsed_seconds=float(clock()) - started)
        record("position_probe_complete", axis_results=result["axis_results"], final_rows=final_rows,
               elapsed_seconds=result["elapsed_seconds"], final_hold_verified=True,
               xyz_precision_measured=False, tension_verified=False)
        return result
    except Exception as error:
        result.update(success=False, error=str(error), elapsed_seconds=float(clock()) - started,
                      emergency_requested=True)
        # Emergency is attempted before any further logging or state changes.
        try:
            result["emergency_result"] = emergency_stop(str(error))
        except Exception as emergency_error:
            result["emergency_error"] = str(emergency_error)
        try:
            record("position_probe_failed", error=str(error), elapsed_seconds=result["elapsed_seconds"],
                   emergency_requested=True, final_hold_verified=False)
        except Exception:
            pass
        raise ProbeSafetyError(str(error), result) from error
