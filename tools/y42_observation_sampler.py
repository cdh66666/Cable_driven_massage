"""Bounded, injected-IO torque observations. Never opens a port or calibrates.

The service owns its serial mutex, command direction mapping, manual-run
recorder and emergency-stop implementation. It must establish a known torque
hold before entering this module. Motor phase current is not a tension sensor.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class ObservationConfig:
    total_budget_s: float = 15.0
    baseline_current_ma: int = 150
    max_command_current_ma: int = 400
    max_observed_current_ma: float = 400.0
    command_max_rpm: float = 8.0
    observed_rpm_trip: float = 25.0
    max_temperature_c: float = 45.0
    max_excursion_deg: float = 90.0
    max_cumulative_travel_deg: float = 180.0
    stable_window_s: float = 0.4
    stable_rpm: float = 0.6
    stable_position_span_deg: float = 0.8
    maximum_dwell_s: float = 1.5
    command_interval_s: float = 0.05
    feedback_interval_s: float = 0.2
    maximum_feedback_age_s: float = 0.8
    maximum_batch_span_s: float = 0.8
    io_timeout_s: float = 0.65
    minimum_ramp_s: float = 0.4
    current_slew_ma_s: float = 400.0
    finish_reserve_s: float = 3.5
    max_vectors: int = 6


class ObservationSafetyError(RuntimeError):
    pass


def _finite(value, label):
    result = float(value)
    if not math.isfinite(result):
        raise ObservationSafetyError(f"{label}不是有限数")
    return result


def _field(value, name):
    return value.get(name) if isinstance(value, dict) else getattr(value, name, None)


def plan_observation_vectors(vectors, config=ObservationConfig()):
    """Validate requested current vectors; they are stimuli, not known poses."""
    for name, value in asdict(config).items():
        if not math.isfinite(float(value)) or float(value) <= 0:
            raise ValueError(f"{name}必须有限且为正")
    if not 150 <= config.baseline_current_ma <= config.max_command_current_ma <= 400:
        raise ValueError("本观察模式仅允许150..400mA")
    if config.command_max_rpm > 8 or config.observed_rpm_trip < config.command_max_rpm:
        raise ValueError("观察命令不得超过8RPM，速度保护不得低于命令上限")
    if config.total_budget_s <= config.finish_reserve_s + config.stable_window_s:
        raise ValueError("总预算不足以保留柔性结束和静止采样")
    if config.maximum_dwell_s < config.stable_window_s:
        raise ValueError("静止窗口超过驻留预算")
    parsed = []
    for vector in vectors:
        row = tuple(int(value) for value in vector)
        if len(row) != 4 or any(float(value) != integer for value, integer in zip(vector, row)):
            raise ValueError("每个观察向量必须有四个整数电流")
        if any(not config.baseline_current_ma <= value <= config.max_command_current_ma for value in row):
            raise ValueError("观察向量超出150..400mA边界")
        parsed.append(row)
    if not parsed or len(parsed) > config.max_vectors:
        raise ValueError(f"本次需要1..{config.max_vectors}个观察向量")
    return tuple(parsed)


def run_observation_sampler(
    vectors, io, clock, stop_requested, *, initial_torque_currents_ma,
    config=ObservationConfig(),
):
    """Observe with an existing, exclusively owned service IO adapter.

    io.send_torque(tuple, max_rpm=..., deadline=...) -> None/True (False fails)
    io.read_snapshot(deadline=...) -> {captured_start, captured_end, motors}
      motors: four Telemetry objects or dicts with address/position/rpm/
      phase_current/temperature/flags; preserve the actual batch timestamps.
    io.emit(event, **fields); io.emergency_stop(reason)
    clock.now(); clock.wait_until(absolute_monotonic_time)

    Initial currents MUST be known previously submitted torque commands, not
    phase-current readings or guesses. Normal completion returns to 150mA and
    verifies a repeated static feedback window; this does not prove rope tension.
    Safety faults and explicit STOP request emergency stopping; they do not
    continue a comfort ramp. Every result is observation-only, never calibration.
    Deadlines require the service adapter to bound its own reads/writes.
    """
    planned = plan_observation_vectors(vectors, config)
    baseline = (config.baseline_current_ma,) * 4
    if initial_torque_currents_ma is None or tuple(initial_torque_currents_ma) != baseline:
        raise ValueError("采样前必须由服务明确建立已知150mA恒力矩保持，不能从相电流推测模式")
    started = float(clock.now())
    deadline = started + config.total_budget_s
    command = baseline
    last_command_at = started - config.command_interval_s
    first_positions = previous_positions = None
    cumulative = [0.0] * 4
    snapshots, samples, raw_batches = [], [], []
    last_batch_end = None
    next_feedback_at = started
    result = {"calibration_published": False, "absolute_pose_verified": False,
              "independent_pose_count": None, "requested_vector_count": len(planned),
              "samples": samples, "snapshots": snapshots, "raw_batches": raw_batches, "status": "running",
              "final_hold_verified": False, "final_hold_window": [], "final_hold_raw_batches": []}

    def guard():
        if stop_requested():
            raise ObservationSafetyError("用户停止观察")
        if float(clock.now()) >= deadline:
            raise ObservationSafetyError("观察总预算到期")

    def read_checked(phase, local_deadline=None):
        nonlocal first_positions, previous_positions, last_batch_end, next_feedback_at
        guard()
        call_deadline = min(deadline, float(clock.now()) + config.io_timeout_s)
        if local_deadline is not None:
            if float(clock.now()) >= local_deadline:
                raise ObservationSafetyError("回150mA后未取得连续静止保持窗口")
            call_deadline = min(call_deadline, local_deadline)
        batch = io.read_snapshot(deadline=call_deadline)
        raw_batch = {
            "captured_start": batch.get("captured_start"), "captured_end": batch.get("captured_end"),
            "motors": [{name: _field(item, name) for name in ("address", "position", "rpm", "phase_current", "temperature", "flags")}
                       for item in batch.get("motors", [])], "phase": phase,
        }
        raw_batches.append(raw_batch)
        if phase == "final_hold_verify":
            result["final_hold_raw_batches"].append(raw_batch)
        io.emit("torque_observation_raw_batch", **raw_batch)
        guard()
        now = float(clock.now())
        if now > call_deadline + 1e-9:
            raise ObservationSafetyError("观察回读超过传入deadline")
        begin = _finite(batch["captured_start"], "回读开始时间")
        end = _finite(batch["captured_end"], "回读结束时间")
        motors = list(batch["motors"])
        if len(motors) != 4 or [_field(item, "address") for item in motors] != [1, 2, 3, 4]:
            raise ObservationSafetyError("四台反馈缺失或ID顺序错误")
        if begin > end or end > now + 1e-6 or now - begin > config.maximum_feedback_age_s:
            raise ObservationSafetyError("反馈过期或时间戳无效")
        if end - begin > config.maximum_batch_span_s or (last_batch_end is not None and (begin < last_batch_end - 1e-6 or end <= last_batch_end + 1e-9)):
            raise ObservationSafetyError("反馈批次滞后/重复，不作为同步静止样本")
        rows = []
        for item in motors:
            address = int(_field(item, "address"))
            row = {name: _finite(_field(item, name), f"M{address} {name}")
                   for name in ("position", "rpm", "phase_current", "temperature", "flags")}
            if int(row["flags"]) != row["flags"] or not int(row["flags"]) & 1 or int(row["flags"]) & 0x08:
                raise ObservationSafetyError(f"M{address}失能/驱动保护")
            if abs(row["rpm"]) > config.observed_rpm_trip:
                raise ObservationSafetyError(f"M{address}速度超过{config.observed_rpm_trip:g}RPM观察保护")
            if abs(row["phase_current"]) > config.max_observed_current_ma:
                raise ObservationSafetyError(f"M{address}电流超过{config.max_observed_current_ma:g}mA观察保护")
            if row["temperature"] >= config.max_temperature_c:
                raise ObservationSafetyError(f"M{address}温度达到观察保护")
            row["address"] = address
            rows.append(row)
        positions = [row["position"] for row in rows]
        if first_positions is None:
            first_positions = positions[:]
            previous_positions = positions[:]
        for axis in range(4):
            cumulative[axis] += abs(positions[axis] - previous_positions[axis])
            if abs(positions[axis] - first_positions[axis]) > config.max_excursion_deg:
                raise ObservationSafetyError(f"M{axis + 1}相对入口位移超过{config.max_excursion_deg:g}°")
            if cumulative[axis] > config.max_cumulative_travel_deg:
                raise ObservationSafetyError(f"M{axis + 1}累计角行程超过{config.max_cumulative_travel_deg:g}°")
        previous_positions = positions
        last_batch_end = end
        next_feedback_at = now + config.feedback_interval_s
        observation = {"captured_start": begin, "captured_end": end, "motors": rows,
                       "command_currents_ma": list(command), "phase": phase,
                       "cumulative_travel_deg": cumulative[:]}
        snapshots.append(observation)
        io.emit("torque_observation_feedback", **observation)
        return observation

    def advance_static_window(window, observation):
        if any(abs(row["rpm"]) > config.stable_rpm for row in observation["motors"]):
            return []
        window = window + [observation]
        if any(max(s["motors"][axis]["position"] for s in window) - min(s["motors"][axis]["position"] for s in window) > config.stable_position_span_deg for axis in range(4)):
            return [observation]
        return window

    def static_window_ready(window):
        # Batch duration itself is not simultaneous stability. read_checked has
        # already rejected missing, overlapping or reused feedback batches.
        return len(window) >= 3 and window[-1]["captured_start"] - window[0]["captured_end"] >= config.stable_window_s

    def ramp(target, phase):
        nonlocal command, last_command_at
        origin = command
        delta = max(abs(b - a) for a, b in zip(origin, target))
        duration = max(config.minimum_ramp_s, 1.875 * delta / config.current_slew_ma_s)
        steps = max(1, math.ceil(duration / config.command_interval_s))
        for step in range(1, steps + 1):
            guard()
            wait_until = max(float(clock.now()), last_command_at + config.command_interval_s)
            if wait_until >= deadline:
                raise ObservationSafetyError("柔性包络将超过总预算")
            clock.wait_until(wait_until)
            guard()
            if last_batch_end is None or float(clock.now()) - last_batch_end > config.maximum_feedback_age_s:
                read_checked(phase)
            fraction = step / steps
            eased = fraction ** 3 * (10.0 - 15.0 * fraction + 6.0 * fraction ** 2)
            command = tuple(round(a + (b - a) * eased) for a, b in zip(origin, target))
            call_deadline = min(deadline, float(clock.now()) + config.io_timeout_s)
            guard()
            sent = io.send_torque(command, max_rpm=config.command_max_rpm, deadline=call_deadline)
            if sent is False:
                raise ObservationSafetyError("恒力矩合帧提交失败")
            guard()
            if float(clock.now()) > call_deadline + 1e-9:
                raise ObservationSafetyError("恒力矩写入超过传入deadline")
            last_command_at = float(clock.now())
            io.emit("torque_observation_command", currents_ma=list(command), max_rpm=config.command_max_rpm, phase=phase)
            if float(clock.now()) >= next_feedback_at:
                read_checked(phase)

    try:
        io.emit("torque_observation_begin", config=asdict(config), vectors=[list(v) for v in planned], observation_only=True)
        read_checked("entry")
        finish_reason = "all_requested_vectors_observed"
        for index, vector in enumerate(planned):
            required_ramp = max(config.minimum_ramp_s, 1.875 * max(abs(b - a) for a, b in zip(command, vector)) / config.current_slew_ma_s)
            if float(clock.now()) + required_ramp + config.maximum_dwell_s + config.finish_reserve_s >= deadline:
                finish_reason = "budget_reserved_for_gentle_hold"
                break
            ramp(vector, f"vector_{index + 1}_entry")
            stationary = []
            dwell_started = float(clock.now())
            accepted = False
            while float(clock.now()) - dwell_started < config.maximum_dwell_s:
                if float(clock.now()) + config.finish_reserve_s >= deadline:
                    break
                observation = read_checked(f"vector_{index + 1}_settle")
                stationary = advance_static_window(stationary, observation)
                if static_window_ready(stationary):
                    sample = {"vector_index": index, "currents_ma": list(vector), "window": stationary[:],
                              "positions_deg": [sum(s["motors"][axis]["position"] for s in stationary) / len(stationary) for axis in range(4)],
                              "pose_known": False, "calibration_sample_accepted": False}
                    samples.append(sample)
                    io.emit("torque_observation_static_window", **sample)
                    accepted = True
                    break
                wake = min(float(clock.now()) + config.feedback_interval_s, deadline - config.finish_reserve_s)
                if wake <= float(clock.now()):
                    break
                clock.wait_until(wake)
            if not accepted:
                finish_reason = "static_window_not_obtained"
                break
        ramp(baseline, "normal_return_to_hold")
        hold_deadline = min(deadline, float(clock.now()) + config.maximum_dwell_s)
        io.emit("torque_observation_final_hold_begin", deadline=hold_deadline, currents_ma=list(command))
        while float(clock.now()) < hold_deadline:
            observation = read_checked("final_hold_verify", local_deadline=hold_deadline)
            window = advance_static_window(result["final_hold_window"], observation)
            result["final_hold_window"] = window
            verified = static_window_ready(window)
            io.emit("torque_observation_final_hold_window", window=window[:], verified=verified,
                    raw_batch_count=len(result["final_hold_raw_batches"]), deadline=hold_deadline)
            guard()
            if float(clock.now()) > hold_deadline + 1e-9:
                raise ObservationSafetyError("静止保持核验超过驻留deadline")
            if verified:
                result["final_hold_verified"] = True
                break
            clock.wait_until(min(float(clock.now()) + config.feedback_interval_s, hold_deadline))
            guard()
        if not result["final_hold_verified"]:
            raise ObservationSafetyError("回150mA后未取得连续静止保持窗口")
        result.update(status="observed" if len(samples) == len(planned) else "incomplete", reason=finish_reason,
                      final_command_currents_ma=list(command), emergency_requested=False)
    except Exception as error:
        reason = f"{type(error).__name__}: {error}"
        result.update(status="safety_stopped", reason=reason, emergency_requested=True, final_hold_verified=False)
        try:
            result["emergency_result"] = io.emergency_stop(reason)
        except Exception as stop_error:
            result["emergency_error"] = str(stop_error)
        try:
            io.emit("torque_observation_safety_stop", reason=reason)
        except Exception:
            pass
    result["elapsed_s"] = float(clock.now()) - started
    result["cumulative_travel_deg"] = cumulative[:]
    return result
