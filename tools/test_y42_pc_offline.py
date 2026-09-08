#!/usr/bin/env python3
"""离线验收：不打开串口、不使能电机，只验证几何和协议代码。

真实运动验收必须在设备上电且 COM7 可打开后另行完成。本脚本的目的，是
在改动代码后先把尺寸、反解、图形端点和四电机合帧这些纯代码问题挡住。
"""

from __future__ import annotations

import atexit
import copy
from functools import wraps
import json
import math
import sys
import tempfile
import time
from pathlib import Path
from unittest.mock import patch


TOOLS = Path(__file__).resolve().parent
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

# Importing the real server reads/migrates its adjacent calibration file.  Even
# a direct CLI invocation of this test must never inspect or rewrite that file.
from test_y42_calibration_flow import isolated_web_module  # noqa: E402

_TEST_STORAGE = tempfile.TemporaryDirectory(prefix="y42-offline-suite-")
atexit.register(_TEST_STORAGE.cleanup)
web = isolated_web_module(_TEST_STORAGE.name)


def _isolated_test_case(function):
    """Keep every individual test independent, including direct function calls."""
    @wraps(function)
    def run():
        original_state = web.state_snapshot()
        original_paths = {
            name: getattr(web, name)
            for name in ("CALIBRATION_FILE", "PROCESS_LOG_FILE", "MANUAL_TEST_RUNS_ROOT", "append_process_log")
        }
        original_stop = web.STOP_EVENT.is_set()
        try:
            with patch.object(web.direct, "open_serial", side_effect=AssertionError("Real serial is forbidden in offline tests")), \
                 patch.object(web.direct, "_TRAFFIC_CALLBACK", web.direct._TRAFFIC_CALLBACK):
                return function()
        finally:
            try:
                # Recorder writes are asynchronous; drain them before restoring
                # STATE, otherwise the next test could inherit a late update.
                deadline = time.monotonic() + 3.0
                while web.MANUAL_TEST_QUEUE.unfinished_tasks and time.monotonic() < deadline:
                    time.sleep(0.01)
                if web.MANUAL_TEST_QUEUE.unfinished_tasks:
                    raise AssertionError("Offline recorder did not finish before state restoration")
            finally:
                for name, value in original_paths.items():
                    setattr(web, name, value)
                with web.STATE_LOCK:
                    web.STATE.clear()
                    web.STATE.update(original_state)
                web.STOP_EVENT.set() if original_stop else web.STOP_EVENT.clear()
    return run


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def close(actual: float, expected: float, tolerance: float, message: str) -> None:
    check(abs(actual - expected) <= tolerance, f"{message}: {actual} != {expected}")


def test_workspace_and_solver() -> None:
    check(web.TRAJECTORY_HZ == 100.0, "轨迹控制未使用当前100Hz目标")
    check(abs(web.TRAJECTORY_INTERVAL - 0.01) < 1e-12, "轨迹周期与100Hz目标不一致")
    check(web.TIMED_POSITION_INTERVAL_MS == 10, "四电机反馈周期没有设置为10ms/100Hz")
    check(web.TIMED_POSITION_HZ == 100.0, "四电机反馈目标没有设置为100Hz")
    check(web.timed_feedback_interval_ms(921600) == 10, "921600波特率没有选用10ms反馈周期")
    check(web.timed_feedback_interval_ms(512000) == 10, "512000波特率没有安全回退到10ms反馈周期")
    check(web.motion_feedback_interval_ms(921600) == 10, "高波特率带载运动没有使用10ms反馈周期")
    check(web.motion_feedback_interval_ms(115200) == 97, "115200带载运动没有使用97ms去谐反馈周期")
    check(web.TRACKING_HZ == 100.0, "张力跟踪解算频率没有设置为100Hz")
    check(web.PATH_SPEED_DEFAULT_MM_S == 50.0, "手绘默认速度没有恢复到 50mm/s")
    check(web.PATH_SPEED_MIN_MM_S == 10.0, "手绘速度下限没有允许 10mm/s")
    check(web.TRACKING_CURRENT_LIMIT_DEFAULT_MA == 2000, "张力跟踪保护上限默认值不是 2A")
    check(web.TRAJECTORY_DEFAULT_CURRENT == 2000, "四位置轨迹保护上限默认值不是 2A")
    check(web.CONTROL_ARCHITECTURE == "four_position_force_primary_pure_ab", "生产控制架构不是四位置微预紧主控制")
    check(web.TRACKING_AUTO_FLOOR_CURRENT_MA == 300, "张力跟踪没有从柔和自动基准建立")
    check(web.TRAJECTORY_START_RPM == 40.0, "轨迹起点速度仍低于 50mm/s 量级")
    check(web.CUSTOM_PATH_TRAVEL_SPEED_MM_S >= 150.0, "笔间换位速度仍过慢")
    check(web.CUSTOM_PATH_STROKE_END_HOLD_SECONDS <= 0.18, "笔间结束等待仍过长")
    check('id="floatingCalibration"' in web.PAGE, "一键标定没有移入顶部悬浮条")
    check("body:not(.advanced-ui) #calibrationSection" in web.PAGE, "普通界面仍显示大块标定卡")
    check("['trajectory','tracking'].includes(latestAuto.auto)?50" in web.PAGE, "轨迹显示没有保持约 20fps")
    check('id="trackingCurrent" type="number" min="50" max="2000" step="1" value="2000"' in web.PAGE, "网页张力保护上限不是 2A")
    four_checkpoint = {"tension_checkpoint_controller": web.TENSION_CHECKPOINT_CONTROLLER_FOUR_TORQUE}
    ok, _ = web.persisted_checkpoint_tension_evidence(four_checkpoint, [150, 151, 149, 152])
    check(ok, "四力矩张力检查点被误拒绝")
    ok, _ = web.persisted_checkpoint_tension_evidence(four_checkpoint, [150, 46, 149, 152])
    check(not ok, "四力矩检查点允许低电流松绳轴热恢复")
    hybrid_checkpoint = {
        "tension_checkpoint_controller": web.TENSION_CHECKPOINT_CONTROLLER_THREE_PLUS_ONE,
        "tension_checkpoint_passive_motor": 4,
        "tension_checkpoint_currents_ma": [350, 350, 350, 149],
    }
    ok, _ = web.persisted_checkpoint_tension_evidence(hybrid_checkpoint, [46, 55, 60, 149])
    check(ok, "3+1位置轴低电流被误判为松绳")
    ok, _ = web.persisted_checkpoint_tension_evidence(hybrid_checkpoint, [20, 18, 16, 76])
    check(ok, "3+1瞬时相电流约为保存带载值一半时被误拒绝")
    ok, _ = web.persisted_checkpoint_tension_evidence(hybrid_checkpoint, [150, 150, 150, 46])
    check(not ok, "3+1恒张力轴低电流仍被允许热恢复")
    force_position_checkpoint = {
        "tension_checkpoint_controller": web.TENSION_CHECKPOINT_CONTROLLER_FOUR_POSITION_FORCE,
        "tension_checkpoint_currents_ma": [420, 510, 390, 470],
    }
    ok, _ = web.persisted_checkpoint_tension_evidence(
        force_position_checkpoint,
        [160, 180, 145, 170],
    )
    check(ok, "四位置力位检查点的四绳带载证据被误拒绝")
    ok, _ = web.persisted_checkpoint_tension_evidence(
        force_position_checkpoint,
        [160, 180, 40, 170],
    )
    check(not ok, "四位置力位检查点允许一根低电流绳热恢复")
    calibration = {
        "workspace_width": 110.0,
        "workspace_height": 148.0,
        "slope": [-4.0, 4.1, -4.05, 3.95],
        "intercept": [100.0, -200.0, 300.0, -400.0],
        "slope_trim_percent": [0.0] * 4,
        "samples": [],
    }
    motor_x, motor_y = web.motor_center_coordinates(calibration)
    check((motor_x, motor_y) == ((-44.0, 154.0, 154.0, -44.0), (148.0, 148.0, 0.0, 0.0)), "STEP 电机中心框不是 198×148")
    exit_x, exit_y = web.cable_exit_coordinates(calibration)
    check((exit_x, exit_y) == ((-31.5, 141.5, 141.5, -31.5), (148.0, 148.0, 0.0, 0.0)), "12.5mm 卷筒参考点坐标错误")
    attachment_x, attachment_y = web.ball_attachment_offsets(calibration)
    expected_attachment_x = (-15.267, 15.267, 15.267, -15.267)
    expected_attachment_y = (11.4115, 11.4115, -11.4115, -11.4115)
    check((attachment_x, attachment_y) == (expected_attachment_x, expected_attachment_y), "球端四连接点没有使用 30.534×22.823mm CAD 尺寸")
    ax, ay = web.workspace_anchor_coordinates(calibration)
    expected_anchor_x = (-16.233, 126.233, 126.233, -16.233)
    expected_anchor_y = (136.5885, 136.5885, 11.4115, 11.4115)
    for actual, expected in zip(ax + ay, expected_anchor_x + expected_anchor_y):
        close(actual, expected, 1e-9, "等效球心锚点")
    check(web.workspace_dimensions(calibration) == (110.0, 148.0), "用户工作区不是 110×148")
    check(web.workspace_dimensions({"effective_side": 150.0}) == (110.0, 148.0), "旧方形回退污染了 PC 工作区")
    check(web.workspace_dimensions({"workspace_width": 150.0, "workspace_height": 150.0}) == (110.0, 148.0), "任意旧工作区尺寸仍可进入 PC 解算")
    check(web.workspace_dimensions({"workspace_width": 90.0, "workspace_height": 80.0}) == (110.0, 148.0), "非目标工作区尺寸未被固定约束")
    check("function drawableGeometry(c){return{x:0,y:0,w:110,h:148}}" in web.PAGE, "绘图画布仍从旧尺寸回退")
    check("function trackingWorkspace(c){return{x:0,y:0,w:110,h:148}}" in web.PAGE, "跟踪画布仍从旧尺寸回退")
    check("gridTargetCurrent" in web.PAGE and "startGridCalibration" in web.PAGE, "网页没有多点网格张力校准入口")
    check("action:'grid-calibration'" in web.PAGE, "网页没有提交网格校准动作")
    check(web.GRID_POSITION_TOLERANCE_DEG == 1.0, "网格到位门槛没有收紧到 1°")
    check(web.CENTER_TENSION_SWEEP_LEVELS_MA[0] == 50, "中心等力矩扫描没有从 50mA 安全档开始")
    check("中心等力矩测试（已停用标定写入）" in web.PAGE, "网页没有明确停用等电流中心写入")
    check("中心等电流仅作诊断，使用四角编码器模型" in web.PAGE, "网页未明确中心等电流仅作诊断")
    check('id="pathSpeed" type="number" min="10" max="5000" value="50"' in web.PAGE, "网页手绘速度默认值不是 50mm/s")
    check(web.GEOMETRY_VERSION == 8, "球心四角几何没有升级到 v8")
    check(web.BALL_CENTER_CONTACT_Z_MM == 25.0, "STEP 球心接触高度不是 25mm")
    check((web.SPOOL_WINDING_Z_MIN_MM, web.SPOOL_WINDING_Z_MAX_MM) == (5.0, 16.0), "STEP 卷筒绕线高度范围错误")
    center_uncertainty = web.spool_height_length_uncertainty_mm(55.0, 74.0, calibration=calibration)
    check(all(1.5 < value < 1.54 for value in center_uncertainty), "中心点卷筒高度不确定度计算错误")
    corner_uncertainty = web.spool_height_length_uncertainty_mm(0.0, 148.0, calibration=calibration)
    check(max(corner_uncertainty) > 4.8, "近卷筒角点没有暴露至少 4.8mm 的高度歧义")

    x, y, z = 37.0, 82.0, 0.0
    lengths = [math.hypot(x - anchor_x, y - anchor_y) for anchor_x, anchor_y in zip(ax, ay)]
    solved = web.solve_position_from_lengths(lengths, calibration)
    check(solved is not None, "平面点反解失败")
    close(solved[0], x, 1e-9, "平面 X 反解误差")
    close(solved[1], y, 1e-9, "平面 Y 反解误差")
    close(solved[2], z, 1e-9, "平面 Z 反解误差")

    # Tracking must use changes from a post-tension anchor.  Fixed per-rope
    # offsets model smooth-drum wrap/slack ambiguity and must cancel exactly.
    anchor_xy = (80.0, 100.0)
    moved_xy = (36.0, 67.0)
    fixed_bias = [3.4, -2.1, 5.2, -4.3]
    anchor_lengths = web.planar_platform_cable_lengths(*anchor_xy, 0.0, calibration)
    moved_lengths = web.planar_platform_cable_lengths(*moved_xy, 0.0, calibration)
    biased_baseline = [value + bias for value, bias in zip(anchor_lengths, fixed_bias)]
    biased_moved = [value + bias for value, bias in zip(moved_lengths, fixed_bias)]
    relative_pose = web.solve_relative_tracking_xy(
        biased_moved,
        biased_baseline,
        anchor_xy,
        calibration,
    )
    check(relative_pose is not None, "增量张力跟踪反解失败")
    close(relative_pose["x_mm"], moved_xy[0], 1e-5, "增量跟踪 X 未抵消固定卷绕偏差")
    close(relative_pose["y_mm"], moved_xy[1], 1e-5, "增量跟踪 Y 未抵消固定卷绕偏差")
    biased_moved[3] += 2.0
    robust_pose = web.solve_relative_tracking_xy(
        biased_moved,
        biased_baseline,
        anchor_xy,
        calibration,
    )
    check(robust_pose is not None, "单绳叠层扰动后增量反解失败")
    check(math.hypot(robust_pose["x_mm"] - moved_xy[0], robust_pose["y_mm"] - moved_xy[1]) < 1.5, "鲁棒增量反解被单绳叠层拖偏")

    # The real centre part is a rigid four-attachment platform.  Its board-plane
    # pose includes yaw; four lengths leave one independent closure residual.
    yaw = 17.0
    rigid_lengths = web.planar_platform_cable_lengths(x, y, yaw, calibration)
    rigid_pose = web.solve_planar_platform_pose(rigid_lengths, calibration)
    check(rigid_pose is not None, "平面刚体位姿反解失败")
    close(rigid_pose["x_mm"], x, 1e-6, "刚体 X 反解误差")
    close(rigid_pose["y_mm"], y, 1e-6, "刚体 Y 反解误差")
    close(rigid_pose["yaw_deg"], yaw, 1e-6, "刚体 yaw 反解误差")
    check(rigid_pose["closure_max_mm"] < 1e-6, "无噪声刚体闭合残差不为零")

    # Three position-controlled cables determine x/y/yaw.  Perturbing only the
    # omitted fourth cable must not move that pose; it must appear as an
    # independent redundant residual instead.
    redundant_lengths = list(rigid_lengths)
    redundant_lengths[3] += 2.0
    three_cable_pose = web.solve_planar_platform_pose(
        redundant_lengths,
        calibration,
        active_indices=(0, 1, 2),
        initial_pose=(x, y, yaw),
    )
    check(three_cable_pose is not None, "三位置绳位姿反解失败")
    close(three_cable_pose["x_mm"], x, 1e-6, "三绳 X 被冗余绳污染")
    close(three_cable_pose["y_mm"], y, 1e-6, "三绳 Y 被冗余绳污染")
    close(three_cable_pose["yaw_deg"], yaw, 1e-6, "三绳 yaw 被冗余绳污染")
    close(three_cable_pose["redundant_residuals_mm"][0], -2.0, 1e-6, "第四绳独立残差错误")

    pair_choice = web.select_position_pair_for_path([(0.0, 0.0), (0.0, 148.0), (110.0, 148.0), (110.0, 0.0)], calibration)
    check(pair_choice["ok"] is True and len(pair_choice["position_indices"]) == 2, "2+2 路径没有选出可观位置对")
    two_cable_pose = web.solve_fixed_yaw_xy_from_pair(
        web.planar_platform_cable_lengths(x, y, 0.0, calibration),
        pair_choice["position_indices"],
        calibration,
        initial_xy=(x, y),
    )
    check(two_cable_pose is not None, "两位置绳 XY 反解失败")
    close(two_cable_pose["x_mm"], x, 1e-6, "2+2 X 反解误差")
    close(two_cable_pose["y_mm"], y, 1e-6, "2+2 Y 反解误差")
    check(two_cable_pose["closure_max_mm"] < 1e-6, "2+2 两根独立闭合残差不为零")

    z = 23.0
    lengths = [math.sqrt((x - anchor_x) ** 2 + (y - anchor_y) ** 2 + z * z) for anchor_x, anchor_y in zip(ax, ay)]
    solved = web.solve_position_from_lengths(lengths, calibration)
    check(solved is not None, "抬高点反解失败")
    close(solved[0], x, 1e-9, "抬高点 X 反解误差")
    close(solved[1], y, 1e-9, "抬高点 Y 反解误差")
    close(solved[2], z, 1e-9, "抬高点 Z 反解误差")

    impossible_lengths = [max(1.0, value - 25.0) for value in lengths]
    impossible_solved = web.solve_position_from_lengths(impossible_lengths, calibration)
    check(impossible_solved is not None, "不可能绳长应保留 XY 供诊断")
    impossible_check = web.absolute_pose_consistency(
        impossible_lengths, impossible_solved[0], impossible_solved[1], calibration
    )
    check(
        impossible_check["physically_possible"] is False
        and impossible_check["median_z_squared_mm2"] < 0.0,
        "负 z² 被钳成零后仍错误地通过物理一致性检查",
    )

    center_model = web.equilibrium_tension_model(55.0, 74.0, calibration)
    check(center_model["ok"] is True, "四连接点中心力矩平衡不可解")
    check(abs(center_model["resultant_moment_mm"]) < 1e-9, "中心张力存在残余力矩")


def test_relative_height_requires_nonzero_baseline_reference() -> None:
    """Correct the h0 != 0 inversion and reject an unknown physical h0."""
    # The planar correction leaves a square-distance delta.  For h0=14.5mm
    # and a true 1mm lift, delta is 2*h0*lift + lift² = 30mm².  The old
    # sqrt(delta) shortcut reports 5.477mm instead of 1mm.
    delta = 2.0 * 14.5 * 1.0 + 1.0
    unknown = web.relative_height_from_z_squared(
        [delta] * 4,
        [0.0] * 4,
    )
    check(unknown["valid"] is True, "未知 h0 的 z² 诊断不应被当作无效样本")
    check(unknown["height_mm"] is None, "未知 h0 不应伪造毫米高度")
    check(
        unknown["height_available"] is False
        and unknown["height_reason"] == "baseline_vertical_distance_unknown",
        "未知 h0 没有明确标记为高度不可用",
    )
    check(
        unknown["confidence"] == 0.0 and unknown["z_squared_confidence"] > 0.0,
        "未知 h0 仍把 z² 拟合置信度冒充为高度置信度",
    )

    corrected = web.relative_height_from_z_squared(
        [delta] * 4,
        [0.0] * 4,
        baseline_vertical_distance_mm=14.5,
    )
    check(corrected["height_available"] is True, "提供 h0 后高度没有恢复可用")
    close(corrected["height_mm"], 1.0, 1e-9, "非零 h0 高度反解")
    check(
        abs(corrected["height_mm"] - math.sqrt(delta)) > 4.0,
        "回归样本仍在使用 sqrt(delta) 伪高度",
    )

    # The real winding heights can differ by motor.  The inversion must use
    # each rope's own h0 before taking a robust aggregate.
    references = [9.0, 12.0, 15.0, 20.0]
    per_rope_deltas = [2.0 * h0 * 2.0 + 2.0 * 2.0 for h0 in references]
    per_rope = web.relative_height_from_z_squared(
        per_rope_deltas,
        [0.0] * 4,
        baseline_vertical_distance_mm=references,
    )
    check(per_rope["height_available"] is True, "逐绳 h0 参考没有启用高度")
    close(per_rope["height_mm"], 2.0, 1e-9, "逐绳非零 h0 高度反解")

    # A large common lift has a visibly different z² spread for different h0
    # values, even though the true height is identical.  Validation must use
    # the reconstructed per-rope heights, not sqrt(max(delta)-min(delta)).
    large_lift = 120.0
    large_deltas = [2.0 * h0 * large_lift + large_lift * large_lift for h0 in references]
    large = web.relative_height_from_z_squared(
        large_deltas,
        [0.0] * 4,
        baseline_vertical_distance_mm=references,
    )
    check(large["valid"] is True, "不同 h0 的同一大幅抬升被错误拒绝")
    close(large["height_mm"], large_lift, 1e-9, "大幅抬升高度反解")
    close(large["spread_mm"], 0.0, 1e-9, "大幅抬升逐绳高度离散度")
    check(
        large["z_squared_spread_mm"] > web.TRACKING_HEIGHT_ACCEPT_SPREAD_MM,
        "大幅抬升回归没有覆盖旧 z² spread 误判路径",
    )

    # A negative vertical radicand beyond the encoder-noise band is physically
    # impossible. It must not be clamped to zero and accepted as contact.
    impossible_height = web.relative_height_from_z_squared(
        [-100.0] * 4,
        [0.0] * 4,
        baseline_vertical_distance_mm=references,
    )
    check(
        impossible_height["valid"] is False
        and impossible_height["height_available"] is False
        and impossible_height["height_reason"] == "negative_vertical_radicand"
        and impossible_height["height_mm"] is None,
        "强负垂直 radicand 被 sqrt(max(0,...)) 错误吞掉",
    )

    invalid_reference = web.relative_height_from_z_squared(
        [delta] * 4,
        [0.0] * 4,
        baseline_vertical_distance_mm=[14.5, 14.5],
    )
    check(
        invalid_reference["height_mm"] is None
        and invalid_reference["height_reason"] == "baseline_vertical_distance_invalid",
        "不完整 h0 参考没有安全拒绝",
    )


def test_independent_calibration_fit() -> None:
    width, height = 110.0, 148.0
    anchors = tuple(zip(*web.workspace_anchor_coordinates({"workspace_width": width, "workspace_height": height})))
    corners = tuple(zip(*web.workspace_corner_coordinates({"workspace_width": width, "workspace_height": height})))
    truth_slope = [-4.0, 4.1, -4.05, 3.95]
    truth_intercept = [100.0, -200.0, 300.0, -400.0]
    samples = []
    for corner_x, corner_y in corners:
        samples.append([
            truth_intercept[motor] + truth_slope[motor] * math.hypot(corner_x - anchors[motor][0], corner_y - anchors[motor][1])
            for motor in range(4)
        ])
    result, reason = web.compute_calibration(samples, width, height, 40.0)
    check(result is not None, f"四台独立拟合失败: {reason}")
    for actual, expected in zip(result["slope"], truth_slope):
        close(actual, expected, 1e-9, "独立斜率")
    for actual, expected in zip(result["intercept"], truth_intercept):
        close(actual, expected, 1e-9, "独立截距")
    check(len(result["distance_samples_mm"]) == 4, "四台独立绳长曲线没有保存")
    check(len(result["position_samples_deg"]) == 4, "四台绝对位置曲线没有保存")
    for corner, (corner_x, corner_y) in enumerate(corners):
        targets = [web.motor_degrees(motor, corner_x, corner_y, result) for motor in range(4)]
        for actual, expected in zip(targets, samples[corner]):
            close(actual, expected, 1e-9, "四角绝对位置锚点")
        solved = web.position_to_xyz(samples[corner], result)
        check(solved is not None, "分段曲线角点反解失败")
        close(solved[0], corner_x, 1e-9, "分段曲线角点 X")
        close(solved[1], corner_y, 1e-9, "分段曲线角点 Y")
        close(solved[2], 0.0, 1e-9, "分段曲线角点 Z")
    check(result["workspace_width"] == 110.0 and result["workspace_height"] == 148.0, "拟合保存了错误工作区")
    rejected, reason = web.compute_calibration(samples, 150.0, 150.0, 40.0)
    check(rejected is None and "110×148" in reason, "非 110×148 校准没有被拒绝")
    rejected, reason = web.compute_calibration(samples, width, height, 35.0)
    check(rejected is None and "40mm" in reason, "非 40mm 球径元数据没有被拒绝")


def test_real_anchor_frame_regression() -> None:
    """Keep real bad observations as evidence; never promote them to runtime.

    Their global gain looked close to the nominal 12mm drum and their global
    R2 was high, but adjacent-corner segments imply a radius larger than the
    actual 20mm flange.  Passing through every point hid this inconsistency.
    The new acceptance criterion is rejection, not a replacement fitted curve.
    """
    real_samples = [
        [442.1666666667, 73.0333333333, -443.3666666667, 166.0333333333],
        [-80.9, -453.5666666667, -212.2, 430.1],
        [-355.9, 102.3, 349.1666666667, 165.3],
        [-105.3, 355.0, -179.1, -352.1666666667],
    ]
    original_samples = [row[:] for row in real_samples]
    prior_calibration = web.calibration_data()
    result, reason = web.compute_calibration(real_samples, 110.0, 148.0, 40.0)
    check(result is None, "已确认不符合卷筒外缘约束的实测曲线仍被发布为有效标定")
    check("M" in reason and "20" in reason and "CAD" in reason, f"物理曲线拒绝原因不具体: {reason}")
    check(real_samples == original_samples, "拒绝坏拟合时修改了珍贵原始角点样本")
    check(web.calibration_data() == prior_calibration, "拒绝的实测曲线污染了当前标定状态")
    ready, ready_reason = web.internal_model_readiness(result)
    check(not ready and bool(ready_reason), "被物理门拒绝的真实样本仍能产生可运行模型")


def test_external_pose_calibration() -> None:
    """Known external XY samples, not encoder-derived XY, fit the live curves."""
    width, height = 110.0, 148.0
    frame = {"workspace_width": width, "workspace_height": height}
    ax, ay = web.workspace_anchor_coordinates(frame)
    truth_slope = [-4.0, 4.1, -4.05, 3.95]
    truth_intercept = [100.0, -200.0, 300.0, -400.0]
    corners = list(zip(*web.workspace_corner_coordinates(frame)))
    corner_positions = [
        [truth_intercept[m] + truth_slope[m] * math.hypot(x - ax[m], y - ay[m]) for m in range(4)]
        for x, y in corners
    ]
    base, reason = web.compute_calibration(corner_positions, width, height, 40.0)
    check(base is not None, f"外部位姿测试的初始模型失败: {reason}")
    points = [(20.0, 20.0), (55.0, 20.0), (90.0, 20.0), (20.0, 74.0), (55.0, 74.0), (90.0, 74.0), (20.0, 128.0), (55.0, 128.0), (90.0, 128.0)]
    samples = []
    for x, y in points:
        lengths = web._external_pose_cable_lengths(x, y, 0.0, base)
        samples.append(
            {
                "xy_mm": [x, y],
                "yaw_deg": 0.0,
                "positions_deg": [truth_intercept[m] + truth_slope[m] * lengths[m] for m in range(4)],
            }
        )
    fitted, reason = web.compute_external_pose_calibration(samples, base, source="camera")
    check(fitted is not None, f"外部位姿拟合失败: {reason}")
    check(fitted["external_pose_verified"] is True, "外部位姿拟合没有标记独立验证")
    check(fitted["calibration_source"] == "camera", "外部位姿来源没有保存")
    for pose in samples:
        expected = pose["positions_deg"]
        actual = [web.motor_degrees(m, pose["xy_mm"][0], pose["xy_mm"][1], fitted) for m in range(4)]
        for value, target in zip(actual, expected):
            close(value, target, 0.01, "外部位姿曲线回放")
    rejected, reason = web.compute_external_pose_calibration(samples, base, source="equal_current")
    check(rejected is None and "拒绝" in reason, "等电流来源意外进入外部位姿拟合")


def _ideal_gate_calibration() -> dict:
    """Build an explicitly synthetic, consistent CAD fixture without ambient data.

    This fixture is only for gate/solver unit tests.  It does not replace any
    measured regression samples or assert that the physical hardware is ideal.
    compute_calibration reads settings from STATE, so isolate that input and
    restore the complete prior state even if fixture construction fails.
    """
    with web.STATE_LOCK:
        previous_state = copy.deepcopy(web.STATE)
    try:
        with web.STATE_LOCK:
            web.STATE["calibration"] = web.blank_calibration()
        frame = {"workspace_width": 110.0, "workspace_height": 148.0}
        anchors = tuple(zip(*web.workspace_anchor_coordinates(frame)))
        corners = tuple(zip(*web.workspace_corner_coordinates(frame)))
        slopes = [-4.0, 4.1, -4.05, 3.95]
        intercepts = [100.0, -200.0, 300.0, -400.0]
        ideal_samples = [
            [
                intercepts[motor] + slopes[motor]
                * math.hypot(x - anchors[motor][0], y - anchors[motor][1])
                for motor in range(4)
            ]
            for x, y in corners
        ]
        calibration, reason = web.compute_calibration(ideal_samples, 110.0, 148.0, 40.0)
        check(calibration is not None, f"理想几何测试fixture未通过正式标定门: {reason}")
        check(calibration["calibration_source"] == "torque_corner_initializer", "理想fixture缺少明确来源")
        ready, reason = web.internal_model_readiness(calibration)
        check(ready, f"理想几何测试fixture不是可用会话: {reason}")
        return calibration
    finally:
        with web.STATE_LOCK:
            web.STATE.clear()
            web.STATE.update(previous_state)


def test_motor_only_motion_gate() -> None:
    """Four corners initialize the model; redundant closure unlocks motion."""
    calibration = _ideal_gate_calibration()
    calibration["valid"] = True
    calibration["workspace_calibration_verified"] = True
    calibration["geometry_version"] = web.GEOMETRY_VERSION
    calibration["session_id"] = ""
    calibration["session_local_verified"] = False
    calibration["session_tension_held"] = False
    calibration["session_invalid_reason"] = ""
    calibration["fixed_guide_exit_verified"] = False
    calibration["deterministic_spool_payout_verified"] = False
    calibration["spool_geometry_reason"] = "测试：光滑卷筒没有固定导绳眼"
    spool_unobservable = web.mark_internal_model_ready(calibration)
    check(spool_unobservable["internal_model_verified"] is False, "可变卷筒切点意外解锁绝对运动")
    check("导绳" in web.internal_model_readiness(spool_unobservable)[1], "卷筒不可观时锁定原因不明确")

    calibration["fixed_guide_exit_verified"] = True
    calibration["deterministic_spool_payout_verified"] = True
    calibration["orientation_mechanical_decoupling_verified"] = False
    calibration["orientation_mechanical_reason"] = "测试：系绳点仍随球滚转/俯仰"
    calibration["redundant_closure_verified"] = True
    calibration["internal_model_verified"] = False
    mechanically_unobservable = web.mark_internal_model_ready(calibration)
    check(
        mechanically_unobservable["internal_model_verified"] is False,
        "系绳点随球滚转/俯仰时意外解锁平面模型",
    )
    check(
        "滚转" in web.internal_model_readiness(mechanically_unobservable)[1],
        "未机械解耦时的锁定原因不明确",
    )

    calibration["orientation_mechanical_decoupling_verified"] = True
    calibration["orientation_mechanical_reason"] = "测试：平面载架已与按摩球姿态解耦"
    calibration["redundant_closure_verified"] = False
    calibration["redundant_closure_reason"] = "测试：尚未通过第四绳冗余闭合"
    calibration["internal_model_verified"] = False
    try:
        web.require_internal_model_ready(calibration, "轨迹")
    except RuntimeError as exc:
        check("模型" in str(exc) or "校准" in str(exc), "内部模型未就绪时的运动锁定原因不明确")
    else:
        raise AssertionError("内部模型未就绪时意外通过绝对轨迹门控")
    initialized = web.mark_internal_model_ready(calibration)
    check(initialized["internal_model_verified"] is False, "仅四角初始化就意外解锁绝对运动")
    check(
        "冗余闭合" in web.internal_model_readiness(initialized)[1],
        "未通过第四绳闭合时的锁定原因不明确",
    )
    calibration["redundant_closure_verified"] = True
    calibration["redundant_closure_reason"] = "测试：保留路径闭合通过"
    verified = web.mark_internal_model_ready(calibration)
    check(verified["internal_model_verified"] is True, "冗余闭合通过后模型没有被识别为内部可用")
    check(verified["internal_model_source"] == "motor_only_redundant_closure", "内部模型来源未记录为冗余闭合")
    check(verified.get("external_pose_verified") is False, "内部门控错误依赖外部位姿标记")
    web.require_internal_model_ready(verified, "轨迹")


def test_planar_pose_settle_gate() -> None:
    """A coherent free-yaw ball may start; inconsistent four-rope data may not."""
    calibration = _ideal_gate_calibration()
    target_xy = (40.0, 20.0)
    targets = [web.motor_degrees(index, *target_xy, calibration) for index in range(4)]

    def telemetry(positions: list[float]) -> list[web.direct.Telemetry]:
        return [
            web.direct.Telemetry(address, rpm=0.0, position=position)
            for address, position in zip(web.MOTOR_IDS, positions)
        ]

    exact = telemetry(targets)
    check(
        web.draw_target_xy_near_and_stopped(exact, target_xy, calibration),
        "精确平面起点被错误拒绝",
    )

    # The real massage ball can retain an in-plane rotation. Four encoder
    # lengths still recover its centre and a tiny fourth-rope closure residual;
    # yaw is a nuisance state and must not create a 30-second false wait.
    rotated_lengths = web.planar_platform_cable_lengths(
        target_xy[0], target_xy[1], 22.0, calibration
    )
    rotated_positions = [
        web.interpolate_curve(
            web.calibrated_rope_curve(calibration, index),
            length,
            extrapolate=True,
        )
        for index, length in enumerate(rotated_lengths)
    ]
    rotated_pose = web.motor_positions_to_planar_pose(rotated_positions, calibration)
    check(
        rotated_pose is not None and abs(rotated_pose["yaw_deg"] - 22.0) < 0.1,
        "自由偏航回归样本没有反解出旋转角",
    )
    check(
        web.draw_target_xy_near_and_stopped(
            telemetry(rotated_positions), target_xy, calibration
        ),
        "球心已到位时被无关偏航角阻塞",
    )

    offsets = [18.0, -7.0, 5.0, -3.0]
    check(
        web.draw_target_xy_near_and_stopped(
            telemetry([target + offset for target, offset in zip(targets, offsets)]),
            target_xy,
            calibration,
            position_offsets_deg=offsets,
        ),
        "会话局部重基准后的起点仍被拒绝",
    )
    local_offsets = [5.0, -7.0, 9.0, 0.0]
    locally_rebased = telemetry(
        [target + offset for target, offset in zip(targets, local_offsets)]
    )
    local_ok, local_xy = web.active_target_xy_near_and_stopped(
        locally_rebased,
        target_xy,
        (0, 1, 2),
        calibration,
        position_offsets_deg=local_offsets,
    )
    check(local_ok and local_xy is not None, "3+1 局部角度重基准没有恢复目标 XY")

    # This reproduces the previous live failure mode: M3/M4 disagree while the
    # old point model attempted to explain the error as height.  The rigid model
    # must expose a pose/closure mismatch and reject the hand-off.
    inconsistent_positions = [
        targets[0],
        targets[1],
        targets[2] + 20.0,
        targets[3] + 10.0,
    ]
    inconsistent_pose = web.motor_positions_to_planar_pose(inconsistent_positions, calibration)
    check(
        inconsistent_pose is not None
        and (
            math.hypot(inconsistent_pose["x_mm"] - target_xy[0], inconsistent_pose["y_mm"] - target_xy[1])
            > web.TRAJECTORY_XY_SETTLE_TOLERANCE
            or abs(inconsistent_pose["yaw_deg"]) > web.TRAJECTORY_PLANAR_YAW_TOLERANCE_DEG
            or inconsistent_pose["closure_max_mm"] > web.TRAJECTORY_PLANAR_CLOSURE_MAX_TOLERANCE_MM
        ),
        "回归样本没有触发刚体位姿门槛",
    )
    check(
        not web.draw_target_xy_near_and_stopped(
            telemetry(inconsistent_positions), target_xy, calibration
        ),
        "四绳离散超限时仍被 XY-only 起点门控放行",
    )

    # A coherent vertical lift remains visible to the point-model diagnostic.
    # With four lengths alone, free yaw and lift are not independently
    # identifiable; the hardware's board-contact constraint selects z=0 during
    # drawing, while the raw diagnostic is retained rather than inventing a yaw
    # command the mechanism does not have.
    anchors = web.workspace_anchor_coordinates(calibration)
    planar_lengths = [
        math.hypot(target_xy[0] - anchor_x, target_xy[1] - anchor_y)
        for anchor_x, anchor_y in zip(*anchors)
    ]
    raised_positions = [
        web.interpolate_curve(
            web.calibrated_rope_curve(calibration, index),
            math.sqrt(length * length + 10.0 * 10.0),
            extrapolate=True,
        )
        for index, length in enumerate(planar_lengths)
    ]
    raised_pose = web.position_to_xyz(raised_positions, calibration)
    check(raised_pose is not None and raised_pose[2] > 9.0, "抬高回归样本未反解出高度")

    # A common-mode short rope error is clamped to z=0 by the display solver,
    # but its signed z² is physically impossible and must not pass the hand-off.
    short_target_xy = (55.0, 74.0)
    center_lengths = [
        math.hypot(short_target_xy[0] - anchor_x, short_target_xy[1] - anchor_y)
        for anchor_x, anchor_y in zip(*web.workspace_anchor_coordinates(calibration))
    ]
    shortened_positions = [
        web.interpolate_curve(
            web.calibrated_rope_curve(calibration, index),
            max(0.0, length - 12.0),
            extrapolate=True,
        )
        for index, length in enumerate(center_lengths)
    ]
    shortened_pose = web.position_to_xyz(shortened_positions, calibration)
    check(shortened_pose is not None and shortened_pose[2] == 0.0, "负 z² 回归样本未触发显示钳位")
    shortened_consistency = web.absolute_pose_consistency(
        web.encoder_positions_to_lengths(shortened_positions, calibration),
        shortened_pose[0],
        shortened_pose[1],
        calibration,
    )
    check(
        shortened_consistency["physically_possible"] is False,
        "负 z² 回归样本没有被标记为物理不可能",
    )
    check(
        not web.draw_target_xy_near_and_stopped(telemetry(shortened_positions), short_target_xy, calibration),
        "负 z² 不可能姿态仍被平面门控放行",
    )


def test_encoder_feedback_lead() -> None:
    """Feedback catch-up stays small, smooth, and decays when stale."""
    nominal = [100.0, -200.0, 300.0, -400.0]
    first = web.update_encoder_feedback_lead(
        [0.0] * 4,
        nominal,
        (80.0, -180.0, 260.0, -380.0),
        1.0,
        1.02,
    )
    check(first == [1.0, -1.0, 1.0, -1.0], "编码器追赶量第一步没有按速率限制")
    saturated = web.update_encoder_feedback_lead(
        [7.5, -7.5, 7.5, -7.5],
        nominal,
        (0.0, 0.0, 0.0, 0.0),
        1.0,
        1.02,
    )
    check(
        all(abs(value) <= web.TRAJECTORY_ENCODER_LEAD_MAX_DEG for value in saturated),
        "编码器追赶量超过绝对上限",
    )
    decayed = web.update_encoder_feedback_lead(
        saturated,
        nominal,
        (0.0, 0.0, 0.0, 0.0),
        0.0,
        1.50,
    )
    check(all(abs(value) < abs(previous) for value, previous in zip(decayed, saturated)), "反馈过期后追赶量没有衰减")


def test_calibration_load_gate() -> None:
    original_file = web.CALIBRATION_FILE
    original_state = web.state_snapshot()
    candidate = _ideal_gate_calibration()
    candidate.update(
        {
            "valid": True,
            "workspace_calibration_verified": True,
            # Use a truly unsupported pre-migration model here.
            "geometry_version": web.GEOMETRY_VERSION - 3,
            "coordinate_frame": "ball_center_workspace",
            "workspace_width": 110.0,
            "workspace_height": 148.0,
        }
    )
    try:
        with tempfile.TemporaryDirectory() as directory:
            candidate_file = Path(directory) / "calibration.json"
            candidate_file.write_text(json.dumps(candidate), encoding="utf-8")
            web.CALIBRATION_FILE = candidate_file
            web.load_calibration()
            loaded = web.calibration_data()
            check(loaded["valid"] is False, "旧几何版本意外被加载为有效校准")
            check("STEP v7实物几何" in loaded["quality_error"], "旧几何版本拒绝原因不明确")
    finally:
        web.CALIBRATION_FILE = original_file
        with web.STATE_LOCK:
            web.STATE.clear()
            web.STATE.update(original_state)


def test_runtime_settings_persistence() -> None:
    original_file = web.CALIBRATION_FILE
    original_append_process_log = web.append_process_log
    original_state = web.state_snapshot()
    verified = _ideal_gate_calibration()
    verified.update(
        {
            "valid": True,
            "workspace_calibration_verified": True,
            "geometry_version": web.GEOMETRY_VERSION,
            "coordinate_frame": "ball_center_workspace",
            "workspace_width": 110.0,
            "workspace_height": 148.0,
            "anchor_width": 198.0,
            "anchor_height": 148.0,
            "motor_frame_origin_mm": [-44.0, 0.0],
            "spool_winding_radius_mm": 12.0,
            "spool_winding_z_range_mm": [5.0, 16.0],
            "ball_center_contact_z_mm": 25.0,
            "cable_exit_model": web.CABLE_EXIT_MODEL,
            "side": 148.0,
            "ball": 40.0,
        }
    )
    try:
        with tempfile.TemporaryDirectory() as directory:
            web.CALIBRATION_FILE = Path(directory) / "calibration.json"
            web.append_process_log = lambda *args, **kwargs: None
            with web.STATE_LOCK:
                web.STATE["calibration"] = verified
            web.save_calibration_settings({"path_speed_mm_s": 260.0})
            persisted = json.loads(web.CALIBRATION_FILE.read_text(encoding="utf-8"))
            check(persisted["valid"] is True, "运行参数保存意外清除了有效校准")
            check(persisted["workspace_calibration_verified"] is True, "运行参数保存意外清除了校准复验标记")

            unverified = dict(verified)
            unverified["workspace_calibration_verified"] = False
            with web.STATE_LOCK:
                web.STATE["calibration"] = unverified
            web.save_calibration_settings({"path_speed_mm_s": 260.0})
            persisted = json.loads(web.CALIBRATION_FILE.read_text(encoding="utf-8"))
            check(persisted["valid"] is False, "未复验校准仍被运行参数保存为有效")
            check(persisted["workspace_calibration_verified"] is False, "未复验校准被意外标记为已复验")
    finally:
        web.CALIBRATION_FILE = original_file
        web.append_process_log = original_append_process_log
        with web.STATE_LOCK:
            web.STATE.clear()
            web.STATE.update(original_state)


def test_shapes_and_polyline() -> None:
    calibration = _ideal_gate_calibration()
    for kind in ("circle", "square", "triangle", "rectangle", "cross"):
        spec = web.test_shape_spec(kind, 110.0, 148.0)
        web.validate_trajectory_bounds(kind, spec)
        samples = [web.sample_path(kind, index / 100.0, spec) for index in range(101)]
        first = samples[0]
        last = samples[-1]
        check(math.isfinite(first[0]) and math.isfinite(first[1]), f"{kind} 起点无效")
        check(math.isfinite(last[0]) and math.isfinite(last[1]), f"{kind} 终点无效")
        check(
            all(20.0 - 1e-9 <= x <= 90.0 + 1e-9 and 20.0 - 1e-9 <= y <= 128.0 + 1e-9 for x, y in samples),
            f"{kind} 校准验收轨迹没有保持在 STEP v7 对称安全区",
        )
        check(
            all(web.equilibrium_tension_model(x, y, calibration)["ok"] for x, y in samples),
            f"{kind} 校准验收轨迹包含四绳正张力不可行点",
        )
        if kind in ("circle", "square", "triangle", "rectangle"):
            close(math.hypot(first[0] - last[0], first[1] - last[1]), 0.0, 1e-9, f"{kind} 未闭合")

    snake, bounds = web.planar_snake_path(calibration)
    check(bounds == (20.0, 20.0, 90.0, 128.0), "蛇形验收没有移入 STEP v7 对称安全区")
    check(web.PLANAR_SNAKE_TEST_CURRENT == 2000, "蛇形验收没有保存实机调试后的 2000mA 电流上限")
    close(web.PLANAR_SNAKE_TEST_SPEED_MM_S, 4.0, 1e-9, "蛇形验收没有保存实机调试后的 4mm/s 速度")
    check(all(20.0 <= x <= 90.0 and 20.0 <= y <= 128.0 for x, y in snake), "蛇形路径越出安全内圈")
    check(all(web.equilibrium_tension_model(x, y, calibration)["ok"] for x, y in snake), "蛇形路径包含四绳正张力不可行点")

    original = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)]
    refined = web.resample_polyline(original, 0.35)
    check(refined[0] == original[0] and refined[-1] == original[-1], "手绘端点被采样器改写")
    check(all(abs(point[1]) < 1e-9 for point in refined[:29]), "手绘直线被平滑成曲线")
    large = web.resample_polyline([(0.0, 0.0), (2000.0, 0.0), (2000.0, 10.0)], 0.35, 100)
    check(len(large) == 100, "大手绘轨迹没有遵守采样上限")
    check(large[0] == (0.0, 0.0) and large[-1] == (2000.0, 10.0), "采样上限触发时丢失手绘端点")
    sharp = web.resample_polyline([(0.0, 0.0), (50.0, 0.0), (50.0, 50.0)], 0.35)
    cumulative_time, point_speeds = web.time_parameterized_path(sharp, 50.0)
    check(len(cumulative_time) == len(sharp) == len(point_speeds), "尖角时间参数表长度错误")
    check(all(second > first for first, second in zip(cumulative_time, cumulative_time[1:])), "尖角时间参数不是严格递增")
    check(cumulative_time[-1] > 2.0, "尖角轨迹没有为加减速增加运行时间")
    corner_index = sharp.index((50.0, 0.0))
    check(point_speeds[corner_index] <= 20.0, "90 度尖角没有自动降速")
    check(web.sample_cumulative_path(sharp, cumulative_time, 0.0) == sharp[0], "时间参数轨迹起点漂移")
    check(web.sample_cumulative_path(sharp, cumulative_time, 1.0) == sharp[-1], "时间参数轨迹终点漂移")
    jittered_line = [
        (index * 0.35, 0.18 * math.sin(index * 0.65))
        for index in range(144)
    ]
    jitter_time, _ = web.time_parameterized_path(jittered_line, 50.0)
    jitter_length = sum(math.dist(first, second) for first, second in zip(jittered_line, jittered_line[1:]))
    check(jitter_length / jitter_time[-1] >= 45.0, "细小手绘抖动仍把 50mm/s 轨迹错误降成慢速")
    try:
        web.validate_trajectory_bounds("rectangle", web.test_shape_spec("rectangle", 150.0, 150.0))
    except RuntimeError as exc:
        check("110×148" in str(exc), "旧工作区轨迹没有被拒绝")
    else:
        raise AssertionError("旧工作区轨迹意外通过边界检查")


def test_grid_compensation_offline() -> None:
    calibration = {
        "workspace_width": 110.0,
        "workspace_height": 148.0,
        "slope": [-4.0, 4.1, -4.05, 3.95],
        "intercept": [100.0, -200.0, 300.0, -400.0],
        "slope_trim_percent": [0.0] * 4,
        "samples": [],
    }
    points = web.grid_sampling_points()
    check(len(points) == 63, "默认网格不是 7×9 的 63 个点")
    check(points[0]["x_mm"] == 0.0 and points[0]["y_mm"] == 0.0, "网格没有从左下边界开始")
    check(points[8]["x_mm"] == 110.0 and points[8]["y_mm"] == 0.0, "蛇形第一行没有走到右边界")
    close(points[9]["y_mm"], 148.0 / 6.0, 1e-6, "蛇形第二行 Y 间距错误")
    check(points[9]["x_mm"] == 110.0, "蛇形第二行没有从右边界接续")
    check(points[54]["x_mm"] == 0.0 and points[54]["y_mm"] == 148.0, "网格没有覆盖左上边界")
    check(len(web.grid_sampling_path(3, 4)) == 12, "可配置网格行列没有生效")

    raw = []
    for point in points:
        positions = [
            web.motor_degrees(index, point["x_mm"], point["y_mm"], calibration)
            for index in range(4)
        ]
        raw.append(
            {
                "row": point["row"],
                "col": point["col"],
                "absolute_positions_deg": positions,
                "phase_current_ma": [150.0 + point["row"], 160.0, 175.0, 190.0],
            }
        )
    table = web.build_grid_compensation(raw, calibration, plane_reference_z_mm=0.0)
    check(table["valid"] is True, "完整网格没有通过离线有效性检查")
    check(table["enabled"] is False, "离线生成的网格意外自动启用")
    check(table["valid_sample_count"] == 63, "有效网格点数量错误")
    check(web.validate_grid_compensation(table)[0] is True, "完整网格持久化结构复核失败")
    check(all(len(node["absolute_positions_deg"]) == 4 for node in table["samples"]), "网格节点未保存四电机绝对位置")
    check(all(len(node["phase_current_ma"]) == 4 for node in table["samples"]), "网格节点未保存四台相电流")
    check(all(node["solved_xy_mm"] is not None and node["solved_z_mm"] is not None for node in table["samples"]), "网格节点未保存 XY/Z 解算")
    check(all(node["fit_error_mm"] is not None for node in table["samples"]), "网格节点未保存拟合误差")
    check(all(node["position_compensation_deg"] is not None for node in table["samples"]), "网格节点未生成位置补偿")
    check(all(node["tension_feedforward_ma"] is not None for node in table["samples"]), "网格节点未生成张力前馈")
    custom_points = web.grid_sampling_points(3, 4)
    custom_raw = [
        {
            "row": point["row"],
            "col": point["col"],
            "absolute_positions_deg": [web.motor_degrees(index, point["x_mm"], point["y_mm"], calibration) for index in range(4)],
            "phase_current_ma": [150.0, 160.0, 175.0, 190.0],
        }
        for point in custom_points
    ]
    attached = web.attach_grid_compensation(calibration, custom_raw, rows=3, cols=4, plane_reference_z_mm=0.0)
    check(attached["grid_rows"] == 3 and attached["grid_cols"] == 4, "可配置网格没有同步持久化行列元数据")
    check(web.validate_grid_compensation(attached["grid_compensation"])[0] is True, "可配置网格结构复核失败")

    enabled = json.loads(json.dumps(table))
    enabled["enabled"] = True
    lookup = web.interpolate_grid_compensation(enabled, 27.5, 56.25)
    check(lookup["ok"] is True and lookup["method"] == "bilinear", "完整网格没有使用双线性插值")
    check(lookup["apply_allowed"] is True, "完整且显式启用的网格没有允许离线补偿应用")
    check(len(lookup["tension_feedforward_ma"]) == 4 and len(lookup["position_compensation_deg"]) == 4, "插值没有返回四电机向量")
    enabled_calibration = dict(calibration)
    enabled_calibration["grid_compensation"] = enabled
    current_limits = web.grid_current_limits_at(27.5, 56.25, enabled_calibration, 1000)
    check(len(current_limits) == 4 and all(300 <= value <= web.OPERATING_CURRENT_MAX_MA for value in current_limits), "网格张力前馈没有生成2A以内的安全电流上限")
    position_map = json.loads(json.dumps(enabled))
    for node in position_map["samples"]:
        node["position_compensation_deg"] = [1.0, -2.0, 3.0, -4.0]
    position_calibration = dict(calibration)
    position_calibration["grid_compensation"] = position_map
    nominal = [web.motor_degrees(index, 27.5, 56.25, position_calibration) for index in range(4)]
    corrected = web.grid_motor_positions_at(27.5, 56.25, position_calibration)
    check(
        all(abs(actual - expected - delta) < 1e-5 for actual, expected, delta in zip(corrected, nominal, [1.0, -2.0, 3.0, -4.0])),
        "网格没有把四台实测绝对位置补偿到轨迹目标",
    )
    low_tension = web.build_grid_compensation(raw, calibration, target_current_ma=300.0, plane_reference_z_mm=0.0)
    check(low_tension["valid"] is False and low_tension["enabled"] is False, "低于张力下限的网格意外通过")
    boundary = web.interpolate_grid_compensation(enabled, -10.0, 200.0)
    check(boundary["clamped"] is True and boundary["x_mm"] == 0.0 and boundary["y_mm"] == 148.0, "网格边界没有安全夹紧")

    incomplete_raw = [record for record in raw if not (record["row"] == 0 and record["col"] == 0)]
    incomplete = web.build_grid_compensation(incomplete_raw, calibration, plane_reference_z_mm=0.0)
    check(incomplete["valid"] is False and incomplete["enabled"] is False, "缺失网格点意外被标记为可用")
    fallback = web.interpolate_grid_compensation(incomplete, 0.0, 0.0)
    check(fallback["ok"] is True and fallback["method"] == "nearest_valid", "缺失网格点没有降级到最近有效点")
    check(fallback["apply_allowed"] is False and fallback["fallback"] is True, "最近邻降级没有保留失效/降级标志")
    bad_version = dict(table)
    bad_version["version"] = 99
    check(web.validate_grid_compensation(bad_version)[0] is False, "未知网格版本意外通过校验")
    normalized_bad = web.normalize_grid_compensation(bad_version)
    check(normalized_bad["valid"] is False and normalized_bad["enabled"] is False, "未知网格版本没有被强制失效")

    center = web.equilibrium_current_targets(55.0, 74.0, 120.0, calibration)
    check(center["ok"] is True, "中心点张力平衡模型无法求解")
    check(all(abs(value - 120.0) < 1e-9 for value in center["target_currents_ma"]), "中心点没有得到四台相同张力目标")
    check(abs(center["model"]["resultant_moment_mm"]) < 1e-9, "中心点四连接绳存在残余力矩")
    off_center = web.equilibrium_current_targets(50.0, 100.0, 120.0, calibration)
    check(off_center["ok"] is True and len(set(off_center["target_currents_ma"])) > 1, "偏心点没有生成差异化张力目标")
    check(off_center["model"]["residual"] <= web.GRID_FORCE_BALANCE_MAX_RESIDUAL, "偏心点合力残差超过模型门槛")
    check(abs(off_center["model"]["resultant_moment_mm"]) < 1e-6, "偏心点张力存在残余力矩")
    edge = web.equilibrium_current_targets(0.0, 0.0, 120.0, calibration)
    check(edge["ok"] is False, "可行域外边界点被错误地伪装成正张力可行")

    original_file = web.CALIBRATION_FILE
    original_state = web.state_snapshot()
    try:
        with tempfile.TemporaryDirectory() as directory:
            web.CALIBRATION_FILE = Path(directory) / "calibration.json"
            saved = web.save_grid_compensation(raw, calibration, plane_reference_z_mm=0.0)
            persisted = json.loads(web.CALIBRATION_FILE.read_text(encoding="utf-8"))
            check(persisted["grid_compensation"]["version"] == web.GRID_COMPENSATION_VERSION, "网格版本没有持久化")
            check(saved["grid_compensation"]["valid"] is True, "持久化封装没有返回网格结构")
    finally:
        web.CALIBRATION_FILE = original_file
        with web.STATE_LOCK:
            web.STATE.clear()
            web.STATE.update(original_state)


def test_center_tension_datum() -> None:
    """Equal-current stability may be diagnosed but must never alter geometry."""
    width, height = 110.0, 148.0
    anchors = tuple(zip(*web.workspace_anchor_coordinates({"workspace_width": width, "workspace_height": height})))
    calibration = {
        "valid": True,
        "workspace_width": width,
        "workspace_height": height,
        "coordinate_frame": "ball_center_workspace",
        "slope": [-4.0, 4.1, -4.05, 3.95],
        "intercept": [100.0, -200.0, 300.0, -400.0],
        "slope_trim_percent": [0.0] * 4,
        "samples": [],
    }
    corner_positions = [
        [
            calibration["intercept"][motor]
            + calibration["slope"][motor]
            * math.hypot(corner_x - anchors[motor][0], corner_y - anchors[motor][1])
            for motor in range(4)
        ]
        for corner_x, corner_y in anchors
    ]
    calibration["samples"] = corner_positions
    calibration["distance_samples_mm"] = [
        [math.hypot(corner_x - anchors[motor][0], corner_y - anchors[motor][1]) for corner_x, corner_y in anchors]
        for motor in range(4)
    ]
    calibration["position_samples_deg"] = [
        [corner_positions[corner][motor] for corner in range(4)]
        for motor in range(4)
    ]
    mechanism_center = (web.MECHANISM_CENTER_X_MM, web.MECHANISM_CENTER_Y_MM)
    targets = [web.motor_degrees(motor, *mechanism_center, calibration) for motor in range(4)]
    confirmations = []
    for sample_index in range(8):
        confirmations.append(
            [
                web.direct.Telemetry(
                    motor + 1,
                    phase_current=190 + motor * 4,
                    rpm=0.0,
                    position=targets[motor] + (0.03 if sample_index % 2 else 0.0),
                    temperature=25,
                    flags=0x03,
                )
                for motor in range(4)
            ]
        )
    level = web.summarize_center_tension_level(confirmations, calibration, 300, 120)
    check(level["passed"] is True, f"等力矩稳定性离线诊断未通过: {level['reason']}")
    check(level["diagnostic_only"] is True and level["pose_verified"] is False, "等力矩诊断被误标为位姿验证")
    check(level["xy_error_mm"] <= 0.01, "中心点反算 XY 没有回到工作区中心")
    check(level["solved_z_mm"] <= 0.01, "平面中心点反算高度不为零")
    staged_levels = []
    for command_current, drift in ((120, -0.2), (150, 0.0), (200, 0.2)):
        staged = dict(level)
        staged["command_current_ma"] = command_current
        staged["absolute_positions_deg"] = [value + drift for value in level["absolute_positions_deg"]]
        staged["phase_current_ma"] = [190.0 + motor for motor in range(4)]
        staged_levels.append(staged)
    selected, selection_reason = web.select_center_tension_level(staged_levels, 120)
    check(selected is not None, f"多档位中心确认意外失败: {selection_reason}")
    check(selected["stability_run_levels_ma"] == [120, 150, 200], "中心确认没有要求连续三个递增档位")
    check(
        selected["absolute_positions_deg"] == staged_levels[0]["absolute_positions_deg"],
        "中心确认没有使用操作者指定电流档的位置",
    )
    drifting_levels = []
    for command_current, drift in ((120, 0.0), (150, 2.5), (200, 5.0)):
        drifting = dict(level)
        drifting["command_current_ma"] = command_current
        drifting["absolute_positions_deg"] = [value + drift for value in level["absolute_positions_deg"]]
        drifting_levels.append(drifting)
    rejected, rejection_reason = web.select_center_tension_level(drifting_levels, 120)
    check(rejected is None and rejection_reason, "中心确认错误地接受了跨档位持续漂移")
    updated, reason = web.apply_center_tension_datum(
        calibration,
        selected,
        {"status": "pass", "recommended_current_ma": 300},
    )
    check(updated is None and "等电流" in reason, "等电流状态仍可写入绝对位置曲线")
    check(all(len(row) == 4 for row in calibration["distance_samples_mm"]), "拒绝中心点时污染了绳长曲线")
    check(all(len(row) == 4 for row in calibration["position_samples_deg"]), "拒绝中心点时污染了位置曲线")


def test_cross_calibration_data_frame() -> None:
    """Recalibration offsets must not change the reusable rope model data."""
    first = {
        "workspace_width": 110.0,
        "workspace_height": 148.0,
        "coordinate_frame": "ball_center_workspace",
        "slope": [-4.0, 4.0, -4.0, 4.0],
        "intercept": [100.0, -200.0, 300.0, -400.0],
        "slope_trim_percent": [0.0] * 4,
        "encoder_positive_direction": [0, 1, 0, 1],
    }
    second = dict(first)
    second["intercept"] = [value + delta for value, delta in zip(first["intercept"], [30.0, -25.0, 18.0, -12.0])]
    relative = [12.0, 20.0, 8.0, 16.0]
    absolute_first = [intercept + slope * length for intercept, slope, length in zip(first["intercept"], first["slope"], relative)]
    absolute_second = [intercept + slope * length for intercept, slope, length in zip(second["intercept"], second["slope"], relative)]
    context_first = web.calibration_model_context(first)
    context_second = web.calibration_model_context(second)
    check(context_first["model_family_key"] == context_second["model_family_key"], "截距平移改变了可复用模型标识")
    check(
        web.calibration_relative_rope_lengths(absolute_first, first) == web.calibration_relative_rope_lengths(absolute_second, second),
        "重新校准后的归一化绳长不一致",
    )
    reservoir = web.BoundedReservoir(64)
    for value in range(10000):
        reservoir.add(value)
    check(reservoir.seen == 10000 and len(reservoir.values) == 64, "有界统计缓存仍随测试长度增长")
    check(reservoir.values[-1] is not None, "蛇形统计不能从有界缓存读取最新样本")
    timing = web.TimingAccumulator()
    for stamp in (0.0, 0.01, 0.02, 0.03):
        timing.add(stamp)
    stats = web._timing_stats(timing, 0.0, 0.03, 500.0)
    check(stats["frames"] == 4 and abs(stats["interval_ms"]["avg"] - 10.0) < 1e-9, "流式节拍统计错误")


def test_four_motor_absolute_frame() -> None:
    packet = web.four_motor_position_frame([100.0, -200.0, 300.0, -400.0], [45.0] * 4, 300)
    check(len(packet) == 61, f"四电机合帧长度错误: {len(packet)}")
    for motor in range(4):
        offset = 4 + motor * 14
        check(packet[offset] == motor + 1, "合帧电机 ID 顺序错误")
        check(packet[offset + 1] == 0xCB, "合帧不是 0xCB 位置命令")
        check(packet[offset + 9] == 0x01, "合帧不是 X 固件绝对位置模式 01")
        check(packet[offset + 10] == 0x00, "合帧意外使用缓存同步模式")
        check(packet[offset + 13] == 0x6B, "子帧结束符错误")
    check(packet[-1] == 0x6B, "合帧结束符错误")

    torque = web.four_motor_torque_frame([150, 210, 330, 450])
    check(len(torque) == 49, f"四电机张力合帧长度错误: {len(torque)}")
    check(torque[:2] == bytes((0x00, 0xAA)), "四电机张力没有使用同步 0xAA 合帧")
    for motor in range(4):
        offset = 4 + motor * 11
        check(torque[offset:offset + 2] == bytes((motor + 1, 0xC5)), "四电机张力子帧顺序错误")
        check(torque[offset + 2] == web.pull_directions()[motor], "四电机张力方向未使用安装方向")
        check(
            int.from_bytes(torque[offset + 5:offset + 7], "big") == [150, 210, 330, 450][motor],
            "四电机张力电流编码错误",
        )
    check(torque[-1] == 0x6B, "四电机张力合帧结束符错误")
    check(web.slewed_current(150, 300, 20) == 170, "张力上升限速错误")
    check(web.slewed_current(300, 150, 20) == 280, "张力下降限速错误")

    hybrid = web.three_position_one_torque_frame(
        [100.0, -200.0, 300.0, -400.0],
        passive_index=3,
        passive_current_ma=120,
        rpms=[45.0] * 4,
        position_current=2000,
    )
    check(len(hybrid) == 58, f"三位置一张力合帧长度错误: {len(hybrid)}")
    check(hybrid[:2] == bytes((0x00, 0xAA)), "3+1 命令没有使用同步 0xAA 合帧")
    for motor in range(3):
        offset = 4 + motor * 14
        check(hybrid[offset] == motor + 1 and hybrid[offset + 1] == 0xCB, "3+1 的位置子帧顺序错误")
    torque_offset = 4 + 3 * 14
    check(hybrid[torque_offset:torque_offset + 3] == bytes((4, 0xC5, web.pull_directions()[3])), "3+1 的第四电机不是拉绳力矩模式")
    check(int.from_bytes(hybrid[torque_offset + 5:torque_offset + 7], "big") == 120, "3+1 张力电流编码错误")
    check(hybrid[torque_offset + 10] == 0x6B and hybrid[-1] == 0x6B, "3+1 合帧结束符错误")

    two_plus_two = web.two_position_two_torque_frame(
        [100.0, -200.0, 300.0, -400.0],
        position_indices=(0, 2),
        torque_currents_ma=[0, 125, 0, 135],
        rpms=[45.0] * 4,
        position_current=2000,
    )
    check(len(two_plus_two) == 55, f"两位置两张力合帧长度错误: {len(two_plus_two)}")
    check(two_plus_two[:2] == bytes((0x00, 0xAA)), "2+2 命令没有使用同步 0xAA 合帧")
    offset = 4
    for motor in range(4):
        opcode = 0xCB if motor in (0, 2) else 0xC5
        check(two_plus_two[offset:offset + 2] == bytes((motor + 1, opcode)), f"2+2 M{motor + 1} 子帧模式错误")
        if opcode == 0xC5:
            expected_current = 125 if motor == 1 else 135
            check(int.from_bytes(two_plus_two[offset + 5:offset + 7], "big") == expected_current, "2+2 张力电流编码错误")
        offset += 14 if opcode == 0xCB else 11
    check(offset == len(two_plus_two) - 1 and two_plus_two[-1] == 0x6B, "2+2 合帧结束符错误")

    # A session-local absolute offset must not be interpreted as fresh motion
    # on every control tick. Only the small Cartesian segment may set RPM.
    calibration = _ideal_gate_calibration()
    x, y, next_x, next_y = 40.0, 20.0, 40.5, 20.0
    nominal_now = web.grid_motor_positions_at(x, y, calibration)
    nominal_next = web.grid_motor_positions_at(next_x, next_y, calibration)
    session_offsets = [20.0, -15.0, 10.0, -5.0]
    overrides = [
        target + correction
        for target, correction in zip(nominal_next, session_offsets)
    ]
    expected_rpms = [
        web.trajectory_target_rpm(current, target, 0.01)
        for current, target in zip(nominal_now, nominal_next)
    ]
    offset_packet = web.trajectory_frame(
        x,
        y,
        next_x,
        next_y,
        0.01,
        1200,
        calibration,
        target_overrides=overrides,
    )
    encoded_rpms = [
        int.from_bytes(offset_packet[7 + motor * 14:9 + motor * 14], "big") / 10.0
        for motor in range(4)
    ]
    check(
        all(abs(actual - expected) <= 0.05 for actual, expected in zip(encoded_rpms, expected_rpms)),
        f"四位置会话偏移被错算成速度: {encoded_rpms} vs {expected_rpms}",
    )
    hybrid_offset_packet = web.hybrid_trajectory_frame(
        x,
        y,
        next_x,
        next_y,
        0.01,
        passive_index=3,
        passive_current_ma=150,
        position_current=1200,
        calibration=calibration,
        target_overrides=overrides,
    )
    hybrid_rpms = [
        int.from_bytes(hybrid_offset_packet[7 + motor * 14:9 + motor * 14], "big") / 10.0
        for motor in range(3)
    ]
    check(
        all(abs(actual - expected) <= 0.05 for actual, expected in zip(hybrid_rpms, expected_rpms[:3])),
        f"3+1 会话偏移被错算成速度: {hybrid_rpms} vs {expected_rpms[:3]}",
    )


def test_calibration_smooth_ramp() -> None:
    start = [900, 150, 150, 150]
    transit = web.calibration_transit_current_vector(1, 900, 150)
    check(transit == [300, 900, 300, 300], f"角点动态制动向量错误: {transit}")
    plan, meta = web.calibration_torque_ramp_plan(start, transit)
    check(plan[0] == start and plan[-1] == transit, "标定五次曲线没有精确包含两个端点")
    check(2.30 <= meta["duration_s"] <= 2.50, f"标定换角时长异常: {meta}")
    check(meta["max_command_step_ma"] <= 13, f"标定单帧电流跃变过大: {meta}")
    check(meta["max_slew_ma_per_s"] <= 650.0, f"标定电流变化率超限: {meta}")
    for motor in range(4):
        values = [row[motor] for row in plan]
        if transit[motor] >= start[motor]:
            check(values == sorted(values), f"M{motor + 1} 标定电流上升不单调")
        else:
            check(values == sorted(values, reverse=True), f"M{motor + 1} 标定电流下降不单调")
        check(min(values) >= 150, f"M{motor + 1} 换角过程中失去正张力")
    packet = web.four_motor_torque_frame(
        transit,
        torque_slope=web.CALIBRATION_TORQUE_DRIVER_SLOPE_MA_PER_S,
        torque_max_rpm=web.CALIBRATION_CORNER_MAX_RPM,
    )
    check(len(packet) == 49 and packet[:2] == bytes((0x00, 0xAA)), "标定没有使用四电机同步合帧")
    for motor in range(4):
        offset = 4 + motor * 11
        check(
            int.from_bytes(packet[offset + 3:offset + 5], "big") == web.CALIBRATION_TORQUE_DRIVER_SLOPE_MA_PER_S,
            "标定驱动器内部电流斜率未写入合帧",
        )
        check(
            int.from_bytes(packet[offset + 8:offset + 10], "big") == round(web.CALIBRATION_CORNER_MAX_RPM * 10.0),
            "标定低速上限未写入合帧",
        )
    seat, seat_meta = web.calibration_torque_ramp_plan(
        transit,
        [150, 900, 150, 150],
        step_ma=15,
        dwell_seconds=0.05,
    )
    check(seat[0] == transit and seat[-1] == [150, 900, 150, 150], "低速落座端点错误")
    check(seat_meta["max_slew_ma_per_s"] <= 350.0, f"低速落座变化率过快: {seat_meta}")


def test_timed_feedback_protocol() -> None:
    check(
        web.timed_feedback_frame(1) == bytes((1, 0x11, 0x18, 0x43, 0x00, 0x0A, 0x6B)),
        "0x11/0x43 定时回传命令错误",
    )
    packet = bytearray(37)
    packet[0:4] = bytes((1, 0x43, 0x25, 0x0C))
    packet[4:6] = (24000).to_bytes(2, "big")
    packet[6:8] = (100).to_bytes(2, "big")
    packet[8:10] = (200).to_bytes(2, "big")
    packet[20:22] = (80).to_bytes(2, "big")
    packet[23:27] = (12345).to_bytes(4, "big")
    packet[33] = 30
    packet[35] = 2
    packet[36] = 0x6B
    raw = bytes(packet)
    check(web.valid_timed_feedback_43_packet(raw, 1, {}), "合法 0x43 回包被拒绝")
    records, response = web._consume_timed_position_buffer(bytearray(raw))
    check(response is None and len(records) == 1, "固定长度 0x43 回包没有被解析")
    close(records[0].position, 1234.5, 1e-9, "0x43 实时位置解析")
    check(records[0].phase_current == 200, "0x43 相电流解析")
    check(records[0].raw_packet_hex == raw.hex(), "0x43 原始回包没有随解码结果保存")
    aggregate = web.multi_motor_timed_feedback_frame(10, 0x43)
    check(len(aggregate) == 33, "四电机定时反馈合帧长度错误")
    check(aggregate[0:2] == bytes((0x00, 0xAA)), "定时反馈没有使用0xAA广播合帧")
    for motor in range(4):
        offset = 4 + motor * 7
        check(aggregate[offset] == motor + 1, "定时反馈合帧电机 ID 顺序错误")
        check(aggregate[offset + 1:offset + 4] == bytes((0x11, 0x18, 0x43)), "定时反馈子命令错误")
        check(aggregate[offset + 4:offset + 6] == bytes((0x00, 0x0A)), "定时反馈周期错误")
        check(aggregate[offset + 6] == 0x6B, "定时反馈子帧结束符错误")


def test_feedback_time_alignment() -> None:
    snapshot = [
        web.TimedFeedbackRecord(1, 100.0, 100, 1.000, rpm=100),
        web.TimedFeedbackRecord(2, 100.0, 100, 0.990, rpm=100),
        web.TimedFeedbackRecord(3, 100.0, 100, 0.995, rpm=100),
        web.TimedFeedbackRecord(4, 100.0, 100, 1.000, rpm=100),
    ]
    aligned = web.time_aligned_feedback_positions(snapshot)
    check(len(aligned) == 4, "四电机时间对齐没有保留完整快照")
    close(aligned[0], 100.0, 1e-9, "时间对齐最新样本不应被修改")
    close(aligned[1], 106.0, 1e-9, "RPM 时间外推错误")
    close(aligned[2], 103.0, 1e-9, "中间样本时间外推错误")
    close(aligned[3], 100.0, 1e-9, "时间对齐末样本错误")


def test_target_reached_gate() -> None:
    targets = [10.0, -20.0, 30.0, -40.0]
    reached = (10.0, -20.0, 30.0, -40.0)
    check(
        web.trajectory_feedback_target_reached(reached, (0.0, 0.0, 0.0, 0.0), (0x03,) * 4, targets),
        "手册位置到达标志门控未接受合法到位反馈",
    )
    check(
        not web.trajectory_feedback_target_reached(reached, (0.0, 0.0, 0.0, 0.0), (0x01,) * 4, targets),
        "未到位标志意外允许前往下一个目标",
    )
    check(
        not web.trajectory_feedback_target_reached(reached, (20.0, 0.0, 0.0, 0.0), (0x03,) * 4, targets),
        "运动中的 RPM 意外允许前往下一个目标",
    )
    close(web.trajectory_target_rpm(0.0, 0.4, 0.01), 8.333333333333334, 1e-9, "10mm/s 轨迹速度换算")
    close(web.trajectory_target_rpm(0.0, 0.04, 0.01), 1.0, 1e-9, "低速轨迹速度下限被意外提高")
    check(web.trajectory_controller_mode({}) == "four_position_force", "绘制控制器默认值不是实测优胜的四位置微预紧")
    check(
        web.trajectory_controller_mode({"controller_mode": "four_position_force"}) == "four_position_force",
        "力位诊断控制器没有保留明确模式",
    )
    legacy_rejected = False
    try:
        web.trajectory_controller_mode({"controller_mode": "three_plus_one"})
    except RuntimeError:
        legacy_rejected = True
    check(legacy_rejected, "旧 3+1 请求仍可重新进入生产绘制路径")


def test_force_position_outer_loop() -> None:
    calibration = {
        "workspace_width": 110.0,
        "workspace_height": 148.0,
        "slope": [-4.0, 4.1, -4.05, 3.95],
        "intercept": [100.0, -200.0, 300.0, -400.0],
        "slope_trim_percent": [0.0] * 4,
        "samples": [],
    }
    profile = web.force_position_self_stress_bias(55.0, 74.0, 0.20, calibration)
    check(profile["ok"] is True and profile["fallback"] is False, "中心自应力模型没有得到正张力解")
    check(len(profile["position_bias_deg"]) == 4, "自应力偏置没有覆盖四台电机")
    check(max(profile["length_bias_mm"]) - min(profile["length_bias_mm"]) < 1e-5, "对称中心的四绳收缩不对称")
    check(all(value <= 1e-9 for value in profile["length_bias_mm"]), "自应力偏置意外放长了绳子")
    stronger = web.force_position_self_stress_bias(55.0, 74.0, 0.40, calibration)
    check(
        sum(abs(value) for value in stronger["position_bias_deg"])
        > sum(abs(value) for value in profile["position_bias_deg"]),
        "提高预紧标量没有增加四电机位置偏置",
    )
    low, low_state = web.update_force_position_contraction(
        0.10,
        [100.0, 100.0, 100.0, 100.0],
        [300.0, 300.0, 300.0, 300.0],
        0.10,
        2000,
    )
    check(0.10 < low <= 0.145001 and low_state["updated"], "最低张力不足时预紧没有按限速增加")
    deadband, deadband_state = web.update_force_position_contraction(
        0.20,
        [300.0, 300.0, 300.0, 300.0],
        [300.0, 300.0, 300.0, 300.0],
        0.10,
        2000,
    )
    close(deadband, 0.20, 1e-9, "张力死区内预紧不应抖动")
    check(not deadband_state["updated"], "张力死区内仍在调整预紧")
    high, high_state = web.update_force_position_contraction(
        0.20,
        [1900.0, 500.0, 500.0, 500.0],
        [300.0, 300.0, 300.0, 300.0],
        0.10,
        2000,
    )
    check(0.1779 <= high < 0.20 and high_state["updated"], "接近 2A 保护上限时没有只放松预紧")
    invalid, invalid_state = web.update_force_position_contraction(
        "invalid",
        [None, 100.0, 100.0, 100.0],
        [300.0] * 4,
        0.01,
        2000,
    )
    close(invalid, web.FORCE_POSITION_CONTRACTION_MIN_MM, 1e-9, "无效反馈回退值不安全")
    check(not invalid_state["updated"], "无效反馈不应更新张力外环")


def test_manual_test_recorder() -> None:
    old_root = web.MANUAL_TEST_RUNS_ROOT
    with tempfile.TemporaryDirectory() as temporary:
        web.MANUAL_TEST_RUNS_ROOT = Path(temporary)
        context = web.begin_manual_test_run(
            "offline_visual_probe",
            (
                "custom",
                {
                    "controller_mode": "four_position",
                    "operator_input_absolute_segments": [[(5.0, 5.0), (100.0, 140.0)]],
                },
            ),
        )
        web.manual_test_emit("offline_probe", value=42)
        web.end_manual_test_run(context, result=True)
        run_directory = Path(str(context["directory"]))
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not (run_directory / "integrity.json").exists():
            time.sleep(0.02)
        check((run_directory / "manifest.json").exists(), "单次实机测试未保存 manifest")
        check((run_directory / "events.jsonl").exists(), "单次实机测试未保存逐帧事件")
        check((run_directory / "final_state.json").exists(), "单次实机测试未保存最终状态")
        check((run_directory / "visual-comparison.svg").exists(), "单次实机测试未生成最终叠图")
        check((run_directory / "integrity.json").exists(), "单次实机测试未生成完整性校验")
        events = [json.loads(line) for line in (run_directory / "events.jsonl").read_text(encoding="utf-8").splitlines()]
        check([item["sequence"] for item in events] == list(range(1, len(events) + 1)), "测试事件序号不连续")
        check(any(item["event"] == "offline_probe" for item in events), "测试语义事件丢失")
        integrity = json.loads((run_directory / "integrity.json").read_text(encoding="utf-8"))
        check(integrity["events"]["records"] == len(events), "完整性文件的事件计数错误")
        check(len(integrity["events"]["sha256"]) == 64, "逐帧事件缺少 SHA-256")
        svg = (run_directory / "visual-comparison.svg").read_text(encoding="utf-8")
        check("最终轨迹直观对比" in svg and "four_position" in svg, "最终视觉叠图缺少控制器标识")
    web.MANUAL_TEST_RUNS_ROOT = old_root


def test_synchronous_serial_traffic_hook() -> None:
    class FakeConnection:
        port = "TEST"
        baudrate = 921600
        _y42_direction_mode = "auto"

        def __init__(self) -> None:
            self.reply = bytearray((1, 0x3A, 0x02, 0x6B))
            self.written = b""

        @property
        def in_waiting(self) -> int:
            return len(self.reply)

        def reset_input_buffer(self) -> None:
            return

        def write(self, payload: bytes) -> int:
            self.written = bytes(payload)
            return len(payload)

        def flush(self) -> None:
            return

        def read(self, count: int) -> bytes:
            payload = bytes(self.reply[:count])
            del self.reply[:count]
            return payload

    captured = []
    web.direct.set_traffic_callback(captured.append)
    try:
        connection = FakeConnection()
        tx = bytes((1, 0x3A, 0x6B))
        rx = web.direct.transact(connection, tx, first_byte_timeout=0.001, inter_byte_quiet=0.0)
        check(connection.written == tx and rx == bytes((1, 0x3A, 0x02, 0x6B)), "伪串口事务失败")
        check(len(captured) == 1, "同步串口事务没有产生唯一原始记录")
        check(captured[0]["tx_hex"] == tx.hex(), "同步串口 TX 原始字节缺失")
        check(captured[0]["rx_hex"] == rx.hex(), "同步串口 RX 原始字节缺失")
    finally:
        web.direct.set_traffic_callback(web.manual_test_serial_transaction)


# Apply the same isolation when invoked by CLI, unittest, or another harness.
# The real-bad-sample test above remains a separate rejection regression.
for _case_name, _case_function in tuple(globals().items()):
    if _case_name.startswith("test_") and callable(_case_function):
        globals()[_case_name] = _isolated_test_case(_case_function)


def main() -> int:
    test_workspace_and_solver()
    test_relative_height_requires_nonzero_baseline_reference()
    test_independent_calibration_fit()
    test_real_anchor_frame_regression()
    test_external_pose_calibration()
    test_motor_only_motion_gate()
    test_planar_pose_settle_gate()
    test_encoder_feedback_lead()
    test_calibration_load_gate()
    test_runtime_settings_persistence()
    test_shapes_and_polyline()
    test_grid_compensation_offline()
    test_center_tension_datum()
    test_cross_calibration_data_frame()
    test_four_motor_absolute_frame()
    test_calibration_smooth_ramp()
    test_timed_feedback_protocol()
    test_feedback_time_alignment()
    test_target_reached_gate()
    test_force_position_outer_loop()
    test_manual_test_recorder()
    test_synchronous_serial_traffic_hook()
    print("OFFLINE_CHECK_PASS: fixed-workspace, signed-z2-diagnostic, solver, relative-tracking-anchor, calibration-fit, external-pose-fit, motor-only-motion-gate, planar-pose-settle-gate, encoder-feedback-lead, shapes, grid-snake, grid-interpolation, grid-persistence, four-position-force-outer-loop, polyline-cap, absolute-frame, calibration-aggregate-quintic-ramp, timed-feedback-parser, feedback-time-alignment, continuous-draw-default, append-only-manual-test-recorder, raw-serial-recording, visual-overlay")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
