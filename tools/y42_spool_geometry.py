"""Pure cylindrical-winch payout geometry and offline inverse checks.

No serial, controller, file or network dependencies.  A smooth cylinder's
changing straight tangent span is not its full payout: the wrapped arc
changes as well.  The model assumes a fixed radius and one continuously
followed winding branch.  It does not observe winding layers, axial exit
height, rope stretch/slip or ball roll/pitch and must not be called
independent metrology.

planar_platform_payouts is the original degrees-at-the-API-boundary function.
platform_payouts_fixed_z is the corresponding fixed-exit-height model used by
the offline twin.
"""
from __future__ import annotations

import math
from typing import Any, Sequence


_RAD_TO_DEG = 180.0 / math.pi
_DEG_TO_RAD = math.pi / 180.0
_DEFAULT_EXIT_Z_MM = 10.5
_MAX_LINE_SEARCH_STEPS = 14


def _finite(value: Any, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label} must be finite") from exc
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def _pair(values: Sequence[float], label: str) -> tuple[float, float]:
    try:
        if len(values) != 2:
            raise ValueError(f"{label} must contain x and y")
        first, second = values
    except (TypeError, ValueError) as exc:
        if isinstance(exc, ValueError) and str(exc) == f"{label} must contain x and y":
            raise
        raise ValueError(f"{label} must contain x and y") from exc
    return _finite(first, f"{label}[0]"), _finite(second, f"{label}[1]")


def _quadruple(values: Sequence[Any], label: str) -> list[Any]:
    try:
        if len(values) != 4:
            raise ValueError(f"{label} must contain four values")
        return list(values)
    except (TypeError, ValueError) as exc:
        if isinstance(exc, ValueError) and str(exc) == f"{label} must contain four values":
            raise
        raise ValueError(f"{label} must contain four values") from exc


def _branch(branch: Any) -> int:
    if isinstance(branch, bool):
        raise ValueError("Winding branch must be -1 or +1")
    value = _finite(branch, "Winding branch")
    if value not in (-1.0, 1.0):
        raise ValueError("Winding branch must be -1 or +1")
    return int(value)


def _radii(radii_mm: Sequence[float]) -> list[float]:
    values = _quadruple(radii_mm, "radii_mm")
    result = [_finite(value, f"radii_mm[{index}]") for index, value in enumerate(values)]
    if any(value <= 0.0 for value in result):
        raise ValueError("Every cylinder radius must be positive")
    return result


def _branches(branches: Sequence[Any]) -> list[int]:
    return [_branch(value) for value in _quadruple(branches, "branches")]


def _references(reference_phi_rad: Sequence[float], label: str = "reference_phi_rad") -> list[float]:
    return [
        _finite(value, f"{label}[{index}]")
        for index, value in enumerate(_quadruple(reference_phi_rad, label))
    ]


def _offsets(attachment_offsets_xy: Sequence[Sequence[float]]) -> list[tuple[float, float]]:
    return [
        _pair(offset, f"attachment_offsets_xy[{index}]")
        for index, offset in enumerate(_quadruple(attachment_offsets_xy, "attachment_offsets_xy"))
    ]


def _axes(axes_xy: Sequence[Sequence[float]]) -> list[tuple[float, float]]:
    return [
        _pair(axis, f"axes_xy[{index}]")
        for index, axis in enumerate(_quadruple(axes_xy, "axes_xy"))
    ]


def _alpha_references(reference_alpha_rad: Sequence[float] | None) -> list[float]:
    values = [0.0] * 4 if reference_alpha_rad is None else _quadruple(
        reference_alpha_rad, "reference_alpha_rad"
    )
    return [
        _finite(value, f"reference_alpha_rad[{index}]")
        for index, value in enumerate(values)
    ]


def unwrap_near(angle_rad: float, reference_rad: float) -> float:
    """Choose the angle lift nearest a fixed/continuous reference (within pi)."""

    angle, reference = _finite(angle_rad, "Azimuth"), _finite(reference_rad, "reference")
    return reference + math.atan2(
        math.sin(angle - reference), math.cos(angle - reference)
    )


def cylindrical_payout(
    axis_xy: Sequence[float],
    endpoint_xy: Sequence[float],
    radius_mm: float,
    branch: int,
    *,
    reference_phi_rad: float,
    reference_alpha_rad: float = 0.0,
) -> dict:
    """Return planar payout, tangent point, and endpoint-space gradient.

    For D=P-C, rho=|D|, phi=arg(D), branch b in {-1,+1}:
    alpha = unwrap(phi) + b*acos(r/rho), s = sqrt(rho**2-r**2),
    L = s - b*r*(alpha-alpha_ref).
    """

    cx, cy = _pair(axis_xy, "Axis")
    px, py = _pair(endpoint_xy, "Endpoint")
    radius = _finite(radius_mm, "Cylinder radius")
    alpha_ref = _finite(reference_alpha_rad, "reference_alpha_rad")
    branch_value = _branch(branch)
    if radius <= 0.0:
        raise ValueError("Cylinder radius must be positive")
    dx, dy = px - cx, py - cy
    rho_squared = dx * dx + dy * dy
    if rho_squared <= radius * radius:
        raise ValueError("Endpoint is on or inside the winding cylinder")
    rho = math.sqrt(rho_squared)
    span = math.sqrt(rho_squared - radius * radius)
    phi = unwrap_near(math.atan2(dy, dx), reference_phi_rad)
    alpha = phi + branch_value * math.acos(radius / rho)
    tangent = (
        cx + (radius * radius * dx - branch_value * radius * span * dy) / rho_squared,
        cy + (radius * radius * dy + branch_value * radius * span * dx) / rho_squared,
    )
    gradient = (
        (span * dx + branch_value * radius * dy) / rho_squared,
        (span * dy - branch_value * radius * dx) / rho_squared,
    )
    length = span - branch_value * radius * (alpha - alpha_ref)
    if not all(math.isfinite(value) for value in (*tangent, *gradient, length)):
        raise ValueError("Planar cylinder geometry is not finite")
    return {
        "length_mm": length,
        "free_span_mm": span,
        "wrapped_arc_delta_mm": -branch_value * radius * (alpha - alpha_ref),
        "phi_rad": phi,
        "alpha_rad": alpha,
        "tangent_xy_mm": tangent,
        "gradient_xy": gradient,
        "dlength_dphi_mm_per_rad": -branch_value * radius,
        "branch": branch_value,
        "radius_mm": radius,
    }


def cylindrical_payout_fixed_z(
    axis_xy: Sequence[float],
    endpoint_xy: Sequence[float],
    radius_mm: float,
    branch: int,
    *,
    exit_z_mm: float,
    endpoint_z_mm: float,
    reference_phi_rad: float,
    reference_alpha_rad: float = 0.0,
) -> dict:
    """Return fixed-exit-height cylindrical payout and its 3D gradient."""

    exit_z = _finite(exit_z_mm, "exit_z_mm")
    endpoint_z = _finite(endpoint_z_mm, "endpoint_z_mm")
    planar = cylindrical_payout(
        axis_xy,
        endpoint_xy,
        radius_mm,
        branch,
        reference_phi_rad=reference_phi_rad,
        reference_alpha_rad=reference_alpha_rad,
    )
    span = planar["free_span_mm"]
    endpoint_x, endpoint_y = _pair(endpoint_xy, "Endpoint")
    axis_x, axis_y = _pair(axis_xy, "Axis")
    dx, dy = endpoint_x - axis_x, endpoint_y - axis_y
    dz = endpoint_z - exit_z
    free_span_3d = math.hypot(span, dz)
    if not math.isfinite(free_span_3d) or free_span_3d <= 0.0:
        raise ValueError("Fixed-exit cable span is not finite")
    planar_gx, planar_gy = planar["gradient_xy"]
    span_gx, span_gy = dx / span, dy / span
    scale = span / free_span_3d
    gradient_xy = (
        planar_gx + (scale - 1.0) * span_gx,
        planar_gy + (scale - 1.0) * span_gy,
    )
    gradient_xyz = (gradient_xy[0], gradient_xy[1], dz / free_span_3d)
    length = free_span_3d + planar["wrapped_arc_delta_mm"]
    if not all(math.isfinite(value) for value in (*gradient_xyz, length)):
        raise ValueError("Fixed-exit cable geometry is not finite")
    return {
        **planar,
        "length_mm": length,
        "planar_free_span_mm": span,
        "free_span_3d_mm": free_span_3d,
        "exit_z_mm": exit_z,
        "endpoint_z_mm": endpoint_z,
        "tangent_xyz_mm": (*planar["tangent_xy_mm"], exit_z),
        "endpoint_xyz_mm": (endpoint_x, endpoint_y, endpoint_z),
        "gradient_xy": gradient_xy,
        "gradient_xyz": gradient_xyz,
        "dlength_dz_mm": gradient_xyz[2],
    }


fixed_z_cylindrical_payout = cylindrical_payout_fixed_z
fixed_exit_cylindrical_payout = cylindrical_payout_fixed_z


def planar_platform_payouts(
    x_mm: float,
    y_mm: float,
    yaw_deg: float,
    axes_xy: Sequence[Sequence[float]],
    attachment_offsets_xy: Sequence[Sequence[float]],
    radii_mm: Sequence[float],
    branches: Sequence[int],
    reference_phi_rad: Sequence[float],
    reference_alpha_rad: Sequence[float] | None = None,
) -> dict:
    """Four full planar payouts and 4x3 Jacobian (x, y, yaw in radians)."""

    x, y = _finite(x_mm, "x_mm"), _finite(y_mm, "y_mm")
    yaw = _finite(yaw_deg, "yaw_deg") * _DEG_TO_RAD
    axes = _axes(axes_xy)
    offsets = _offsets(attachment_offsets_xy)
    radii = _radii(radii_mm)
    branch_values = _branches(branches)
    phi_references = _references(reference_phi_rad)
    alpha_references = _alpha_references(reference_alpha_rad)
    c, s = math.cos(yaw), math.sin(yaw)
    records, jacobian = [], []
    for index in range(4):
        bx, by = offsets[index]
        rx, ry = c * bx - s * by, s * bx + c * by
        record = cylindrical_payout(
            axes[index],
            (x + rx, y + ry),
            radii[index],
            branch_values[index],
            reference_phi_rad=phi_references[index],
            reference_alpha_rad=alpha_references[index],
        )
        gx, gy = record["gradient_xy"]
        jacobian.append([gx, gy, -gx * ry + gy * rx])
        records.append(record)
    return {
        "lengths_mm": [record["length_mm"] for record in records],
        "jacobian_xy_yaw_rad": jacobian,
        "cables": records,
    }


def platform_payouts_fixed_z(
    x_mm: float,
    y_mm: float,
    z_mm: float,
    yaw_rad: float,
    axes_xy: Sequence[Sequence[float]],
    attachment_offsets_xy: Sequence[Sequence[float]],
    radii_mm: Sequence[float],
    branches: Sequence[int],
    reference_phi_rad: Sequence[float],
    *,
    exit_z_mm: float = _DEFAULT_EXIT_Z_MM,
    reference_alpha_rad: Sequence[float] | None = None,
) -> dict:
    """Four fixed-exit-height payouts and a 4x4 x/y/z/yaw Jacobian."""

    x, y, z, yaw = (
        _finite(x_mm, "x_mm"),
        _finite(y_mm, "y_mm"),
        _finite(z_mm, "z_mm"),
        _finite(yaw_rad, "yaw_rad"),
    )
    exit_z = _finite(exit_z_mm, "exit_z_mm")
    axes = _axes(axes_xy)
    offsets = _offsets(attachment_offsets_xy)
    radii = _radii(radii_mm)
    branch_values = _branches(branches)
    phi_references = _references(reference_phi_rad)
    alpha_references = _alpha_references(reference_alpha_rad)
    c, s = math.cos(yaw), math.sin(yaw)
    records, jacobian = [], []
    endpoints_xy, endpoints_xyz, tangents_xy, tangents_xyz = [], [], [], []
    for index in range(4):
        bx, by = offsets[index]
        rx, ry = c * bx - s * by, s * bx + c * by
        endpoint_xy = (x + rx, y + ry)
        record = cylindrical_payout_fixed_z(
            axes[index],
            endpoint_xy,
            radii[index],
            branch_values[index],
            exit_z_mm=exit_z,
            endpoint_z_mm=z,
            reference_phi_rad=phi_references[index],
            reference_alpha_rad=alpha_references[index],
        )
        gx, gy, gz = record["gradient_xyz"]
        jacobian.append([gx, gy, gz, -gx * ry + gy * rx])
        records.append(record)
        endpoints_xy.append(endpoint_xy)
        endpoints_xyz.append((*endpoint_xy, z))
        tangents_xy.append(record["tangent_xy_mm"])
        tangents_xyz.append(record["tangent_xyz_mm"])
    if not all(math.isfinite(value) for row in jacobian for value in row):
        raise ValueError("Fixed-exit platform Jacobian is not finite")
    return {
        "lengths_mm": [record["length_mm"] for record in records],
        "jacobian_xyz_yaw_rad": jacobian,
        "jacobian_xy_yaw_rad": [[row[0], row[1], row[3]] for row in jacobian],
        "jacobian": jacobian,
        "tangent_xy_mm": tangents_xy,
        "tangent_xyz_mm": tangents_xyz,
        "endpoints_xy_mm": endpoints_xy,
        "endpoints_xyz_mm": endpoints_xyz,
        "cables": records,
        "pose": [x, y, z, yaw],
        "exit_z_mm": exit_z,
    }


fixed_z_platform_payouts = platform_payouts_fixed_z
fixed_exit_platform_payouts = platform_payouts_fixed_z


def _initial_xy(
    initial_xy: Sequence[float] | None,
    axes: Sequence[tuple[float, float]],
) -> tuple[float, float]:
    if initial_xy is None:
        return (
            sum(axis[0] for axis in axes) / len(axes),
            sum(axis[1] for axis in axes) / len(axes),
        )
    return _pair(initial_xy, "initial_xy")


def _validate_iterations(value: Any) -> int:
    if isinstance(value, bool):
        raise ValueError("max_iterations must be a positive integer")
    try:
        result = int(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("max_iterations must be a positive integer") from exc
    if result != value or result <= 0:
        raise ValueError("max_iterations must be a positive integer")
    return result


def solve_fixed_z_yaw_xy(
    target_lengths_mm: Sequence[float],
    z_mm: float,
    yaw_rad: float,
    axes_xy: Sequence[Sequence[float]],
    attachment_offsets_xy: Sequence[Sequence[float]],
    radii_mm: Sequence[float],
    branches: Sequence[int],
    reference_phi_rad: Sequence[float],
    *,
    exit_z_mm: float = _DEFAULT_EXIT_Z_MM,
    reference_alpha_rad: Sequence[float] | None = None,
    initial_xy: Sequence[float] | None = None,
    max_iterations: int = 60,
    tolerance_mm: float = 1e-8,
    step_tolerance_mm: float = 1e-10,
) -> dict:
    """Numerically solve platform X/Y with fixed Z and yaw.

    The target values are full payout lengths from platform_payouts_fixed_z;
    they may be negative because the wrapped-angle datum is arbitrary.  A
    damped Gauss-Newton step uses the shared analytic XY Jacobian.  A result is
    returned only after both requested residual and finite-value checks pass;
    nonconvergence raises ValueError instead of returning a plausible pose.
    """

    target_values = _quadruple(target_lengths_mm, "target_lengths_mm")
    targets = [
        _finite(value, f"target_lengths_mm[{index}]")
        for index, value in enumerate(target_values)
    ]
    tolerance = _finite(tolerance_mm, "tolerance_mm")
    step_tolerance = _finite(step_tolerance_mm, "step_tolerance_mm")
    if tolerance <= 0.0 or step_tolerance <= 0.0:
        raise ValueError("Solver tolerances must be positive")
    iterations = _validate_iterations(max_iterations)
    z = _finite(z_mm, "z_mm")
    yaw = _finite(yaw_rad, "yaw_rad")
    exit_z = _finite(exit_z_mm, "exit_z_mm")
    axes = _axes(axes_xy)
    offsets = _offsets(attachment_offsets_xy)
    radii = _radii(radii_mm)
    branch_values = _branches(branches)
    phi_references = _references(reference_phi_rad)
    alpha_references = _alpha_references(reference_alpha_rad)
    x, y = _initial_xy(initial_xy, axes)

    def evaluate(candidate_x: float, candidate_y: float) -> dict:
        return platform_payouts_fixed_z(
            candidate_x,
            candidate_y,
            z,
            yaw,
            axes,
            offsets,
            radii,
            branch_values,
            phi_references,
            exit_z_mm=exit_z,
            reference_alpha_rad=alpha_references,
        )

    current = evaluate(x, y)
    converged = False
    used_iterations = 0
    for iteration in range(1, iterations + 1):
        used_iterations = iteration
        residuals = [
            float(predicted - target)
            for predicted, target in zip(current["lengths_mm"], targets)
        ]
        if not all(math.isfinite(value) for value in residuals):
            raise ValueError("XY inverse residual is not finite")
        max_residual = max(abs(value) for value in residuals)
        if max_residual <= tolerance:
            converged = True
            break
        j = current["jacobian_xyz_yaw_rad"]
        h00 = h01 = h11 = g0 = g1 = 0.0
        for row, residual in zip(j, residuals):
            jx, jy = float(row[0]), float(row[1])
            h00 += jx * jx
            h01 += jx * jy
            h11 += jy * jy
            g0 += jx * residual
            g1 += jy * residual
        determinant = h00 * h11 - h01 * h01
        if not math.isfinite(determinant) or determinant <= 1e-14:
            raise ValueError("XY inverse Jacobian is singular")
        step_x = (-h11 * g0 + h01 * g1) / determinant
        step_y = (h01 * g0 - h00 * g1) / determinant
        step_norm = math.hypot(step_x, step_y)
        if not math.isfinite(step_norm):
            raise ValueError("XY inverse step is not finite")
        if step_norm > 24.0:
            scale = 24.0 / step_norm
            step_x *= scale
            step_y *= scale
            step_norm = 24.0
        objective = sum(value * value for value in residuals)
        accepted = None
        fraction = 1.0
        for _ in range(_MAX_LINE_SEARCH_STEPS):
            candidate_x, candidate_y = x + fraction * step_x, y + fraction * step_y
            try:
                candidate = evaluate(candidate_x, candidate_y)
            except ValueError:
                candidate = None
            if candidate is not None:
                candidate_residuals = [
                    float(predicted - target)
                    for predicted, target in zip(candidate["lengths_mm"], targets)
                ]
                candidate_objective = sum(value * value for value in candidate_residuals)
                if math.isfinite(candidate_objective) and candidate_objective < objective:
                    accepted = (candidate_x, candidate_y, candidate)
                    break
            fraction *= 0.5
        if accepted is None:
            raise ValueError("XY inverse did not converge: no improving finite step")
        x, y, current = accepted
        accepted_step = fraction * step_norm
        if accepted_step <= step_tolerance:
            residuals = [
                float(predicted - target)
                for predicted, target in zip(current["lengths_mm"], targets)
            ]
            if max(abs(value) for value in residuals) > tolerance:
                raise ValueError("XY inverse did not converge within tolerance")
    final_residuals = [
        float(predicted - target)
        for predicted, target in zip(current["lengths_mm"], targets)
    ]
    if not all(math.isfinite(value) for value in final_residuals):
        raise ValueError("XY inverse residual is not finite")
    residual_max = max(abs(value) for value in final_residuals)
    residual_rms = math.sqrt(sum(value * value for value in final_residuals) / 4.0)
    if not converged or residual_max > tolerance:
        raise ValueError(
            f"XY inverse did not converge after {used_iterations} iterations "
            f"(max residual {residual_max:.6g} mm)"
        )
    return {
        "x_mm": x,
        "y_mm": y,
        "xy_mm": [x, y],
        "z_mm": z,
        "yaw_rad": yaw,
        "predicted_lengths_mm": list(current["lengths_mm"]),
        "target_lengths_mm": targets,
        "residuals_mm": final_residuals,
        "residual_rms_mm": residual_rms,
        "residual_max_mm": residual_max,
        "iterations": used_iterations,
        "converged": True,
        "jacobian_xyz_yaw_rad": current["jacobian_xyz_yaw_rad"],
        "hardware_validated": False,
        "calibration_status": "not_calibrated",
    }


solve_fixed_z_yaw_xy_from_lengths = solve_fixed_z_yaw_xy
inverse_fixed_z_yaw_xy = solve_fixed_z_yaw_xy


def quantize_angle_deg(angle_deg: float, resolution_deg: float = 0.1) -> float:
    """Round one target angle to the nearest finite encoder grid."""

    angle = _finite(angle_deg, "angle_deg")
    resolution = _finite(resolution_deg, "resolution_deg")
    if resolution <= 0.0:
        raise ValueError("resolution_deg must be positive")
    units = angle / resolution
    if not math.isfinite(units):
        raise ValueError("Quantization grid index is not finite")
    # Symmetric half-away-from-zero avoids Python's banker-round surprise at
    # exactly half a 0.1-degree encoder count.
    rounded_units = math.floor(units + 0.5) if units >= 0.0 else math.ceil(units - 0.5)
    result = rounded_units * resolution
    if not math.isfinite(result):
        raise ValueError("Quantized angle is not finite")
    return result


def quantize_angles_deg(
    angles_deg: Sequence[float], resolution_deg: float = 0.1
) -> list[float]:
    values = _quadruple(angles_deg, "angles_deg")
    return [quantize_angle_deg(value, resolution_deg) for value in values]


def _angle_offsets(angle_offsets_deg: Sequence[float] | None) -> list[float]:
    if angle_offsets_deg is None:
        return [0.0] * 4
    values = _quadruple(angle_offsets_deg, "angle_offsets_deg")
    return [
        _finite(value, f"angle_offsets_deg[{index}]")
        for index, value in enumerate(values)
    ]


def lengths_to_angles_deg(
    lengths_mm: Sequence[float],
    radii_mm: Sequence[float],
    *,
    branches: Sequence[int] | None = None,
    angle_offsets_deg: Sequence[float] | None = None,
) -> list[float]:
    """Convert model lengths to a pure radius-based target-angle datum.

    This mathematical helper does not assert that a real motor follows this
    radius or winding direction.
    """

    lengths = [
        _finite(value, f"lengths_mm[{index}]")
        for index, value in enumerate(_quadruple(lengths_mm, "lengths_mm"))
    ]
    radii = _radii(radii_mm)
    branch_values = [1] * 4 if branches is None else _branches(branches)
    offsets = _angle_offsets(angle_offsets_deg)
    result = [
        branch_values[index] * lengths[index] / radii[index] * _RAD_TO_DEG + offsets[index]
        for index in range(4)
    ]
    if not all(math.isfinite(value) for value in result):
        raise ValueError("Target angle conversion is not finite")
    return result


def angles_to_lengths_mm(
    angles_deg: Sequence[float],
    radii_mm: Sequence[float],
    *,
    branches: Sequence[int] | None = None,
    angle_offsets_deg: Sequence[float] | None = None,
) -> list[float]:
    """Inverse of lengths_to_angles_deg for offline roundtrips."""

    angles = [
        _finite(value, f"angles_deg[{index}]")
        for index, value in enumerate(_quadruple(angles_deg, "angles_deg"))
    ]
    radii = _radii(radii_mm)
    branch_values = [1] * 4 if branches is None else _branches(branches)
    offsets = _angle_offsets(angle_offsets_deg)
    result = [
        branch_values[index]
        * (angles[index] - offsets[index])
        * _DEG_TO_RAD
        * radii[index]
        for index in range(4)
    ]
    if not all(math.isfinite(value) for value in result):
        raise ValueError("Length conversion is not finite")
    return result


def fixed_z_yaw_xy_roundtrip(
    x_mm: float,
    y_mm: float,
    z_mm: float,
    yaw_rad: float,
    axes_xy: Sequence[Sequence[float]],
    attachment_offsets_xy: Sequence[Sequence[float]],
    radii_mm: Sequence[float],
    branches: Sequence[int],
    reference_phi_rad: Sequence[float],
    *,
    exit_z_mm: float = _DEFAULT_EXIT_Z_MM,
    reference_alpha_rad: Sequence[float] | None = None,
    angle_resolution_deg: float | None = None,
    angle_offsets_deg: Sequence[float] | None = None,
    initial_xy: Sequence[float] | None = None,
    max_iterations: int = 60,
    tolerance_mm: float = 1e-8,
) -> dict:
    """Validate pose -> target angles -> lengths -> fixed-Z/Yaw XY roundtrip.

    angle_resolution_deg=None or 0 means no quantization.  Passing 0.1 models
    the target-angle grid used by offline encoder checks.  The report is pure
    geometric evidence and carries hardware_validated=False.
    """

    x, y, z, yaw, exit_z = (
        _finite(x_mm, "x_mm"),
        _finite(y_mm, "y_mm"),
        _finite(z_mm, "z_mm"),
        _finite(yaw_rad, "yaw_rad"),
        _finite(exit_z_mm, "exit_z_mm"),
    )
    radii = _radii(radii_mm)
    branch_values = _branches(branches)
    forward = platform_payouts_fixed_z(
        x,
        y,
        z,
        yaw,
        axes_xy,
        attachment_offsets_xy,
        radii,
        branch_values,
        reference_phi_rad,
        exit_z_mm=exit_z,
        reference_alpha_rad=reference_alpha_rad,
    )
    target_lengths = list(forward["lengths_mm"])
    target_angles = lengths_to_angles_deg(
        target_lengths,
        radii,
        branches=branch_values,
        angle_offsets_deg=angle_offsets_deg,
    )
    if angle_resolution_deg is None:
        quantized_angles = list(target_angles)
        resolution = None
    else:
        resolution = _finite(angle_resolution_deg, "angle_resolution_deg")
        if resolution < 0.0:
            raise ValueError("angle_resolution_deg must be nonnegative")
        quantized_angles = (
            list(target_angles)
            if resolution == 0.0
            else quantize_angles_deg(target_angles, resolution)
        )
    quantized_lengths = angles_to_lengths_mm(
        quantized_angles,
        radii,
        branches=branch_values,
        angle_offsets_deg=angle_offsets_deg,
    )
    solver_tolerance = _finite(tolerance_mm, "tolerance_mm")
    if resolution is not None and resolution > 0.0:
        # Four independently rounded cable targets are generally no longer
        # exactly coplanar in the two-unknown XY fit.  Permit the known
        # half-count length error while retaining a strict finite residual
        # requirement in solve_fixed_z_yaw_xy.
        quantization_length_bound = max(radii) * resolution * _DEG_TO_RAD * 0.5
        solver_tolerance = max(solver_tolerance, 1.5 * quantization_length_bound)
    solved = solve_fixed_z_yaw_xy(
        quantized_lengths,
        z,
        yaw,
        axes_xy,
        attachment_offsets_xy,
        radii,
        branch_values,
        reference_phi_rad,
        exit_z_mm=exit_z,
        reference_alpha_rad=reference_alpha_rad,
        initial_xy=initial_xy,
        max_iterations=max_iterations,
        tolerance_mm=solver_tolerance,
    )
    recovered_x, recovered_y = solved["x_mm"], solved["y_mm"]
    xy_error = math.hypot(recovered_x - x, recovered_y - y)
    length_error = [
        float(after - before)
        for after, before in zip(quantized_lengths, target_lengths)
    ]
    angle_error = [
        float(after - before)
        for after, before in zip(quantized_angles, target_angles)
    ]
    if not all(math.isfinite(value) for value in (*length_error, *angle_error, xy_error)):
        raise ValueError("Roundtrip report is not finite")
    return {
        "target_pose": [x, y, z, yaw],
        "recovered_xy_mm": [recovered_x, recovered_y],
        "target_lengths_mm": target_lengths,
        "target_angles_deg": target_angles,
        "quantized_angles_deg": quantized_angles,
        "roundtrip_lengths_mm": quantized_lengths,
        "angle_resolution_deg": resolution,
        "solver_tolerance_mm": solver_tolerance,
        "angle_error_deg": angle_error,
        "max_angle_error_deg": max(abs(value) for value in angle_error),
        "length_error_mm": length_error,
        "max_length_error_mm": max(abs(value) for value in length_error),
        "xy_error_mm": xy_error,
        "pose_error_mm": xy_error,
        "solver": solved,
        "converged": True,
        "hardware_validated": False,
        "calibration_status": "not_calibrated",
        "scope": "pure fixed-z/yaw geometry; real winding direction and effective radius unverified",
    }


validate_fixed_z_yaw_xy_roundtrip = fixed_z_yaw_xy_roundtrip
validate_target_angle_roundtrip = fixed_z_yaw_xy_roundtrip
